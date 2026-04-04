#include "iap15w4k61s4.h"
#include <intrins.h>

#define FOSC                  11059200UL
#define UART_BAUD             9600UL
#define TIMER0_TICK_US        20U
#define SERVO_FRAME_US        20000U
#define SERVO_FRAME_TICKS     (SERVO_FRAME_US / TIMER0_TICK_US)
#define TIMER0_RELOAD         (65536UL - (FOSC / (1000000UL / TIMER0_TICK_US)))
#define UART1_RELOAD          (256UL - (FOSC / 12UL / 32UL / UART_BAUD))

#define JOINT_COUNT           3U
#define OUTPUT_COUNT          5U
#define SERVO_MIN_US          500U
#define SERVO_MAX_US          2500U
#define CMD_BUF_SIZE          64U
#define DEFAULT_GRAB_MS       500U

#define JOINT1_HOME_DEG       60U
#define JOINT2_HOME_DEG       70U
#define JOINT3_HOME_DEG       60U

#define PUMP_OUTPUT_ID        4U
#define VALVE_OUTPUT_ID       5U
#define PUMP_PWM_ON_US        2000U
#define PUMP_PWM_OFF_US       1000U
#define VALVE_CLOSED_PWM_US   2000U
#define VALVE_OPEN_PWM_US     1000U

#define EEPROM_BASE_ADDR      0x0400U
#define EEPROM_SECTOR_SIZE    512U
#define EEPROM_RECORD_SIZE    10U
#define EEPROM_MAGIC          0xA5U
#define EEPROM_VERSION        0x01U
#define EEPROM_COMMIT         0x5AU
#define EEPROM_CMD_IDLE       0U
#define EEPROM_CMD_READ       1U
#define EEPROM_CMD_PROGRAM    2U
#define EEPROM_CMD_ERASE      3U
#define EEPROM_ENABLE         0x83U /* SYSCLK < 12MHz, see STC15 datasheet */

#define JOINT1_REVERSED       0
#define JOINT2_REVERSED       0
#define JOINT3_REVERSED       0

static volatile u16 frame_tick = 0;
static volatile u16 grab_frames_left = 0;
static volatile bit cmd_ready = 0;
static volatile u8 cmd_len = 0;
static volatile u8 rx_len = 0;
static volatile u8 xdata cmd_buf[CMD_BUF_SIZE];

static u16 servo_current_us[OUTPUT_COUNT];
static u16 servo_target_us[OUTPUT_COUNT];
static u16 servo_frames_left[OUTPUT_COUNT];
static u16 servo_pulse_ticks[OUTPUT_COUNT];
static u16 persisted_pose_us[JOINT_COUNT];
static bit persisted_pose_valid = 0;
static u16 eeprom_next_addr = EEPROM_BASE_ADDR;
static u8 xdata parse_buf[CMD_BUF_SIZE];
static u8 xdata parse_ids[JOINT_COUNT];
static u16 xdata parse_values[JOINT_COUNT];

static u16 clamp_servo_us(u16 pulse_us);
static u16 default_joint_home_pulse(u8 joint_id);
static void servo_schedule_move(u8 servo_id, u16 pulse_us, u16 move_time_ms);
static u16 angle_to_pulse(u8 servo_id, u16 angle);

static void timer0_reload(void)
{
    TH0 = (u8)(TIMER0_RELOAD >> 8);
    TL0 = (u8)TIMER0_RELOAD;
}

static void iap_idle(void)
{
    IAP_CONTR = 0;
    IAP_CMD = EEPROM_CMD_IDLE;
    IAP_TRIG = 0;
    IAP_ADDRH = 0x80;
    IAP_ADDRL = 0;
}

static u8 iap_read_byte(u16 addr)
{
    u8 dat;

    IAP_CONTR = EEPROM_ENABLE;
    IAP_CMD = EEPROM_CMD_READ;
    IAP_ADDRL = (u8)addr;
    IAP_ADDRH = (u8)(addr >> 8);
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();
    dat = IAP_DATA;
    iap_idle();
    return dat;
}

static void iap_program_byte(u16 addr, u8 dat)
{
    bit old_ea = EA;

    EA = 0;
    IAP_CONTR = EEPROM_ENABLE;
    IAP_CMD = EEPROM_CMD_PROGRAM;
    IAP_ADDRL = (u8)addr;
    IAP_ADDRH = (u8)(addr >> 8);
    IAP_DATA = dat;
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();
    iap_idle();
    EA = old_ea;
}

static void iap_erase_sector(u16 addr)
{
    bit old_ea = EA;

    EA = 0;
    IAP_CONTR = EEPROM_ENABLE;
    IAP_CMD = EEPROM_CMD_ERASE;
    IAP_ADDRL = (u8)addr;
    IAP_ADDRH = (u8)(addr >> 8);
    IAP_TRIG = 0x5A;
    IAP_TRIG = 0xA5;
    _nop_();
    iap_idle();
    EA = old_ea;
}

static u8 pose_checksum(const u16* pose)
{
    u8 i;
    u8 sum = (u8)(EEPROM_MAGIC ^ EEPROM_VERSION);

    for (i = 0; i < JOINT_COUNT; i++)
    {
        sum ^= (u8)pose[i];
        sum ^= (u8)(pose[i] >> 8);
    }

    return sum;
}

static void copy_pose(u16* dst, const u16* src)
{
    u8 i;

    for (i = 0; i < JOINT_COUNT; i++)
    {
        dst[i] = src[i];
    }
}

static u8 pose_equals(const u16* left, const u16* right)
{
    u8 i;

    for (i = 0; i < JOINT_COUNT; i++)
    {
        if (left[i] != right[i])
        {
            return 0;
        }
    }

    return 1;
}

static void eeprom_write_pose_record(const u16* pose)
{
    u8 i;
    u8 checksum;
    u16 end_addr = (u16)(EEPROM_BASE_ADDR + EEPROM_SECTOR_SIZE);

    if (persisted_pose_valid && pose_equals(pose, persisted_pose_us))
    {
        return;
    }

    if ((u16)(eeprom_next_addr + EEPROM_RECORD_SIZE) > end_addr)
    {
        iap_erase_sector(EEPROM_BASE_ADDR);
        eeprom_next_addr = EEPROM_BASE_ADDR;
    }

    checksum = pose_checksum(pose);

    iap_program_byte(eeprom_next_addr + 0U, EEPROM_MAGIC);
    iap_program_byte(eeprom_next_addr + 1U, EEPROM_VERSION);
    for (i = 0; i < JOINT_COUNT; i++)
    {
        iap_program_byte((u16)(eeprom_next_addr + 2U + (u16)i * 2U), (u8)pose[i]);
        iap_program_byte((u16)(eeprom_next_addr + 3U + (u16)i * 2U), (u8)(pose[i] >> 8));
    }
    iap_program_byte(eeprom_next_addr + 8U, checksum);
    iap_program_byte(eeprom_next_addr + 9U, EEPROM_COMMIT);

    copy_pose(persisted_pose_us, pose);
    persisted_pose_valid = 1;
    eeprom_next_addr = (u16)(eeprom_next_addr + EEPROM_RECORD_SIZE);
}

static void persist_current_pose(void)
{
    eeprom_write_pose_record(servo_current_us);
}

static void move_to_home_pose(u16 move_time_ms)
{
    u8 i;
    u16 pulse_us;

    for (i = 0; i < JOINT_COUNT; i++)
    {
        pulse_us = persisted_pose_valid ? persisted_pose_us[i] : default_joint_home_pulse((u8)(i + 1U));
        servo_schedule_move((u8)(i + 1U), pulse_us, move_time_ms);
    }
}

static void eeprom_load_last_pose(void)
{
    u16 addr = EEPROM_BASE_ADDR;
    u16 end_addr = (u16)(EEPROM_BASE_ADDR + EEPROM_SECTOR_SIZE);
    u16 pose[JOINT_COUNT];
    u8 checksum;
    u8 i;

    persisted_pose_valid = 0;
    eeprom_next_addr = EEPROM_BASE_ADDR;

    while ((u16)(addr + EEPROM_RECORD_SIZE) <= end_addr)
    {
        if (iap_read_byte(addr + 9U) != EEPROM_COMMIT)
        {
            break;
        }

        if (iap_read_byte(addr + 0U) != EEPROM_MAGIC ||
            iap_read_byte(addr + 1U) != EEPROM_VERSION)
        {
            break;
        }

        for (i = 0; i < JOINT_COUNT; i++)
        {
            pose[i] = (u16)iap_read_byte((u16)(addr + 2U + (u16)i * 2U));
            pose[i] |= (u16)((u16)iap_read_byte((u16)(addr + 3U + (u16)i * 2U)) << 8);
            pose[i] = clamp_servo_us(pose[i]);
        }

        checksum = iap_read_byte(addr + 8U);
        if (checksum != pose_checksum(pose))
        {
            break;
        }

        copy_pose(persisted_pose_us, pose);
        persisted_pose_valid = 1;
        eeprom_next_addr = (u16)(addr + EEPROM_RECORD_SIZE);
        addr = (u16)(addr + EEPROM_RECORD_SIZE);
    }
}

static u16 clamp_servo_us(u16 pulse_us)
{
    if (pulse_us < SERVO_MIN_US)
    {
        pulse_us = SERVO_MIN_US;
    }
    else if (pulse_us > SERVO_MAX_US)
    {
        pulse_us = SERVO_MAX_US;
    }

    return pulse_us;
}

static u16 pulse_us_to_ticks(u16 pulse_us)
{
    pulse_us = clamp_servo_us(pulse_us);
    return (u16)((pulse_us + (TIMER0_TICK_US / 2U)) / TIMER0_TICK_US);
}

static u16 default_joint_home_pulse(u8 joint_id)
{
    if (joint_id == 1U)
    {
        return angle_to_pulse(1U, JOINT1_HOME_DEG);
    }

    if (joint_id == 2U)
    {
        return angle_to_pulse(2U, JOINT2_HOME_DEG);
    }

    return angle_to_pulse(3U, JOINT3_HOME_DEG);
}

static void servo_write_pin(u8 index, bit level)
{
    switch (index)
    {
        case 0: P50 = level; break; /* joint 1 / base */
        case 1: P34 = level; break; /* joint 2 / big arm */
        case 2: P04 = level; break; /* joint 3 / small arm */
        case 3: P53 = level; break; /* pump switch module */
        default: P05 = level; break; /* valve switch module */
    }
}

static void all_servo_pins_low(void)
{
    P50 = 0;
    P34 = 0;
    P04 = 0;
    P53 = 0;
    P05 = 0;
}

static void output_write_immediate(u8 output_id, u16 pulse_us)
{
    u8 index;

    if (output_id < 1U || output_id > OUTPUT_COUNT)
    {
        return;
    }

    index = (u8)(output_id - 1U);
    pulse_us = clamp_servo_us(pulse_us);
    servo_current_us[index] = pulse_us;
    servo_target_us[index] = pulse_us;
    servo_frames_left[index] = 0;
    servo_pulse_ticks[index] = pulse_us_to_ticks(pulse_us);
}

static void pump_write(u8 on)
{
    output_write_immediate(PUMP_OUTPUT_ID, on ? PUMP_PWM_ON_US : PUMP_PWM_OFF_US);
}

static void valve_write_closed(u8 closed)
{
    output_write_immediate(VALVE_OUTPUT_ID, closed ? VALVE_CLOSED_PWM_US : VALVE_OPEN_PWM_US);
}

static void gpio_init(void)
{
    /* P0.4 / P0.5 */
    P0M0 |= 0x30;
    P0M1 &= (u8)~0x30;

    /* P3.4 */
    P3M0 |= 0x10;
    P3M1 &= (u8)~0x10;

    /* P5.0 / P5.2 / P5.3 */
    P5M0 |= 0x0D;
    P5M1 &= (u8)~0x0D;

    all_servo_pins_low();
    pump_write(0);
    valve_write_closed(0);
    P52 = 0; /* spare blue port 5 */
}

static void servo_init_state(void)
{
    u8 i;
    u16 init_us;

    for (i = 0; i < JOINT_COUNT; i++)
    {
        init_us = persisted_pose_valid ? persisted_pose_us[i] : default_joint_home_pulse((u8)(i + 1U));
        servo_current_us[i] = init_us;
        servo_target_us[i] = init_us;
        servo_frames_left[i] = 0;
        servo_pulse_ticks[i] = pulse_us_to_ticks(init_us);
    }

    output_write_immediate(PUMP_OUTPUT_ID, PUMP_PWM_OFF_US);
    output_write_immediate(VALVE_OUTPUT_ID, VALVE_OPEN_PWM_US);
}

static void servo_apply_frame_step(void)
{
    u8 i;

    for (i = 0; i < OUTPUT_COUNT; i++)
    {
        if (servo_frames_left[i] > 0)
        {
            int diff = (int)servo_target_us[i] - (int)servo_current_us[i];

            if (servo_frames_left[i] == 1 || diff == 0)
            {
                servo_current_us[i] = servo_target_us[i];
                servo_frames_left[i] = 0;
            }
            else
            {
                int step = diff / (int)servo_frames_left[i];
                if (step == 0)
                {
                    step = (diff > 0) ? 1 : -1;
                }
                servo_current_us[i] = (u16)((int)servo_current_us[i] + step);
                servo_frames_left[i] = (u16)(servo_frames_left[i] - 1U);
            }
        }

        servo_pulse_ticks[i] = pulse_us_to_ticks(servo_current_us[i]);
    }
}

static void servo_schedule_move(u8 servo_id, u16 pulse_us, u16 move_time_ms)
{
    u8 index;
    u16 frames;

    if (servo_id < 1 || servo_id > OUTPUT_COUNT)
    {
        return;
    }

    index = (u8)(servo_id - 1U);
    pulse_us = clamp_servo_us(pulse_us);
    frames = move_time_ms / 20U;
    if (frames == 0)
    {
        frames = 1;
    }

    servo_target_us[index] = pulse_us;
    servo_frames_left[index] = frames;
}

static void servo_stop_all(void)
{
    u8 i;

    for (i = 0; i < OUTPUT_COUNT; i++)
    {
        servo_target_us[i] = servo_current_us[i];
        servo_frames_left[i] = 0;
    }
}

static u8 is_digit(u8 ch)
{
    return (u8)(ch >= '0' && ch <= '9');
}

static u16 parse_number(const u8* buf, u8* pos, u8 len, u8* ok)
{
    u16 value = 0;
    u8 has_digit = 0;

    while (*pos < len && is_digit(buf[*pos]))
    {
        has_digit = 1;
        value = (u16)(value * 10U + (u16)(buf[*pos] - '0'));
        (*pos)++;
    }

    *ok = has_digit;
    return value;
}

static u16 angle_to_pulse(u8 servo_id, u16 angle)
{
    u16 pulse;

    if (angle > 180U)
    {
        angle = 180U;
    }

    if ((servo_id == 1U && JOINT1_REVERSED) ||
        (servo_id == 2U && JOINT2_REVERSED) ||
        (servo_id == 3U && JOINT3_REVERSED))
    {
        angle = (u16)(180U - angle);
    }

    pulse = (u16)(SERVO_MIN_US + ((u32)(SERVO_MAX_US - SERVO_MIN_US) * angle) / 180UL);
    return clamp_servo_us(pulse);
}

static u8 starts_with(const u8* buf, u8 len, const char* text)
{
    u8 i = 0;

    while (text[i] != 0)
    {
        if (i >= len || buf[i] != (u8)text[i])
        {
            return 0;
        }
        i++;
    }

    return 1;
}

static void start_grab(u16 pump_time_ms)
{
    u16 frames = pump_time_ms / 20U;

    if (frames == 0)
    {
        frames = 1;
    }

    valve_write_closed(1);
    pump_write(1);
    grab_frames_left = frames;
}

static void release_object(void)
{
    grab_frames_left = 0;
    pump_write(0);
    valve_write_closed(0);
}

static void handle_special_command(void)
{
    u8 ok;
    u8 pos;
    u16 value;

    if (starts_with(parse_buf, cmd_len, "#STOP"))
    {
        servo_stop_all();
        return;
    }

    if (starts_with(parse_buf, cmd_len, "#SAVEHOME"))
    {
        persist_current_pose();
        return;
    }

    if (starts_with(parse_buf, cmd_len, "#HOME"))
    {
        pos = 5;
        value = parse_number(parse_buf, &pos, cmd_len, &ok);
        if (!ok)
        {
            value = 800U;
        }
        move_to_home_pose(value);
        return;
    }

    if (starts_with(parse_buf, cmd_len, "#PUMP"))
    {
        if (cmd_len >= 6U)
        {
            pump_write((u8)(parse_buf[5] == '1'));
            if (parse_buf[5] == '1')
            {
                grab_frames_left = 0;
            }
        }
        return;
    }

    if (starts_with(parse_buf, cmd_len, "#VALVE"))
    {
        if (cmd_len >= 7U)
        {
            valve_write_closed((u8)(parse_buf[6] == '1'));
        }
        return;
    }

    if (starts_with(parse_buf, cmd_len, "#GRAB"))
    {
        pos = 5;
        value = parse_number(parse_buf, &pos, cmd_len, &ok);
        if (!ok)
        {
            value = DEFAULT_GRAB_MS;
        }
        start_grab(value);
        return;
    }

    if (starts_with(parse_buf, cmd_len, "#RELEASE"))
    {
        release_object();
    }
}

static void handle_motion_command(void)
{
    u8 pos = 0;
    u8 i;
    u8 count = 0;
    u8 ok;
    u16 move_time_ms = 0;
    u8 mode;
    u16 value;

    while (pos < cmd_len)
    {
        if (parse_buf[pos] != '#')
        {
            return;
        }
        pos++;

        if (count >= JOINT_COUNT)
        {
            return;
        }

        parse_ids[count] = (u8)parse_number(parse_buf, &pos, cmd_len, &ok);
        if (!ok || pos >= cmd_len || parse_ids[count] < 1U || parse_ids[count] > JOINT_COUNT)
        {
            return;
        }

        mode = parse_buf[pos++];
        value = parse_number(parse_buf, &pos, cmd_len, &ok);
        if (!ok)
        {
            return;
        }

        if (mode == 'A')
        {
            parse_values[count] = angle_to_pulse(parse_ids[count], value);
        }
        else if (mode == 'P')
        {
            parse_values[count] = clamp_servo_us(value);
        }
        else
        {
            return;
        }

        count++;

        if (pos < cmd_len && parse_buf[pos] == 'T')
        {
            pos++;
            move_time_ms = parse_number(parse_buf, &pos, cmd_len, &ok);
            if (!ok)
            {
                return;
            }
            break;
        }
    }

    if (move_time_ms == 0)
    {
        move_time_ms = 100U;
    }

    for (i = 0; i < count; i++)
    {
        servo_schedule_move(parse_ids[i], parse_values[i], move_time_ms);
    }

}

static void handle_ascii_command(void)
{
    u8 i;

    if (!cmd_ready)
    {
        return;
    }

    EA = 0;
    for (i = 0; i < cmd_len; i++)
    {
        parse_buf[i] = cmd_buf[i];
    }
    parse_buf[cmd_len] = 0;
    cmd_ready = 0;
    EA = 1;

    if (cmd_len == 0)
    {
        return;
    }

    if (starts_with(parse_buf, cmd_len, "#STOP") ||
        starts_with(parse_buf, cmd_len, "#SAVEHOME") ||
        starts_with(parse_buf, cmd_len, "#HOME") ||
        starts_with(parse_buf, cmd_len, "#PUMP") ||
        starts_with(parse_buf, cmd_len, "#VALVE") ||
        starts_with(parse_buf, cmd_len, "#GRAB") ||
        starts_with(parse_buf, cmd_len, "#RELEASE"))
    {
        handle_special_command();
        return;
    }

    handle_motion_command();
}

static void uart1_init(void)
{
    AUXR &= (u8)~AUXR_UART1_T2;
    TMOD &= 0x0F;
    TMOD |= 0x20;
    TH1 = (u8)UART1_RELOAD;
    TL1 = (u8)UART1_RELOAD;
    SCON = 0x50;
    TR1 = 1;
    ES = 1;
}

static void timer0_init(void)
{
    AUXR |= AUXR_T0_1T;
    TMOD &= 0xF0;
    TMOD |= 0x01;
    timer0_reload();
    ET0 = 1;
    TR0 = 1;
}

void timer0_isr(void) interrupt 1
{
    u8 i;

    timer0_reload();

    if (frame_tick == 0)
    {
        servo_apply_frame_step();
        for (i = 0; i < OUTPUT_COUNT; i++)
        {
            servo_write_pin(i, 1);
        }

        if (grab_frames_left > 0)
        {
            grab_frames_left--;
            if (grab_frames_left == 0)
            {
                pump_write(0);
            }
        }
    }

    for (i = 0; i < OUTPUT_COUNT; i++)
    {
        if (frame_tick == servo_pulse_ticks[i])
        {
            servo_write_pin(i, 0);
        }
    }

    frame_tick++;
    if (frame_tick >= SERVO_FRAME_TICKS)
    {
        frame_tick = 0;
    }
}

void uart1_isr(void) interrupt 4
{
    u8 value;

    if (RI)
    {
        RI = 0;
        value = SBUF;

        if (!cmd_ready)
        {
            if (value == '\n')
            {
                if (rx_len > 0 && cmd_buf[rx_len - 1U] == '\r')
                {
                    rx_len--;
                }
                cmd_len = rx_len;
                rx_len = 0;
                cmd_ready = 1;
            }
            else if (value != 0)
            {
                if (rx_len < (u8)(CMD_BUF_SIZE - 1U))
                {
                    cmd_buf[rx_len++] = value;
                }
                else
                {
                    rx_len = 0;
                }
            }
        }
    }

    if (TI)
    {
        TI = 0;
    }
}

void main(void)
{
    gpio_init();
    eeprom_load_last_pose();
    servo_init_state();
    uart1_init();
    timer0_init();

    EA = 1;

    while (1)
    {
        handle_ascii_command();
    }
}
