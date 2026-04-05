# STC15 Robot Arm Lower Controller

This is the first dedicated lower-controller firmware for the borrowed board.

MCU:

- `IAP15W4K61S4`

Baud:

- `9600 8N1`

Fixed wiring in this firmware:

- joint `1` / base rotation -> blue port `0` -> `P50`
- joint `2` / big arm reach -> blue port `1` -> `P34`
- joint `3` / small arm lift -> blue port `2` -> `P04`
- pump control -> blue port `3` -> `P53`
- valve control -> blue port `4` -> `P05`
- blue port `5` / `P52` is spare

Servo commands:

- single joint by angle:
  - `#1A90T800\r\n`
- multiple joints by angle:
  - `#1A90#2A120#3A60T1000\r\n`
- low-level pulse form is also supported:
  - `#1P1500T800\r\n`
  - `#1P1500#2P1600#3P1400T800\r\n`
- stop current servo interpolation:
  - `#STOP\r\n`

Vacuum commands:

- pump on:
  - `#PUMP1\r\n`
- pump off:
  - `#PUMP0\r\n`
- valve closed / hold:
  - `#VALVE1\r\n`
- valve open / release path:
  - `#VALVE0\r\n`
- grab with timed pump run, then auto stop pump and keep valve closed:
  - `#GRAB500\r\n`
- release object:
  - `#RELEASE\r\n`

Notes:

- default angle mapping is linear `0..180 -> 500..2500us`
- current default home pose in code is `joint1=60`, `joint2=65`, `joint3=55`
- use `#SAVEHOME` to save the current pose to internal EEPROM
- on reboot, the controller restores the saved home pose instead of always forcing `90/90/90`
- if an old saved home pose already exists in EEPROM, it overrides the compile-time default until you save the new pose again
- use `#HOME800` to move back to the saved home pose in `800ms`
- pump/valve are now driven as PWM-triggered switch modules instead of plain GPIO levels
- current default switch pulses are:
  - pump on `2000us`, pump off `1000us`
  - valve closed `2000us`, valve open `1000us`
- if one joint turns in the opposite direction, flip the corresponding `JOINTx_REVERSED` macro in `main.c`
- this firmware is intended to be the lower machine, with a Windows vision program acting as the upper machine

Recommended calibration workflow:

- keep the serial port open while calibrating:
  - `python tools\arm_controller_sender.py --port COM7 shell`
- then type commands such as:
  - `angle 1:90 2:90 3:90 time=1000`
  - `angle 1:10 time=800`
  - `home 800`
  - `savehome`
