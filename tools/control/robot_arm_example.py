from tools.robot_arm_controller import RobotArmController


def main() -> None:
    with RobotArmController(port="COM5", baud=9600) as arm:
        arm.home(800)
        arm.move_angles({1: 60, 2: 70, 3: 60}, 800)
        arm.valve_close()
        arm.pump_on()
        arm.pump_off()
        arm.valve_open()


if __name__ == "__main__":
    main()
