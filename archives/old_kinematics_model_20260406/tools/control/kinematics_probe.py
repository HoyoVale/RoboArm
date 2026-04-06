from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.config import HOME_MOVE_MS, SERIAL_BAUD, SERIAL_PORT
from src.kinematics import PalletizingArmKinematics
from tools.control.robot_arm_controller import RobotArmController


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Probe kinematics and move the robot arm by XYZ targets.")
    parser.add_argument("--port", default=SERIAL_PORT, help="Serial port, for example COM7")
    parser.add_argument("--baud", type=int, default=SERIAL_BAUD, help="Serial baud rate")

    subparsers = parser.add_subparsers(dest="action", required=True)

    ik_parser = subparsers.add_parser("ik", help="Only solve IK/FK without moving the arm")
    ik_parser.add_argument("--x", type=float, required=True)
    ik_parser.add_argument("--y", type=float, required=True)
    ik_parser.add_argument("--z", type=float, required=True)

    fk_parser = subparsers.add_parser("fk", help="Compute end-effector XYZ from servo angles")
    fk_parser.add_argument("--servo1", type=float, required=True)
    fk_parser.add_argument("--servo2", type=float, required=True)
    fk_parser.add_argument("--servo3", type=float, required=True)

    goto_parser = subparsers.add_parser("goto", help="Solve IK and move to XYZ")
    goto_parser.add_argument("--x", type=float, required=True)
    goto_parser.add_argument("--y", type=float, required=True)
    goto_parser.add_argument("--z", type=float, required=True)
    goto_parser.add_argument("--time", type=int, default=HOME_MOVE_MS, help="Move time in ms")

    angles_parser = subparsers.add_parser("angles", help="Move by explicit servo angles")
    angles_parser.add_argument("--servo1", type=int, required=True)
    angles_parser.add_argument("--servo2", type=int, required=True)
    angles_parser.add_argument("--servo3", type=int, required=True)
    angles_parser.add_argument("--time", type=int, default=HOME_MOVE_MS, help="Move time in ms")

    home_parser = subparsers.add_parser("home", help="Move to saved home pose")
    home_parser.add_argument("--time", type=int, default=HOME_MOVE_MS, help="Move time in ms")

    subparsers.add_parser("shell", help="Interactive calibration shell")
    return parser


def print_model(arm: PalletizingArmKinematics) -> None:
    print("[Model]")
    print(
        f"  links: L1={arm.L1:.1f}, L2={arm.L2:.1f}, L3={arm.L3:.1f}, "
        f"L4={arm.L4:.1f}, L5={arm.L5:.1f}, L6={arm.L6_nominal:.1f}"
    )
    print(
        f"  home : servo1={arm.servo1_home:.1f}, servo2={arm.servo2_home:.1f}, "
        f"servo3={arm.servo3_home:.1f}"
    )
    print(
        f"  range: servo1=[{arm.servo1_cmd_min:.1f}, {arm.servo1_cmd_max:.1f}], "
        f"servo2=[{arm.servo2_min:.1f}, {arm.servo2_max:.1f}], "
        f"servo3=[{arm.servo3_min:.1f}, {arm.servo3_max:.1f}]"
    )


def print_ik_result(arm: PalletizingArmKinematics, x_mm: float, y_mm: float, z_mm: float) -> tuple[int, int, int] | None:
    angles = arm.inverse_kinematics(x_mm, y_mm, z_mm)
    print(f"[IK] target=({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f})")
    if angles is None:
        print("  result: unreachable")
        return None

    servo1, servo2, servo3 = (int(round(value)) for value in angles)
    print(f"  servo=({servo1}, {servo2}, {servo3})")
    fk_xyz = arm.forward_kinematics(float(servo1), float(servo2), float(servo3))
    print(f"  fk   ={fk_xyz}")
    return servo1, servo2, servo3


def print_fk_result(arm: PalletizingArmKinematics, servo1: float, servo2: float, servo3: float) -> None:
    xyz = arm.forward_kinematics(servo1, servo2, servo3)
    print(f"[FK] servo=({servo1:.1f}, {servo2:.1f}, {servo3:.1f}) -> xyz={xyz}")


def move_to_xyz(controller: RobotArmController, arm: PalletizingArmKinematics, x_mm: float, y_mm: float, z_mm: float, move_time_ms: int) -> None:
    servo_angles = print_ik_result(arm, x_mm, y_mm, z_mm)
    if servo_angles is None:
        return
    servo1, servo2, servo3 = servo_angles
    sent = controller.move_angles({1: servo1, 2: servo2, 3: servo3}, move_time_ms)
    print("  sent:", repr(sent))


def print_help() -> None:
    print("commands:")
    print("  model")
    print("  ik x y z")
    print("  fk s1 s2 s3")
    print("  goto x y z [time=800]")
    print("  angles s1 s2 s3 [time=800]")
    print("  home [time=800]")
    print("  quit")


def run_shell(port: str, baud: int) -> int:
    arm = PalletizingArmKinematics()
    print_model(arm)
    with RobotArmController(port=port, baud=baud) as controller:
        print(f"[Shell] connected: {port} @ {baud}")
        print_help()
        while True:
            try:
                line = input("probe> ").strip()
            except EOFError:
                print()
                break

            if not line:
                continue

            parts = shlex.split(line)
            command = parts[0].lower()
            args = parts[1:]

            try:
                if command in ("quit", "exit"):
                    break
                if command == "help":
                    print_help()
                elif command == "model":
                    print_model(arm)
                elif command == "ik" and len(args) == 3:
                    print_ik_result(arm, float(args[0]), float(args[1]), float(args[2]))
                elif command == "fk" and len(args) == 3:
                    print_fk_result(arm, float(args[0]), float(args[1]), float(args[2]))
                elif command == "goto":
                    move_time_ms = HOME_MOVE_MS
                    numeric = []
                    for item in args:
                        if item.startswith("time="):
                            move_time_ms = int(item.split("=", 1)[1])
                        else:
                            numeric.append(float(item))
                    if len(numeric) != 3:
                        raise ValueError("goto 用法: goto x y z [time=800]")
                    move_to_xyz(controller, arm, numeric[0], numeric[1], numeric[2], move_time_ms)
                elif command == "angles":
                    move_time_ms = HOME_MOVE_MS
                    numeric = []
                    for item in args:
                        if item.startswith("time="):
                            move_time_ms = int(item.split("=", 1)[1])
                        else:
                            numeric.append(int(item))
                    if len(numeric) != 3:
                        raise ValueError("angles 用法: angles s1 s2 s3 [time=800]")
                    sent = controller.move_angles({1: numeric[0], 2: numeric[1], 3: numeric[2]}, move_time_ms)
                    print("  sent:", repr(sent))
                elif command == "home":
                    move_time_ms = HOME_MOVE_MS
                    if args:
                        if len(args) != 1 or not args[0].startswith("time="):
                            raise ValueError("home 用法: home [time=800]")
                        move_time_ms = int(args[0].split("=", 1)[1])
                    sent = controller.home(move_time_ms)
                    print("  sent:", repr(sent))
                else:
                    print("unknown command")
                    print_help()
            except Exception as exc:  # pragma: no cover - interactive utility
                print(f"[Error] {exc}")
    return 0


def main() -> int:
    args = build_parser().parse_args()
    arm = PalletizingArmKinematics()

    if args.action == "ik":
        print_model(arm)
        print_ik_result(arm, args.x, args.y, args.z)
        return 0

    if args.action == "fk":
        print_model(arm)
        print_fk_result(arm, args.servo1, args.servo2, args.servo3)
        return 0

    if args.action == "shell":
        return run_shell(args.port, args.baud)

    with RobotArmController(port=args.port, baud=args.baud) as controller:
        print_model(arm)
        if args.action == "goto":
            move_to_xyz(controller, arm, args.x, args.y, args.z, args.time)
        elif args.action == "angles":
            sent = controller.move_angles(
                {1: args.servo1, 2: args.servo2, 3: args.servo3},
                args.time,
            )
            print("  sent:", repr(sent))
        elif args.action == "home":
            sent = controller.home(args.time)
            print("  sent:", repr(sent))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
