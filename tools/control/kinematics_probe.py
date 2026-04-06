from __future__ import annotations

import argparse
import shlex
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.config import HOME_MOVE_MS, SERIAL_BAUD, SERIAL_PORT
from src.kinematics import ForwardState, InverseCandidate, PalletizingArmKinematics
from tools.control.robot_arm_controller import RobotArmController


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Probe closed-form kinematics and move the robot arm by XYZ targets.")
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
        f"  A: A1={arm.A1:.1f}, A2={arm.A2:.1f}, A3={arm.A3:.1f}, A4={arm.A4:.1f}, "
        f"A5={arm.A5:.1f}, A6={arm.A6:.1f}, A7={arm.A7:.1f}, A8={arm.A8:.1f}"
    )
    print(f"  B: B1={arm.B1:.1f}, B2={arm.B2:.1f}, B3={arm.B3:.1f}")
    print(
        f"  home : servo1={arm.servo1_home:.1f}, servo2={arm.servo2_home:.1f}, "
        f"servo3={arm.servo3_home:.1f}"
    )
    print(
        f"  range: servo1=[{arm.servo1_cmd_min:.1f}, {arm.servo1_cmd_max:.1f}], "
        f"servo2=[{arm.servo2_min:.1f}, {arm.servo2_max:.1f}], "
        f"servo3=[{arm.servo3_min:.1f}, {arm.servo3_max:.1f}]"
    )


def print_forward_state(state: ForwardState) -> None:
    print(
        f"[FK] servo=({state.servo1_deg:.1f}, {state.servo2_deg:.1f}, {state.servo3_deg:.1f}) "
        f"-> xyz=({state.x_mm:.1f}, {state.y_mm:.1f}, {state.z_mm:.1f})"
    )
    print(
        f"  yaw={state.yaw_deg:.2f}deg theta1={state.theta1_deg:.2f}deg "
        f"theta2={state.theta2_deg:.2f}deg"
    )
    print(f"  planar: radial={state.radial_mm:.2f} z={state.z_mm:.2f}")
    print(
        f"  branch={state.branch_name} "
        f"left={state.left_branch}/{state.left_candidates} "
        f"right={state.right_branch}/{state.right_candidates} "
        f"residual={state.branch_residual_mm:.2f}mm"
    )


def print_candidates(candidates: list[InverseCandidate]) -> None:
    if not candidates:
        print("  result: unreachable")
        return

    for candidate in candidates:
        print(
            f"  branch={candidate.branch} "
            f"servo=({candidate.servo1_deg:.1f}, {candidate.servo2_deg:.1f}, {candidate.servo3_deg:.1f}) "
            f"theta1={candidate.theta1_deg:.2f} theta2={candidate.theta2_deg:.2f} "
            f"fk=({candidate.fk_x_mm:.1f}, {candidate.fk_y_mm:.1f}, {candidate.fk_z_mm:.1f}) "
            f"err={candidate.error_mm:.2f}mm residual={candidate.branch_residual_mm:.2f}mm"
        )


def print_ik_result(arm: PalletizingArmKinematics, x_mm: float, y_mm: float, z_mm: float) -> tuple[int, int, int] | None:
    print(f"[IK] target=({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f})")
    candidates = arm.inverse_kinematics_debug(x_mm, y_mm, z_mm)
    print_candidates(candidates)
    best = arm.inverse_kinematics(x_mm, y_mm, z_mm)
    if best is None:
        return None

    servo1, servo2, servo3 = (int(round(value)) for value in best)
    print(f"  selected=({servo1}, {servo2}, {servo3})")
    state = arm.forward_state(float(servo1), float(servo2), float(servo3))
    print(
        f"  selected_fk=({state.x_mm:.1f}, {state.y_mm:.1f}, {state.z_mm:.1f}) "
        f"theta1={state.theta1_deg:.2f} theta2={state.theta2_deg:.2f}"
    )
    return servo1, servo2, servo3


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
                    state = arm.forward_state(float(args[0]), float(args[1]), float(args[2]))
                    print_forward_state(state)
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
                    state = arm.forward_state(float(numeric[0]), float(numeric[1]), float(numeric[2]))
                    print_forward_state(state)
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
            except Exception as exc:
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
        print_forward_state(arm.forward_state(args.servo1, args.servo2, args.servo3))
        return 0

    if args.action == "shell":
        return run_shell(args.port, args.baud)

    with RobotArmController(port=args.port, baud=args.baud) as controller:
        print_model(arm)
        if args.action == "goto":
            move_to_xyz(controller, arm, args.x, args.y, args.z, args.time)
        elif args.action == "angles":
            print_forward_state(arm.forward_state(args.servo1, args.servo2, args.servo3))
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
