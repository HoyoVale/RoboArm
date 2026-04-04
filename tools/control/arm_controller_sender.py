import argparse
import shlex
import sys

try:
    from tools.control.robot_arm_controller import (
        DEFAULT_GRAB_MS,
        HOME_MOVE_TIME_MS,
        RobotArmController,
        build_angle_command,
        build_home_command,
        build_pulse_command,
        ensure_crlf,
    )
except ImportError:
    from robot_arm_controller import (
        DEFAULT_GRAB_MS,
        HOME_MOVE_TIME_MS,
        RobotArmController,
        build_angle_command,
        build_home_command,
        build_pulse_command,
        ensure_crlf,
    )


def build_text_for_action(args: argparse.Namespace) -> str:
    if args.action == "angle":
        return build_angle_command(_parse_cli_items(args.item), args.time)
    if args.action == "pulse":
        return build_pulse_command(_parse_cli_items(args.item), args.time)
    if args.action == "pump":
        return ensure_crlf("#PUMP1" if args.state == "on" else "#PUMP0")
    if args.action == "valve":
        return ensure_crlf("#VALVE1" if args.state == "close" else "#VALVE0")
    if args.action == "grab":
        return ensure_crlf(f"#GRAB{args.time}")
    if args.action == "release":
        return ensure_crlf("#RELEASE")
    if args.action == "stop":
        return ensure_crlf("#STOP")
    if args.action == "home":
        return build_home_command(args.time)
    if args.action == "savehome":
        return ensure_crlf("#SAVEHOME")
    if args.action == "raw":
        return ensure_crlf(args.text)
    raise SystemExit(f"unknown action: {args.action}")


def _parse_cli_items(items: list[str]) -> list[tuple[int, int]]:
    parsed: list[tuple[int, int]] = []

    if not items:
        raise SystemExit("至少提供一个 --item")

    for item in items:
        try:
            sid_text, value_text = item.split(":", 1)
            parsed.append((int(sid_text), int(value_text)))
        except ValueError as exc:
            raise SystemExit(f"无效的 --item: {item}，应为 1:90 这种格式") from exc

    return parsed


def parse_shell_time(tokens: list[str], default_ms: int) -> tuple[list[tuple[int, int]], int]:
    items: list[tuple[int, int]] = []
    move_time_ms = default_ms

    for token in tokens:
        if token.startswith("time="):
            try:
                move_time_ms = int(token.split("=", 1)[1])
            except ValueError as exc:
                raise SystemExit(f"无效的时间参数: {token}") from exc
        else:
            try:
                servo_text, value_text = token.split(":", 1)
                items.append((int(servo_text), int(value_text)))
            except ValueError as exc:
                raise SystemExit(f"无效的项目参数: {token}") from exc

    return items, move_time_ms


def parse_shell_command(line: str) -> tuple[str | None, str | None]:
    tokens = shlex.split(line)
    if not tokens:
        return None, None

    command = tokens[0].lower()
    args = tokens[1:]

    if command in ("quit", "exit"):
        return "exit", None
    if command == "help":
        return "help", None
    if command == "angle":
        items, move_time_ms = parse_shell_time(args, HOME_MOVE_TIME_MS)
        return "send", build_angle_command(items, move_time_ms)
    if command == "pulse":
        items, move_time_ms = parse_shell_time(args, HOME_MOVE_TIME_MS)
        return "send", build_pulse_command(items, move_time_ms)
    if command == "pump":
        if len(args) != 1 or args[0] not in ("on", "off"):
            raise SystemExit("pump 用法: pump on|off")
        return "send", ensure_crlf("#PUMP1" if args[0] == "on" else "#PUMP0")
    if command == "valve":
        if len(args) != 1 or args[0] not in ("close", "open"):
            raise SystemExit("valve 用法: valve close|open")
        return "send", ensure_crlf("#VALVE1" if args[0] == "close" else "#VALVE0")
    if command == "grab":
        move_time_ms = DEFAULT_GRAB_MS
        if len(args) == 1:
            try:
                move_time_ms = int(args[0])
            except ValueError as exc:
                raise SystemExit("grab 用法: grab 500") from exc
        elif len(args) > 1:
            raise SystemExit("grab 用法: grab 500")
        return "send", ensure_crlf(f"#GRAB{move_time_ms}")
    if command == "release":
        return "send", ensure_crlf("#RELEASE")
    if command == "stop":
        return "send", ensure_crlf("#STOP")
    if command == "home":
        move_time_ms = HOME_MOVE_TIME_MS
        if len(args) == 1:
            try:
                move_time_ms = int(args[0])
            except ValueError as exc:
                raise SystemExit("home 用法: home 800") from exc
        elif len(args) > 1:
            raise SystemExit("home 用法: home 800")
        return "send", build_home_command(move_time_ms)
    if command == "savehome":
        if args:
            raise SystemExit("savehome 用法: savehome")
        return "send", ensure_crlf("#SAVEHOME")
    if command == "raw":
        if not args:
            raise SystemExit("raw 用法: raw \"#1A90T800\"")
        return "send", ensure_crlf(" ".join(args))

    raise SystemExit(f"未知命令: {command}")


def print_shell_help() -> None:
    print("可用命令:")
    print("  angle 1:90 2:120 3:60 time=1000")
    print("  pulse 1:1500 2:1600 time=800")
    print("  pump on|off")
    print("  valve close|open")
    print("  grab 500")
    print("  release")
    print("  stop")
    print("  home 800")
    print("  savehome")
    print("  raw \"#1A90T800\"")
    print("  help")
    print("  quit")


def run_shell(port: str, baud: int) -> int:
    with RobotArmController(port=port, baud=baud) as controller:
        print(f"serial shell connected: {port} @ {baud}")
        print("输入 help 查看命令，输入 quit 退出。")
        while True:
            try:
                line = input("arm> ")
            except EOFError:
                print()
                break

            try:
                action, text = parse_shell_command(line)
            except SystemExit as exc:
                print(exc)
                continue

            if action is None:
                continue
            if action == "exit":
                break
            if action == "help":
                print_shell_help()
                continue

            sent = controller.send_raw(text)
            print("sent:", repr(sent))

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Send commands to the STC15 robot arm lower controller.")
    parser.add_argument("--port", default="COM5", help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate")
    subparsers = parser.add_subparsers(dest="action", required=True)

    angle_parser = subparsers.add_parser("angle", help="Move one or more joints by angle")
    angle_parser.add_argument("--item", action="append", default=[], help="SERVO:ANGLE, for example 1:90")
    angle_parser.add_argument("--time", type=int, default=HOME_MOVE_TIME_MS, help="Move time in ms")

    pulse_parser = subparsers.add_parser("pulse", help="Move one or more joints by pulse width")
    pulse_parser.add_argument("--item", action="append", default=[], help="SERVO:PULSE, for example 1:1500")
    pulse_parser.add_argument("--time", type=int, default=HOME_MOVE_TIME_MS, help="Move time in ms")

    pump_parser = subparsers.add_parser("pump", help="Control pump directly")
    pump_parser.add_argument("state", choices=["on", "off"])

    valve_parser = subparsers.add_parser("valve", help="Control valve directly")
    valve_parser.add_argument("state", choices=["close", "open"])

    grab_parser = subparsers.add_parser("grab", help="Pump for a while, then auto-stop and hold")
    grab_parser.add_argument("--time", type=int, default=DEFAULT_GRAB_MS, help="Pump run time in ms")

    subparsers.add_parser("release", help="Open valve and release the object")
    subparsers.add_parser("stop", help="Stop current servo interpolation")
    home_parser = subparsers.add_parser("home", help="Move all three joints to the saved home pose")
    home_parser.add_argument("--time", type=int, default=HOME_MOVE_TIME_MS, help="Move time in ms")
    subparsers.add_parser("savehome", help="Save the current pose as the persisted home pose")
    subparsers.add_parser("shell", help="Keep the serial port open for interactive calibration")

    raw_parser = subparsers.add_parser("raw", help="Send a raw line and auto-append CRLF")
    raw_parser.add_argument("text")

    args = parser.parse_args()

    if args.action == "shell":
        return run_shell(args.port, args.baud)

    text = build_text_for_action(args)
    with RobotArmController(port=args.port, baud=args.baud) as controller:
        sent = controller.send_raw(text)
    print("sent:", repr(sent))
    return 0


if __name__ == "__main__":
    sys.exit(main())
