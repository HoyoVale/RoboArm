from __future__ import annotations

import time
from typing import Iterable

try:
    import serial
except ImportError as exc:
    raise SystemExit(
        "pyserial 未安装，请先运行: python -m pip install pyserial"
    ) from exc


HOME_MOVE_TIME_MS = 1200
DEFAULT_GRAB_MS = 500
SERVO_MIN_ID = 1
SERVO_MAX_ID = 3
ANGLE_MIN = 0
ANGLE_MAX = 180
PULSE_MIN = 500
PULSE_MAX = 2500


def ensure_crlf(text: str) -> str:
    return text if text.endswith("\r\n") else text + "\r\n"


def _normalize_items(items: dict[int, int] | Iterable[tuple[int, int]]) -> list[tuple[int, int]]:
    if isinstance(items, dict):
        normalized = list(items.items())
    else:
        normalized = list(items)

    if not normalized:
        raise ValueError("至少提供一个舵机目标")

    return normalized


def _validate_move_time_ms(move_time_ms: int) -> None:
    if move_time_ms < 20 or move_time_ms > 30000:
        raise ValueError("time 建议在 20 到 30000 之间")


def angle_to_pulse(angle_deg: float) -> int:
    if angle_deg < ANGLE_MIN or angle_deg > ANGLE_MAX:
        raise ValueError("angle 只能是 0 到 180")
    pulse_span = PULSE_MAX - PULSE_MIN
    return int(round(PULSE_MIN + (float(angle_deg) / 180.0) * pulse_span))


def angles_to_pulses(items: dict[int, float] | Iterable[tuple[int, float]]) -> list[tuple[int, int]]:
    normalized = _normalize_items(items)
    return [(servo_id, angle_to_pulse(value)) for servo_id, value in normalized]


def _build_servo(items: dict[int, int] | Iterable[tuple[int, int]], mode: str, move_time_ms: int) -> str:
    _validate_move_time_ms(move_time_ms)
    normalized = _normalize_items(items)

    parts: list[str] = []
    for servo_id, value in normalized:
        if servo_id < SERVO_MIN_ID or servo_id > SERVO_MAX_ID:
            raise ValueError("servo_id 只能是 1 到 3")

        if mode == "A":
            if value < ANGLE_MIN or value > ANGLE_MAX:
                raise ValueError("angle 只能是 0 到 180")
        elif mode == "P":
            if value < PULSE_MIN or value > PULSE_MAX:
                raise ValueError("pulse 只能是 500 到 2500")
        else:
            raise ValueError(f"unknown mode: {mode}")

        parts.append(f"#{servo_id}{mode}{value}")

    return ensure_crlf("".join(parts) + f"T{move_time_ms}")


def build_angle_command(items: dict[int, int] | Iterable[tuple[int, int]], move_time_ms: int = HOME_MOVE_TIME_MS) -> str:
    return _build_servo(items, "A", move_time_ms)


def build_pulse_command(items: dict[int, int] | Iterable[tuple[int, int]], move_time_ms: int = HOME_MOVE_TIME_MS) -> str:
    return _build_servo(items, "P", move_time_ms)


def build_angle_as_pulse_command(
    items: dict[int, float] | Iterable[tuple[int, float]],
    move_time_ms: int = HOME_MOVE_TIME_MS,
) -> str:
    return build_pulse_command(angles_to_pulses(items), move_time_ms)


def build_home_command(move_time_ms: int = HOME_MOVE_TIME_MS) -> str:
    _validate_move_time_ms(move_time_ms)
    return ensure_crlf(f"#HOME{move_time_ms}")


class RobotArmController:
    def __init__(self, port: str = "COM5", baud: int = 9600, timeout: float = 0.2):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self._ser: serial.Serial | None = None

    def open(self) -> None:
        if self._ser is not None and self._ser.is_open:
            return

        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baud
        ser.timeout = self.timeout
        ser.dsrdtr = False
        ser.rtscts = False
        ser.dtr = False
        ser.rts = False
        ser.open()
        self._ser = ser

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def __enter__(self) -> "RobotArmController":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send_raw(self, text: str) -> str:
        self.open()
        assert self._ser is not None
        text = ensure_crlf(text)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._ser.write(text.encode("ascii"))
        self._ser.flush()
        time.sleep(0.05)
        return text

    def move_angles(self, items: dict[int, int] | Iterable[tuple[int, int]], move_time_ms: int = HOME_MOVE_TIME_MS) -> str:
        return self.send_raw(build_angle_command(items, move_time_ms))

    def move_pulses(self, items: dict[int, int] | Iterable[tuple[int, int]], move_time_ms: int = HOME_MOVE_TIME_MS) -> str:
        return self.send_raw(build_pulse_command(items, move_time_ms))

    def move_angles_precise(
        self,
        items: dict[int, float] | Iterable[tuple[int, float]],
        move_time_ms: int = HOME_MOVE_TIME_MS,
    ) -> str:
        return self.send_raw(build_angle_as_pulse_command(items, move_time_ms))

    def pump_on(self) -> str:
        return self.send_raw("#PUMP1")

    def pump_off(self) -> str:
        return self.send_raw("#PUMP0")

    def valve_close(self) -> str:
        return self.send_raw("#VALVE1")

    def valve_open(self) -> str:
        return self.send_raw("#VALVE0")

    def grab(self, pump_time_ms: int = DEFAULT_GRAB_MS) -> str:
        if pump_time_ms < 1 or pump_time_ms > 30000:
            raise ValueError("pump_time_ms 建议在 1 到 30000 之间")
        return self.send_raw(f"#GRAB{pump_time_ms}")

    def release(self) -> str:
        return self.send_raw("#RELEASE")

    def stop(self) -> str:
        return self.send_raw("#STOP")

    def home(self, move_time_ms: int = HOME_MOVE_TIME_MS) -> str:
        return self.send_raw(build_home_command(move_time_ms))

    def save_home(self) -> str:
        return self.send_raw("#SAVEHOME")
