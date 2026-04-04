from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class _PlanarSolution:
    servo1_deg: float
    servo2_deg: float
    servo3_deg: float


class PalletizingArmKinematics:
    """
    3 自由度吸盘机械臂的平面逆运动学模型。

    建模约定：
    - 原点在底座旋转中心投影到桌面的点
    - X 正前方，Y 左侧，Z 桌面向上
    - L1/L2 把肩关节基点放到 (r=L2, z=L1)
    - L5 始终水平，L6 始终竖直向下
    - 2 号舵机对应 L3 的绝对角
    - 3 号舵机对应 L4 相对 L3 的夹角，而不是 L4 的绝对角
    - 2 号舵机角度增大时，机械臂向前伸
    - 3 号舵机角度增大时，末端下降

    这个模型与实机校准一致：
    - servo2 = 70° 时，L3 竖直
    - servo3 = 60° 时，L4 水平
    """

    def __init__(self) -> None:
        # 连杆参数，单位 mm
        self.L1 = 70.0
        self.L2 = 20.0
        self.L3 = 155.0
        self.L4 = 160.0
        self.L5 = 35.0
        self.L6_nominal = 38.0

        # 实机校准零位
        self.servo1_home = 60.0
        self.servo2_home = 70.0
        self.servo3_home = 60.0

        # 文档给出的安全范围
        self.servo1_doc_min = -30.0
        self.servo1_doc_max = 150.0
        self.servo2_min = 50.0
        self.servo2_max = 150.0
        self.servo3_min = 40.0
        self.servo3_max = 130.0

        # 下位机角度接口使用 0~180，所以上位机还要满足硬件可发送范围。
        self.servo1_cmd_min = 0.0
        self.servo1_cmd_max = 150.0

    def _servo1_from_xy(self, x_mm: float, y_mm: float) -> float:
        yaw_deg = math.degrees(math.atan2(y_mm, x_mm))
        return self.servo1_home + yaw_deg

    def _wrist_target(self, radial_mm: float, z_mm: float) -> tuple[float, float]:
        # 吸盘接触点 -> 腕点
        wrist_r_mm = radial_mm - self.L5
        wrist_z_mm = z_mm + self.L6_nominal
        return wrist_r_mm, wrist_z_mm

    def _is_servo_valid(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> bool:
        return (
            self.servo1_cmd_min <= servo1_deg <= self.servo1_cmd_max
            and self.servo2_min <= servo2_deg <= self.servo2_max
            and self.servo3_min <= servo3_deg <= self.servo3_max
        )

    def inverse_kinematics(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float] | None:
        radial_mm = math.hypot(x_mm, y_mm)
        if radial_mm <= 1e-6:
            return None

        servo1_deg = self._servo1_from_xy(x_mm, y_mm)

        wrist_r_mm, wrist_z_mm = self._wrist_target(radial_mm, z_mm)
        dx = wrist_r_mm - self.L2
        dz = wrist_z_mm - self.L1

        distance_sq = dx * dx + dz * dz
        distance_mm = math.sqrt(distance_sq)
        if distance_mm > (self.L3 + self.L4) or distance_mm < abs(self.L3 - self.L4):
            return None

        cos_relative = (distance_sq - self.L3 * self.L3 - self.L4 * self.L4) / (2.0 * self.L3 * self.L4)
        cos_relative = max(-1.0, min(1.0, cos_relative))
        sin_relative_abs = math.sqrt(max(0.0, 1.0 - cos_relative * cos_relative))

        solutions: list[_PlanarSolution] = []
        for elbow_sign in (-1.0, 1.0):
            theta_rel = math.atan2(elbow_sign * sin_relative_abs, cos_relative)
            shoulder_rad = math.atan2(dz, dx) - math.atan2(
                self.L4 * math.sin(theta_rel),
                self.L3 + self.L4 * math.cos(theta_rel),
            )

            shoulder_deg = math.degrees(shoulder_rad)
            relative_deg = math.degrees(theta_rel)

            # 2 号舵机零位时 L3 竖直向上，对应数学角 90°。
            # 舵机角度增大时，L3 向前摆，整体更前伸。
            servo2_deg = self.servo2_home + (90.0 - shoulder_deg)
            # 3 号舵机零位时 L4 水平向前，此时 L4 相对 L3 的夹角为 -90°。
            # 舵机角度增大时，相对夹角更负，L4 向下压，末端下降。
            servo3_deg = self.servo3_home + (-90.0 - relative_deg)

            if self._is_servo_valid(servo1_deg, servo2_deg, servo3_deg):
                solutions.append(
                    _PlanarSolution(
                        servo1_deg=servo1_deg,
                        servo2_deg=servo2_deg,
                        servo3_deg=servo3_deg,
                    )
                )

        if not solutions:
            return None

        best = min(
            solutions,
            key=lambda item: (
                abs(item.servo1_deg - self.servo1_home)
                + abs(item.servo2_deg - self.servo2_home)
                + abs(item.servo3_deg - self.servo3_home)
            ),
        )
        return (
            round(best.servo1_deg, 1),
            round(best.servo2_deg, 1),
            round(best.servo3_deg, 1),
        )

    def forward_kinematics(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> tuple[float, float, float]:
        yaw_rad = math.radians(servo1_deg - self.servo1_home)
        shoulder_deg = 90.0 - (servo2_deg - self.servo2_home)
        relative_deg = -90.0 - (servo3_deg - self.servo3_home)

        shoulder_rad = math.radians(shoulder_deg)
        forearm_rad = shoulder_rad + math.radians(relative_deg)

        radial_mm = (
            self.L2
            + self.L3 * math.cos(shoulder_rad)
            + self.L4 * math.cos(forearm_rad)
            + self.L5
        )
        z_mm = (
            self.L1
            + self.L3 * math.sin(shoulder_rad)
            + self.L4 * math.sin(forearm_rad)
            - self.L6_nominal
        )

        x_mm = radial_mm * math.cos(yaw_rad)
        y_mm = radial_mm * math.sin(yaw_rad)
        return round(x_mm, 1), round(y_mm, 1), round(z_mm, 1)


if __name__ == "__main__":
    arm = PalletizingArmKinematics()
    home_angles = (60.0, 70.0, 60.0)
    print("Home FK:", arm.forward_kinematics(*home_angles))
    sample_targets = (
        (170.0, 0.0, 70.0),
        (230.0, 0.0, 70.0),
        (200.0, 200.0, 70.0),
    )
    for target in sample_targets:
        result = arm.inverse_kinematics(*target)
        print("IK", target, "->", result)
