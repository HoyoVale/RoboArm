"""
该脚本用于解析和计算 3 自由度吸盘机械臂的运动学，包括正运动学和逆运动学。

对于机械臂连杆和关节的建模，我已经写在了docs/机械臂参数文件夹下

"""

from __future__ import annotations

import math
from dataclasses import dataclass


# 使用dataclass装饰，定义一个平面解逆的结果的数据结构。
# 这个结构体包含三个属性，分别对应三个舵机的角度，单位是度。
# frozen=True 表示创建后是常量，无法修改。
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
    - 2 号舵机角度增大时，机械臂向后缩
    - 3 号舵机角度增大时，末端下降

    这个模型与实机校准一致：
    - servo1 = 60° 时，底座对准正前方
    - servo2 = 65° 时，L3 竖直
    - servo3 = 55° 时，L4 水平
    - inverse_kinematics() 返回的是可直接发给下位机的原始舵机角度
    - 下位机协议当前只接受 0~180 的无符号角度，不接受负角度
    """

    def __init__(self) -> None:
        # 连杆参数，单位 mm
        self.L1 = 74.0
        self.L2 = 23.0
        self.L3 = 143.5
        self.L4 = 160.0
        self.L5 = 35.0
        self.L6_nominal = 44.0

        # 实机校准零位
        self.servo1_home = 60.0
        self.servo2_home = 65.0
        self.servo3_home = 55.0

        # 这里使用的是可直接发给下位机的原始舵机角安全范围。
        self.servo1_cmd_min = 20.0
        self.servo1_cmd_max = 100.0
        self.servo2_min = 20.0
        self.servo2_max = 80.0
        self.servo3_min = 50.0
        self.servo3_max = 90.0

    def _servo1_from_xy(self, x_mm: float, y_mm: float) -> float:
        """
        compute the angle for servo1 based on the target x and y coordinates.
        根据目标点（在桌面上）的x,y坐标计算一号舵机需要转动的角度
        """
        yaw_deg = math.degrees(math.atan2(y_mm, x_mm))  # 计算arctan(y/x) 得到偏航角，转化为角度
        return self.servo1_home + yaw_deg               # 一号舵机角度增大，机械臂向左旋转，所以做加法

    def _wrist_target(self, radial_mm: float, z_mm: float) -> tuple[float, float]:
        """
        compute the target position for the wrist based on the radial and vertical coordinates.
        计算到达目标点时腕点的坐标，输入是目标点的径向距离和垂直高度，输出是腕点相对于肩点的径向和垂直坐标。
        """
        # 计算相对距离：吸盘接触点 -> 腕点
        wrist_r_mm = radial_mm - self.L5        # 吸盘中心相对于腕点向前 L5，单位mm 
        wrist_z_mm = z_mm + self.L6_nominal     # 吸盘中心相对于腕点向下 L6，垂直坐标，单位mm
        return wrist_r_mm, wrist_z_mm

    def _is_servo_valid(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> bool:
        """
        判断计算得到的舵机角度是否在安全范围内
        """
        return (
            self.servo1_cmd_min <= servo1_deg <= self.servo1_cmd_max
            and self.servo2_min <= servo2_deg <= self.servo2_max
            and self.servo3_min <= servo3_deg <= self.servo3_max
        )

    def inverse_kinematics(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float] | None:
        """
        解逆向运动学：给定末端目标点的笛卡尔坐标，计算对应的三个舵机角度。
        """
        radial_mm = math.hypot(x_mm, y_mm) # sqrt(x_mm^2 + y_mm^2), 计算目标点到原定的径向距离
        if radial_mm <= 1e-6:
            return None
        # 计算一号舵机转动的角度
        servo1_deg = self._servo1_from_xy(x_mm, y_mm)
        # 计算腕点的目标坐标
        wrist_r_mm, wrist_z_mm = self._wrist_target(radial_mm, z_mm)
        dx = wrist_r_mm - self.L2   # 计算腕点相对于肩点的径向距离
        dz = wrist_z_mm - self.L1   # 计算腕点相对于肩点的垂直距离

        distance_sq = dx * dx + dz * dz
        distance_mm = math.sqrt(distance_sq) 
        if distance_mm > (self.L3 + self.L4) or distance_mm < abs(self.L3 - self.L4):
            return None
        # 余弦定理先求 L3 与 L4 的相对夹角 θ_rel 的余弦值。
        cos_θ = (distance_sq - self.L3 * self.L3 - self.L4 * self.L4) / (2.0 * self.L3 * self.L4)
        cos_θ = max(-1.0, min(1.0, cos_θ))
        sin_θ_abs = math.sqrt(max(0.0, 1.0 - cos_θ * cos_θ))

        # γ: 肩点指向腕点的方向角
        γ_rad = math.atan2(dz, dx)
        # β: 肩到腕连线 与 L3 之间的夹角
        cos_β = (self.L3 * self.L3 + distance_sq - self.L4 * self.L4) / (2.0 * self.L3 * distance_mm)
        cos_β = max(-1.0, min(1.0, cos_β))
        β_rad = math.acos(cos_β)

        solutions: list[_PlanarSolution] = []
        for elbow_sign in (-1.0, 1.0):
            # θ_rel 是 L4 相对 L3 的夹角。
            # 由于同一个腕点通常对应“肘上/肘下”两组解，所以 sin(θ_rel) 有正负两种情况。
            θ_rel = math.atan2(elbow_sign * sin_θ_abs, cos_θ)

            # 肩角更直观地写成：肩到腕方向角 γ 再减/加 一个几何补偿角 β。
            # elbow_sign = +1 -> η = γ - β
            # elbow_sign = -1 -> η = γ + β
            η_rad = γ_rad - elbow_sign * β_rad

            η_deg = math.degrees(η_rad)
            θ_deg = math.degrees(θ_rel)

            # 2 号舵机零位时 L3 竖直向上，对应数学角 90°。
            # 舵机角度增大时，L3 向后缩；向前伸时舵机角度减小。
            servo2_deg = self.servo2_home + (η_deg - 90.0)
            # 3 号舵机零位时 L4 水平向前，此时 L4 相对 L3 的夹角为 -90°。
            # 舵机角度增大时，相对夹角更负，L4 向下压，末端下降。
            servo3_deg = self.servo3_home + (-90.0 - θ_deg)

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
        """
        解正向运动学：给定三个舵机的角度，计算末端目标点的笛卡尔坐标。 
        """
        # 计算底座旋转角的弧度值
        yaw_rad = math.radians(servo1_deg - self.servo1_home)
        # 计算肩关节的绝对角度 η 和肘关节的相对夹角 θ
        θ1_deg = - servo2_deg + self.servo2_home
        # 3 号舵机角度增大时，末端下降，所以 θ 的计算需要反过来。
        θ2_deg = servo3_deg - self.servo3_home

        θ1_rad = math.radians(θ1_deg)
        θ2_rad = math.radians(θ2_deg)
        # 计算前臂的绝对角度，等于肩关节绝对角度加上肘关节夹角
        forearm_rad = θ1_rad + θ2_rad

        radial_mm = (
            self.L2
            + self.L3 * math.sin(θ1_rad) 
            + self.L4 * math.cos(forearm_rad)
            + self.L5
        )
        z_mm = (
            self.L1
            + self.L3 * math.cos(θ1_rad) 
            - self.L4 * math.sin(forearm_rad)
            - self.L6_nominal
        )

        x_mm = radial_mm * math.cos(yaw_rad)
        y_mm = radial_mm * math.sin(yaw_rad)
        return round(x_mm, 1), round(y_mm, 1), round(z_mm, 1)


if __name__ == "__main__":
    arm = PalletizingArmKinematics()
    home_angles = (60.0, 65.0, 55.0)
    print("Home FK:", arm.forward_kinematics(*home_angles))
    sample_targets = (
        (170.0, 0.0, 70.0),
        (230.0, 0.0, 70.0),
        (200.0, 200.0, 70.0),
    )
    for target in sample_targets:
        result = arm.inverse_kinematics(*target)
        print("IK", target, "->", result)
