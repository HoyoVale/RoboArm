"""
三自由度吸盘机械臂的主链解析运动学模型。

权威机构说明见：
docs/机械臂参数/README.md

本文件采用的运行时建模约定：
- 全局输出坐标系：
  原点取 L1 与地面的接触点，
  X 指向机械臂前方，Y 指向机械臂左侧，Z 竖直向上。
- 内部局部平面坐标系：
  原点取 S（即二号舵机轴中心），
  局部 x 指向前方水平，局部 z 指向竖直向上。
- yaw_deg = servo1_deg - servo1_home
  一号舵机带动整机绕竖直轴旋转。
- theta1_deg = servo2_home - servo2_deg
  theta1 表示 L3 相对竖直方向的转角。
  servo2 增大时 theta1 变小，L3 向后收缩。
- phi4_deg = servo3_deg - servo3_home
  phi4 表示 L4 相对 home 位姿的下压角。
  phi4 = 0 时 L4 水平；
  phi4 为正表示 L4 下压；
  servo3 增大时 phi4 变大，L4 下压。

运行时只使用主链 L1~L6：
- S4 = L3 末端
- S5 = S4 + L4 方向向量
- seat = S5 + 水平 L5
- end = seat - 竖直 L6

文档里的 A/B 连杆仅保留为机构说明，不参与运行时 FK/IK 求解。
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path


if __package__ in (None, ""):
    PROJECT_ROOT = Path(__file__).resolve().parents[1]
    if str(PROJECT_ROOT) not in sys.path:
        sys.path.insert(0, str(PROJECT_ROOT))

from src.config import (
    KINEMATICS_L6_NOMINAL_MM,
    SERVO1_YAW_BIAS_DEG,
    SERVO1_YAW_SCALE,
    SERVO2_THETA1_BIAS_DEG,
    SERVO2_THETA1_SCALE,
    SERVO3_PHI4_BIAS_DEG,
    SERVO3_PHI4_SCALE,
)


@dataclass(frozen=True)
class ForwardState:
    servo1_deg: float
    servo2_deg: float
    servo3_deg: float
    yaw_deg: float
    theta1_deg: float
    phi4_deg: float
    radial_mm: float
    z_mm: float
    x_mm: float
    y_mm: float
    branch_name: str
    left_branch: str
    right_branch: str
    branch_residual_mm: float
    left_candidates: int
    right_candidates: int


@dataclass(frozen=True)
class InverseCandidate:
    branch: str
    servo1_deg: float
    servo2_deg: float
    servo3_deg: float
    theta1_deg: float
    phi4_deg: float
    fk_x_mm: float
    fk_y_mm: float
    fk_z_mm: float
    error_mm: float
    branch_residual_mm: float


class PalletizingArmKinematics:
    def __init__(self) -> None:
        self.L1 = 74.0
        self.L2 = 23.0
        self.L3 = 148.0
        self.L4 = 160.0
        self.L5 = 33.0
        self.L6_nominal = float(KINEMATICS_L6_NOMINAL_MM)

        self.servo1_home = 60.0
        self.servo2_home = 65.0
        self.servo3_home = 55.0

        self.servo1_cmd_min = 20.0
        self.servo1_cmd_max = 100.0
        self.servo2_min = 20.0
        self.servo2_max = 80.0
        self.servo3_min = 50.0
        self.servo3_max = 90.0

        self.servo1_yaw_scale = float(SERVO1_YAW_SCALE)
        self.servo1_yaw_bias_deg = float(SERVO1_YAW_BIAS_DEG)
        self.servo2_theta1_scale = float(SERVO2_THETA1_SCALE)
        self.servo2_theta1_bias_deg = float(SERVO2_THETA1_BIAS_DEG)
        self.servo3_phi4_scale = float(SERVO3_PHI4_SCALE)
        self.servo3_phi4_bias_deg = float(SERVO3_PHI4_BIAS_DEG)
        if abs(self.servo1_yaw_scale) < 1e-9:
            raise ValueError("SERVO1_YAW_SCALE 不能为 0")
        if abs(self.servo2_theta1_scale) < 1e-9:
            raise ValueError("SERVO2_THETA1_SCALE 不能为 0")
        if abs(self.servo3_phi4_scale) < 1e-9:
            raise ValueError("SERVO3_PHI4_SCALE 不能为 0")

    def _is_servo_valid(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> bool:
        return (
            self.servo1_cmd_min <= servo1_deg <= self.servo1_cmd_max
            and self.servo2_min <= servo2_deg <= self.servo2_max
            and self.servo3_min <= servo3_deg <= self.servo3_max
        )

    def _servo1_from_xy(self, x_mm: float, y_mm: float) -> float:
        if abs(x_mm) < 1e-9 and abs(y_mm) < 1e-9:
            return self.servo1_home
        yaw_deg = math.degrees(math.atan2(y_mm, x_mm))
        return self.servo1_home + (yaw_deg - self.servo1_yaw_bias_deg) / self.servo1_yaw_scale

    def _yaw_from_servo1(self, servo1_deg: float) -> float:
        return self.servo1_yaw_scale * (servo1_deg - self.servo1_home) + self.servo1_yaw_bias_deg

    def _theta1_from_servo2(self, servo2_deg: float) -> float:
        return self.servo2_theta1_scale * (self.servo2_home - servo2_deg) + self.servo2_theta1_bias_deg

    def _phi4_from_servo3(self, servo3_deg: float) -> float:
        return self.servo3_phi4_scale * (servo3_deg - self.servo3_home) + self.servo3_phi4_bias_deg

    def _alpha4_from_phi4(self, phi4_deg: float) -> float:
        """
        将 L4 的“下压为正”角度转换成几何方向角。

        alpha4 以局部平面 +x 水平前方为 0，逆时针为正（向上为正）。
        """
        return -phi4_deg

    @staticmethod
    def _circle_intersections(
        center0: tuple[float, float],
        radius0: float,
        center1: tuple[float, float],
        radius1: float,
    ) -> list[tuple[float, float]]:
        x0, z0 = center0
        x1, z1 = center1
        dx = x1 - x0
        dz = z1 - z0
        distance = math.hypot(dx, dz)
        if distance < 1e-9:
            return []
        if distance > radius0 + radius1 + 1e-9:
            return []
        if distance < abs(radius0 - radius1) - 1e-9:
            return []

        a = (radius0 * radius0 - radius1 * radius1 + distance * distance) / (2.0 * distance)
        h_sq = radius0 * radius0 - a * a
        if h_sq < -1e-6:
            return []
        h = math.sqrt(max(0.0, h_sq))

        xm = x0 + a * dx / distance
        zm = z0 + a * dz / distance
        rx = -dz * h / distance
        rz = dx * h / distance

        if h < 1e-9:
            return [(xm, zm)]
        return [(xm + rx, zm + rz), (xm - rx, zm - rz)]

    def _local_chain_from_servo2_servo3(
        self, servo2_deg: float, servo3_deg: float
    ) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float], tuple[float, float], float, float]:
        theta1_deg = self._theta1_from_servo2(servo2_deg)
        phi4_deg = self._phi4_from_servo3(servo3_deg)

        theta1_rad = math.radians(theta1_deg)
        alpha4_deg = self._alpha4_from_phi4(phi4_deg)
        alpha4_rad = math.radians(alpha4_deg)

        s4 = (self.L3 * math.sin(theta1_rad), self.L3 * math.cos(theta1_rad))
        s5 = (
            s4[0] + self.L4 * math.cos(alpha4_rad),
            s4[1] + self.L4 * math.sin(alpha4_rad),
        )
        seat = (s5[0] + self.L5, s5[1])
        end = (seat[0], seat[1] - self.L6_nominal)
        return s4, s5, seat, end, theta1_deg, phi4_deg

    def _local_planar_to_global_base(self, end_local_mm: tuple[float, float]) -> tuple[float, float]:
        radial_mm = self.L2 + end_local_mm[0]
        z_mm = self.L1 + end_local_mm[1]
        return radial_mm, z_mm

    def forward_state(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> ForwardState:
        yaw_deg = self._yaw_from_servo1(servo1_deg)
        _, _, _, end_local, theta1_deg, phi4_deg = self._local_chain_from_servo2_servo3(servo2_deg, servo3_deg)
        radial_mm, z_mm = self._local_planar_to_global_base(end_local)
        yaw_rad = math.radians(yaw_deg)
        x_mm = radial_mm * math.cos(yaw_rad)
        y_mm = radial_mm * math.sin(yaw_rad)

        return ForwardState(
            servo1_deg=servo1_deg,
            servo2_deg=servo2_deg,
            servo3_deg=servo3_deg,
            yaw_deg=yaw_deg,
            theta1_deg=theta1_deg,
            phi4_deg=phi4_deg,
            radial_mm=radial_mm,
            z_mm=z_mm,
            x_mm=x_mm,
            y_mm=y_mm,
            branch_name="analytic_main_chain",
            left_branch="n/a",
            right_branch="n/a",
            branch_residual_mm=0.0,
            left_candidates=1,
            right_candidates=1,
        )

    def forward_kinematics(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> tuple[float, float, float]:
        state = self.forward_state(servo1_deg, servo2_deg, servo3_deg)
        return round(state.x_mm, 1), round(state.y_mm, 1), round(state.z_mm, 1)

    def _candidate_home_penalty(self, candidate: InverseCandidate) -> float:
        return (
            abs(candidate.servo1_deg - self.servo1_home)
            + abs(candidate.servo2_deg - self.servo2_home)
            + abs(candidate.servo3_deg - self.servo3_home)
        )

    def _servo2_from_theta1(self, theta1_deg: float) -> float:
        return self.servo2_home - (theta1_deg - self.servo2_theta1_bias_deg) / self.servo2_theta1_scale

    def _servo3_from_phi4(self, phi4_deg: float) -> float:
        return self.servo3_home + (phi4_deg - self.servo3_phi4_bias_deg) / self.servo3_phi4_scale

    def _build_candidate(
        self,
        branch: str,
        servo1_deg: float,
        servo2_deg: float,
        servo3_deg: float,
        x_mm: float,
        y_mm: float,
        z_mm: float,
    ) -> InverseCandidate | None:
        if not self._is_servo_valid(servo1_deg, servo2_deg, servo3_deg):
            return None

        state = self.forward_state(servo1_deg, servo2_deg, servo3_deg)
        error_mm = math.sqrt(
            (state.x_mm - x_mm) ** 2
            + (state.y_mm - y_mm) ** 2
            + (state.z_mm - z_mm) ** 2
        )
        return InverseCandidate(
            branch=branch,
            servo1_deg=servo1_deg,
            servo2_deg=servo2_deg,
            servo3_deg=servo3_deg,
            theta1_deg=state.theta1_deg,
            phi4_deg=state.phi4_deg,
            fk_x_mm=state.x_mm,
            fk_y_mm=state.y_mm,
            fk_z_mm=state.z_mm,
            error_mm=error_mm,
            branch_residual_mm=0.0,
        )

    def _analytic_candidates(self, x_mm: float, y_mm: float, z_mm: float) -> list[InverseCandidate]:
        servo1_deg = self._servo1_from_xy(x_mm, y_mm)
        if not (self.servo1_cmd_min <= servo1_deg <= self.servo1_cmd_max):
            return []

        radial_mm = math.hypot(x_mm, y_mm)
        end_local = (radial_mm - self.L2, z_mm - self.L1)
        seat = (end_local[0], end_local[1] + self.L6_nominal)
        s5 = (seat[0] - self.L5, seat[1])

        candidates: list[InverseCandidate] = []
        for index, s4 in enumerate(self._circle_intersections((0.0, 0.0), self.L3, s5, self.L4)):
            theta1_deg = math.degrees(math.atan2(s4[0], s4[1]))
            alpha4_deg = math.degrees(math.atan2(s5[1] - s4[1], s5[0] - s4[0]))
            phi4_deg = -alpha4_deg
            servo2_deg = self._servo2_from_theta1(theta1_deg)
            servo3_deg = self._servo3_from_phi4(phi4_deg)
            candidate = self._build_candidate(
                branch=f"analytic:{index}",
                servo1_deg=servo1_deg,
                servo2_deg=servo2_deg,
                servo3_deg=servo3_deg,
                x_mm=x_mm,
                y_mm=y_mm,
                z_mm=z_mm,
            )
            if candidate is not None:
                candidates.append(candidate)

        candidates.sort(key=lambda item: (self._candidate_home_penalty(item), item.error_mm, item.branch))
        return candidates

    def inverse_kinematics_debug(self, x_mm: float, y_mm: float, z_mm: float) -> list[InverseCandidate]:
        return self._analytic_candidates(x_mm, y_mm, z_mm)

    def inverse_kinematics(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float] | None:
        candidates = self._analytic_candidates(x_mm, y_mm, z_mm)
        if not candidates:
            return None
        best = candidates[0]
        return best.servo1_deg, best.servo2_deg, best.servo3_deg
