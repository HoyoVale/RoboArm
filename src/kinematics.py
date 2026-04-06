"""
Closed-form kinematics for the 3-DOF vacuum arm.

Authoritative mechanism description:
docs/机械臂参数/README.md

Model conventions used here:
- theta1 = servo2_home - servo2_deg
  theta1 is the angle of L3 relative to the vertical axis.
- theta2 = servo3_home - servo3_deg
  theta2 is the angle change of B1 relative to its home pose.
- servo1 rotates the whole arm around the vertical axis.
- Left-view A linkage and right-view B linkage are solved as closed-form
  circle-intersection branches, then matched at the shared wrist point S5.
- In the left view, S1 is a fixed base pivot determined by the rigid A1/A2
  bracket. It must not rotate together with servo2/L3.
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

from src.config import IK_COARSE_STEP_DEG, IK_FINE_STEP_DEG, IK_MAX_POSITION_ERROR_MM, KINEMATICS_L6_NOMINAL_MM


@dataclass(frozen=True)
class _LeftCandidate:
    branch: str
    s1: tuple[float, float]
    s2: tuple[float, float]
    s3: tuple[float, float]
    s4: tuple[float, float]
    s5: tuple[float, float]
    s6: tuple[float, float]
    end: tuple[float, float]
    l4_angle_rad: float


@dataclass(frozen=True)
class _RightCandidate:
    branch: str
    t1: tuple[float, float]
    t2: tuple[float, float]
    s5: tuple[float, float]
    l4_angle_rad: float


@dataclass(frozen=True)
class ForwardState:
    servo1_deg: float
    servo2_deg: float
    servo3_deg: float
    yaw_deg: float
    theta1_deg: float
    theta2_deg: float
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
    theta2_deg: float
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

        self.A1 = 36.0
        self.A2 = 24.0
        self.A3 = 149.0
        self.A4 = 42.0
        self.A5 = 69.0
        self.A6 = 42.0
        self.A7 = 160.0
        self.A8 = 31.0

        self.B1 = 54.0
        self.B2 = 148.0
        self.B3 = 54.0

        self.servo1_home = 60.0
        self.servo2_home = 65.0
        self.servo3_home = 55.0

        self.servo1_cmd_min = 20.0
        self.servo1_cmd_max = 100.0
        self.servo2_min = 20.0
        self.servo2_max = 80.0
        self.servo3_min = 50.0
        self.servo3_max = 90.0

        self.ik_coarse_step_deg = float(IK_COARSE_STEP_DEG)
        self.ik_fine_step_deg = float(IK_FINE_STEP_DEG)
        self.ik_max_position_error_mm = float(IK_MAX_POSITION_ERROR_MM)

        self._home_s1_vector = self._solve_home_s1_vector()

    def _is_servo_valid(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> bool:
        return (
            self.servo1_cmd_min <= servo1_deg <= self.servo1_cmd_max
            and self.servo2_min <= servo2_deg <= self.servo2_max
            and self.servo3_min <= servo3_deg <= self.servo3_max
        )

    def _theta1_from_servo2(self, servo2_deg: float) -> float:
        return self.servo2_home - servo2_deg

    def _theta2_from_servo3(self, servo3_deg: float) -> float:
        return self.servo3_home - servo3_deg

    def _servo2_from_theta1(self, theta1_deg: float) -> float:
        return self.servo2_home - theta1_deg

    def _servo3_from_theta2(self, theta2_deg: float) -> float:
        return self.servo3_home - theta2_deg

    def _servo1_from_xy(self, x_mm: float, y_mm: float) -> float:
        yaw_deg = math.degrees(math.atan2(y_mm, x_mm))
        return self.servo1_home + yaw_deg

    @staticmethod
    def _distance(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _rotate(vector: tuple[float, float], angle_rad: float) -> tuple[float, float]:
        x_mm, z_mm = vector
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        return (x_mm * cos_a - z_mm * sin_a, x_mm * sin_a + z_mm * cos_a)

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

    def _solve_home_s1_vector(self) -> tuple[float, float]:
        s_origin = (0.0, 0.0)
        s4_home = (0.0, self.L3)
        end_home = (self.L4 + self.L5, self.L3 - self.L6_nominal)
        s6_home = (end_home[0], end_home[1] + self.A8)

        best_score: float | None = None
        best_s1: tuple[float, float] | None = None
        rigid_radius = math.hypot(self.A1, self.A2)

        for s3 in self._circle_intersections(s4_home, self.A6, s6_home, self.A7):
            for s2 in self._circle_intersections(s4_home, self.A4, s3, self.A5):
                for s1 in self._circle_intersections(s2, self.A3, s_origin, rigid_radius):
                    # Match the documented home sketch: S1 is a fixed base pivot
                    # located behind and above S in the left-view plane.
                    score = (
                        (0.0 if s1[0] < 0.0 and s1[1] > 0.0 else 1000.0)
                        + abs(s1[0] + 34.526)
                        + abs(s1[1] - 26.075)
                    )
                    if best_score is None or score < best_score:
                        best_score = score
                        best_s1 = s1

        if best_s1 is None:
            raise RuntimeError("无法根据文档参数求出 A1/A2 摇臂的 home 向量。")
        return best_s1

    def _left_candidates(self, theta1_deg: float) -> list[_LeftCandidate]:
        theta1_rad = math.radians(theta1_deg)
        s4 = (self.L3 * math.sin(theta1_rad), self.L3 * math.cos(theta1_rad))
        # S1 belongs to the rigid base-side A1/A2 bracket. It is fixed in the
        # left-view frame and does not rotate with L3.
        s1 = self._home_s1_vector

        candidates: list[_LeftCandidate] = []
        for s2_index, s2 in enumerate(self._circle_intersections(s1, self.A3, s4, self.A4)):
            for s3_index, s3 in enumerate(self._circle_intersections(s2, self.A5, s4, self.A6)):
                end_center_l4 = (s4[0] + self.L5, s4[1] - self.L6_nominal)
                end_center_a7 = (s3[0], s3[1] - self.A8)
                for end_index, end in enumerate(self._circle_intersections(end_center_l4, self.L4, end_center_a7, self.A7)):
                    s5 = (end[0] - self.L5, end[1] + self.L6_nominal)
                    s6 = (end[0], end[1] + self.A8)
                    l4_angle_rad = math.atan2(s5[1] - s4[1], s5[0] - s4[0])
                    candidates.append(
                        _LeftCandidate(
                            branch=f"L{s2_index}-{s3_index}-{end_index}",
                            s1=s1,
                            s2=s2,
                            s3=s3,
                            s4=s4,
                            s5=s5,
                            s6=s6,
                            end=end,
                            l4_angle_rad=l4_angle_rad,
                        )
                    )
        return candidates

    def _right_candidates(self, theta1_deg: float, theta2_deg: float) -> list[_RightCandidate]:
        theta1_rad = math.radians(theta1_deg)
        theta2_rad = math.radians(theta2_deg)
        s4 = (self.L3 * math.sin(theta1_rad), self.L3 * math.cos(theta1_rad))
        # B1 home is taken to be parallel with L4 and pointing toward the arm front.
        t1 = self._rotate((self.B1, 0.0), -theta2_rad)

        candidates: list[_RightCandidate] = []
        for t2_index, t2 in enumerate(self._circle_intersections(t1, self.B2, s4, self.B3)):
            vx = s4[0] - t2[0]
            vz = s4[1] - t2[1]
            norm = math.hypot(vx, vz)
            if norm < 1e-9:
                continue
            # L4 and B3 are collinear at T3(S4), with L4 extending opposite
            # to the direction from T3 to T2.
            s5 = (s4[0] - self.L4 * vx / norm, s4[1] - self.L4 * vz / norm)
            if s5[0] < s4[0] - 1e-6:
                continue
            l4_angle_rad = math.atan2(s5[1] - s4[1], s5[0] - s4[0])
            candidates.append(
                _RightCandidate(
                    branch=f"R{t2_index}",
                    t1=t1,
                    t2=t2,
                    s5=s5,
                    l4_angle_rad=l4_angle_rad,
                )
            )
        return candidates

    def _select_forward_branch(
        self,
        theta1_deg: float,
        theta2_deg: float,
    ) -> tuple[_LeftCandidate, _RightCandidate, float]:
        left_candidates = self._left_candidates(theta1_deg)
        right_candidates = self._right_candidates(theta1_deg, theta2_deg)
        if not left_candidates or not right_candidates:
            raise ValueError("当前舵机角下闭链无有效分支。")

        best_combo: tuple[_LeftCandidate, _RightCandidate, float] | None = None
        best_score: float | None = None

        for left in left_candidates:
            for right in right_candidates:
                angle_delta = (left.l4_angle_rad - right.l4_angle_rad + math.pi) % (2.0 * math.pi) - math.pi
                residual_mm = abs(angle_delta) * self.L4
                s5_distance_mm = self._distance(left.s5, right.s5)
                # Prefer front-working branches and low residuals.
                score = residual_mm + 0.05 * s5_distance_mm
                if left.end[0] < -self.L2:
                    score += 500.0
                if left.end[1] < -self.L6_nominal - 10.0:
                    score += 50.0
                if best_score is None or score < best_score:
                    best_score = score
                    best_combo = (left, right, residual_mm)

        if best_combo is None:
            raise ValueError("无法选择闭链分支。")
        return best_combo

    def forward_state(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> ForwardState:
        yaw_deg = servo1_deg - self.servo1_home
        theta1_deg = self._theta1_from_servo2(servo2_deg)
        theta2_deg = self._theta2_from_servo3(servo3_deg)

        left, right, residual_mm = self._select_forward_branch(theta1_deg, theta2_deg)

        radial_mm = self.L2 + left.end[0]
        z_mm = self.L1 + left.end[1]
        yaw_rad = math.radians(yaw_deg)
        x_mm = radial_mm * math.cos(yaw_rad)
        y_mm = radial_mm * math.sin(yaw_rad)

        return ForwardState(
            servo1_deg=servo1_deg,
            servo2_deg=servo2_deg,
            servo3_deg=servo3_deg,
            yaw_deg=yaw_deg,
            theta1_deg=theta1_deg,
            theta2_deg=theta2_deg,
            radial_mm=radial_mm,
            z_mm=z_mm,
            x_mm=x_mm,
            y_mm=y_mm,
            branch_name=f"{left.branch}/{right.branch}",
            left_branch=left.branch,
            right_branch=right.branch,
            branch_residual_mm=residual_mm,
            left_candidates=len(self._left_candidates(theta1_deg)),
            right_candidates=len(self._right_candidates(theta1_deg, theta2_deg)),
        )

    def forward_kinematics(self, servo1_deg: float, servo2_deg: float, servo3_deg: float) -> tuple[float, float, float]:
        state = self.forward_state(servo1_deg, servo2_deg, servo3_deg)
        return round(state.x_mm, 1), round(state.y_mm, 1), round(state.z_mm, 1)

    def _inverse_candidates(self, x_mm: float, y_mm: float, z_mm: float) -> list[InverseCandidate]:
        radial_target_mm = math.hypot(x_mm, y_mm)
        if radial_target_mm <= 1e-9:
            return []

        servo1_center_deg = self._servo1_from_xy(x_mm, y_mm)
        servo1_candidates: list[float] = []
        for servo1_deg in (servo1_center_deg - 1.0, servo1_center_deg, servo1_center_deg + 1.0):
            if self.servo1_cmd_min <= servo1_deg <= self.servo1_cmd_max:
                servo1_candidates.append(servo1_deg)
        if not servo1_candidates:
            return []

        coarse_hits: list[InverseCandidate] = []
        servo2_value = self.servo2_min
        while servo2_value <= self.servo2_max + 1e-9:
            servo3_value = self.servo3_min
            while servo3_value <= self.servo3_max + 1e-9:
                for servo1_deg in servo1_candidates:
                    state = self.forward_state(servo1_deg, servo2_value, servo3_value)
                    error_mm = math.sqrt(
                        (state.x_mm - x_mm) ** 2
                        + (state.y_mm - y_mm) ** 2
                        + (state.z_mm - z_mm) ** 2
                    )
                    coarse_hits.append(
                        InverseCandidate(
                            branch=f"coarse:{state.branch_name}",
                            servo1_deg=servo1_deg,
                            servo2_deg=servo2_value,
                            servo3_deg=servo3_value,
                            theta1_deg=state.theta1_deg,
                            theta2_deg=state.theta2_deg,
                            fk_x_mm=state.x_mm,
                            fk_y_mm=state.y_mm,
                            fk_z_mm=state.z_mm,
                            error_mm=error_mm,
                            branch_residual_mm=state.branch_residual_mm,
                        )
                    )
                servo3_value += self.ik_coarse_step_deg
            servo2_value += self.ik_coarse_step_deg

        if not coarse_hits:
            return []

        coarse_best = min(
            coarse_hits,
            key=lambda item: (
                item.error_mm,
                item.branch_residual_mm,
                abs(item.servo1_deg - self.servo1_home)
                + abs(item.servo2_deg - self.servo2_home)
                + abs(item.servo3_deg - self.servo3_home),
            ),
        )

        fine_hits: list[InverseCandidate] = []
        servo2_start = max(self.servo2_min, coarse_best.servo2_deg - self.ik_coarse_step_deg)
        servo2_stop = min(self.servo2_max, coarse_best.servo2_deg + self.ik_coarse_step_deg)
        servo3_start = max(self.servo3_min, coarse_best.servo3_deg - self.ik_coarse_step_deg)
        servo3_stop = min(self.servo3_max, coarse_best.servo3_deg + self.ik_coarse_step_deg)

        servo2_value = servo2_start
        while servo2_value <= servo2_stop + 1e-9:
            servo3_value = servo3_start
            while servo3_value <= servo3_stop + 1e-9:
                for servo1_deg in servo1_candidates:
                    state = self.forward_state(servo1_deg, servo2_value, servo3_value)
                    error_mm = math.sqrt(
                        (state.x_mm - x_mm) ** 2
                        + (state.y_mm - y_mm) ** 2
                        + (state.z_mm - z_mm) ** 2
                    )
                    fine_hits.append(
                        InverseCandidate(
                            branch=f"fine:{state.branch_name}",
                            servo1_deg=servo1_deg,
                            servo2_deg=servo2_value,
                            servo3_deg=servo3_value,
                            theta1_deg=state.theta1_deg,
                            theta2_deg=state.theta2_deg,
                            fk_x_mm=state.x_mm,
                            fk_y_mm=state.y_mm,
                            fk_z_mm=state.z_mm,
                            error_mm=error_mm,
                            branch_residual_mm=state.branch_residual_mm,
                        )
                    )
                servo3_value += self.ik_fine_step_deg
            servo2_value += self.ik_fine_step_deg

        return fine_hits or coarse_hits

    def inverse_kinematics_debug(self, x_mm: float, y_mm: float, z_mm: float) -> list[InverseCandidate]:
        return self._inverse_candidates(x_mm, y_mm, z_mm)

    def inverse_kinematics(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float] | None:
        candidates = self._inverse_candidates(x_mm, y_mm, z_mm)
        if not candidates:
            return None

        best = min(
            candidates,
            key=lambda item: (
                item.error_mm,
                item.branch_residual_mm,
                abs(item.servo1_deg - self.servo1_home)
                + abs(item.servo2_deg - self.servo2_home)
                + abs(item.servo3_deg - self.servo3_home),
            ),
        )
        if best.error_mm > self.ik_max_position_error_mm:
            return None

        return (
            round(best.servo1_deg, 1),
            round(best.servo2_deg, 1),
            round(best.servo3_deg, 1),
        )


if __name__ == "__main__":
    arm = PalletizingArmKinematics()
    home_angles = (60.0, 65.0, 55.0)
    print("Home FK:", arm.forward_kinematics(*home_angles))
