from __future__ import annotations

import time
from dataclasses import dataclass

from src.config import (
    APPROACH_CLEARANCE_MM,
    APPROACH_MOVE_MS,
    DEFAULT_GRAB_PUMP_MS,
    DESCEND_MOVE_MS,
    GRAB_SETTLE_SEC,
    HOME_MOVE_MS,
    LIFT_CLEARANCE_MM,
    LIFT_MOVE_MS,
    MOVE_SETTLE_SEC,
    PLACE_MOVE_MS,
    PLACE_X_MM,
    PLACE_Y_MM,
    PLACE_Z_MM,
    RELEASE_SETTLE_SEC,
)
from src.kinematics import PalletizingArmKinematics
from tools.control.robot_arm_controller import RobotArmController


class GraspExecutionError(RuntimeError):
    pass


@dataclass(frozen=True)
class GraspTarget:
    x_mm: float
    y_mm: float
    z_mm: float
    label: str


class GraspExecutor:
    def __init__(
        self,
        arm_ctrl: RobotArmController,
        arm_ik: PalletizingArmKinematics,
        place_x_mm: float = PLACE_X_MM,
        place_y_mm: float = PLACE_Y_MM,
        place_z_mm: float = PLACE_Z_MM,
        approach_clearance_mm: float = APPROACH_CLEARANCE_MM,
        lift_clearance_mm: float = LIFT_CLEARANCE_MM,
        grab_pump_ms: int = DEFAULT_GRAB_PUMP_MS,
    ):
        self.arm_ctrl = arm_ctrl
        self.arm_ik = arm_ik
        self.place_x_mm = place_x_mm
        self.place_y_mm = place_y_mm
        self.place_z_mm = place_z_mm
        self.approach_clearance_mm = approach_clearance_mm
        self.lift_clearance_mm = lift_clearance_mm
        self.grab_pump_ms = grab_pump_ms

    def _sleep_after_move(self, move_time_ms: int) -> None:
        time.sleep(move_time_ms / 1000.0 + MOVE_SETTLE_SEC)

    def _resolve_angles(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[int, int, int]:
        angles = self.arm_ik.inverse_kinematics(x_mm, y_mm, z_mm)
        if angles is None:
            raise GraspExecutionError(
                f"目标点不可达: x={x_mm:.1f}, y={y_mm:.1f}, z={z_mm:.1f}"
            )
        return tuple(int(round(angle)) for angle in angles)

    def _move_xyz(self, x_mm: float, y_mm: float, z_mm: float, move_time_ms: int) -> None:
        servo1, servo2, servo3 = self._resolve_angles(x_mm, y_mm, z_mm)
        self.arm_ctrl.move_angles(
            {1: servo1, 2: servo2, 3: servo3},
            move_time_ms=move_time_ms,
        )
        self._sleep_after_move(move_time_ms)

    def can_execute_target(self, x_mm: float, y_mm: float, z_mm: float) -> bool:
        pick_above_z = z_mm + self.approach_clearance_mm
        place_above_z = self.place_z_mm + self.approach_clearance_mm
        transit_z = max(z_mm + self.lift_clearance_mm, pick_above_z, place_above_z)
        checkpoints = (
            (x_mm, y_mm, pick_above_z),
            (x_mm, y_mm, z_mm),
            (x_mm, y_mm, transit_z),
            (self.place_x_mm, self.place_y_mm, transit_z),
            (self.place_x_mm, self.place_y_mm, self.place_z_mm),
        )
        return all(self.arm_ik.inverse_kinematics(*checkpoint) is not None for checkpoint in checkpoints)

    def execute_pick_and_place(self, target: GraspTarget) -> None:
        pick_above_z = target.z_mm + self.approach_clearance_mm
        place_above_z = self.place_z_mm + self.approach_clearance_mm
        transit_z = max(target.z_mm + self.lift_clearance_mm, pick_above_z, place_above_z)

        self.arm_ctrl.home(HOME_MOVE_MS)
        self._sleep_after_move(HOME_MOVE_MS)

        self._move_xyz(target.x_mm, target.y_mm, pick_above_z, APPROACH_MOVE_MS)
        self._move_xyz(target.x_mm, target.y_mm, target.z_mm, DESCEND_MOVE_MS)

        self.arm_ctrl.grab(self.grab_pump_ms)
        time.sleep(GRAB_SETTLE_SEC)

        self._move_xyz(target.x_mm, target.y_mm, transit_z, LIFT_MOVE_MS)
        self._move_xyz(self.place_x_mm, self.place_y_mm, transit_z, PLACE_MOVE_MS)
        self._move_xyz(self.place_x_mm, self.place_y_mm, self.place_z_mm, DESCEND_MOVE_MS)

        self.arm_ctrl.release()
        time.sleep(RELEASE_SETTLE_SEC)

        self._move_xyz(self.place_x_mm, self.place_y_mm, place_above_z, LIFT_MOVE_MS)
        self.arm_ctrl.home(HOME_MOVE_MS)
        self._sleep_after_move(HOME_MOVE_MS)
