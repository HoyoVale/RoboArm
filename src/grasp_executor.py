"""
抓取执行器
负责将视觉控制系统提供的抓取目标转化为指令
"""

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
from tools.control.robot_arm_controller import RobotArmController, angles_to_pulses


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

    def _log(self, stage: str, message: str) -> None:
        print(f"[GraspExecutor][{stage}] {message}")

    def _sleep_after_move(self, move_time_ms: int) -> None:
        time.sleep(move_time_ms / 1000.0 + MOVE_SETTLE_SEC)

    def _start_vacuum_hold(self) -> None:
        self._log("GRAB", "valve close")
        self.arm_ctrl.valve_close()
        time.sleep(0.10)
        self._log("GRAB", "pump on")
        self.arm_ctrl.pump_on()
        time.sleep(self.grab_pump_ms / 1000.0 + GRAB_SETTLE_SEC)
        self._log("GRAB", "vacuum hold ready")

    def _release_vacuum_hold(self) -> None:
        release_lead_sec = min(0.20, RELEASE_SETTLE_SEC)
        self._log("RELEASE", "valve open")
        self.arm_ctrl.valve_open()
        time.sleep(release_lead_sec)
        self._log("RELEASE", "pump off")
        self.arm_ctrl.pump_off()
        time.sleep(max(0.0, RELEASE_SETTLE_SEC - release_lead_sec))
        self._log("RELEASE", "release settle complete")

    def _resolve_angles(self, x_mm: float, y_mm: float, z_mm: float) -> tuple[float, float, float]:
        angles = self.arm_ik.inverse_kinematics(x_mm, y_mm, z_mm)
        if angles is None:
            raise GraspExecutionError(
                f"目标点不可达: x={x_mm:.1f}, y={y_mm:.1f}, z={z_mm:.1f}"
            )
        self._log(
            "IK",
            "xyz="
            f"({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) -> "
            f"servo=({angles[0]:.2f}, {angles[1]:.2f}, {angles[2]:.2f})",
        )
        return angles

    def _move_xyz(self, stage: str, x_mm: float, y_mm: float, z_mm: float, move_time_ms: int) -> None:
        servo1, servo2, servo3 = self._resolve_angles(x_mm, y_mm, z_mm)
        pulses = dict(angles_to_pulses({1: servo1, 2: servo2, 3: servo3}))
        self._log(
            stage,
            "move_xyz "
            f"target=({x_mm:.1f}, {y_mm:.1f}, {z_mm:.1f}) "
            f"servo=({servo1:.2f}, {servo2:.2f}, {servo3:.2f}) "
            f"pulse=({pulses[1]}, {pulses[2]}, {pulses[3]}) "
            f"time={move_time_ms}ms",
        )
        self.arm_ctrl.move_angles_precise({1: servo1, 2: servo2, 3: servo3}, move_time_ms=move_time_ms)
        self._sleep_after_move(move_time_ms)
        self._log(stage, "move complete")

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

        self._log(
            "START",
            f"label={target.label} "
            f"pick=({target.x_mm:.1f}, {target.y_mm:.1f}, {target.z_mm:.1f}) "
            f"place=({self.place_x_mm:.1f}, {self.place_y_mm:.1f}, {self.place_z_mm:.1f})",
        )

        self._log("HOME_START", f"home time={HOME_MOVE_MS}ms")
        self.arm_ctrl.home(HOME_MOVE_MS)
        self._sleep_after_move(HOME_MOVE_MS)
        self._log("HOME_START", "home complete")

        self._move_xyz("APPROACH_PICK", target.x_mm, target.y_mm, pick_above_z, APPROACH_MOVE_MS)
        self._move_xyz("DESCEND_PICK", target.x_mm, target.y_mm, target.z_mm, DESCEND_MOVE_MS)

        self._log("GRAB", "start vacuum hold sequence")
        self._start_vacuum_hold()

        self._move_xyz("LIFT_AFTER_PICK", target.x_mm, target.y_mm, transit_z, LIFT_MOVE_MS)
        self._move_xyz("MOVE_TO_PLACE", self.place_x_mm, self.place_y_mm, transit_z, PLACE_MOVE_MS)
        self._move_xyz("DESCEND_PLACE", self.place_x_mm, self.place_y_mm, self.place_z_mm, DESCEND_MOVE_MS)

        self._log("RELEASE", "start release sequence")
        self._release_vacuum_hold()

        self._move_xyz("LIFT_AFTER_RELEASE", self.place_x_mm, self.place_y_mm, place_above_z, LIFT_MOVE_MS)
        self._log("HOME_END", f"home time={HOME_MOVE_MS}ms")
        self.arm_ctrl.home(HOME_MOVE_MS)
        self._sleep_after_move(HOME_MOVE_MS)
        self._log("HOME_END", "home complete")
        self._log("DONE", "pick and place sequence complete")
