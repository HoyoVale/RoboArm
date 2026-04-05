from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.config import (
    AUTO_COOLDOWN_SEC,
    AUTO_STABLE_FRAMES,
    AUTO_STABLE_POSITION_JITTER_MM,
    CAMERA_HEIGHT,
    CAMERA_INDEX,
    CAMERA_WIDTH,
    COLOR_CONTACT_BACKOFF_PX,
    COLOR_LABEL,
    COLOR_LOWER_RED_1,
    COLOR_LOWER_RED_2,
    COLOR_MIN_RADIUS_PX,
    COLOR_UPPER_RED_1,
    COLOR_UPPER_RED_2,
    DEFAULT_TRIGGER_MODE,
    PICK_Z_MM,
    SERIAL_BAUD,
    SERIAL_PORT,
)
from src.grasp_executor import GraspExecutor, GraspExecutionError, GraspTarget
from src.kinematics import PalletizingArmKinematics
from src.target_tracker import StableTargetTracker, TargetObservation
from src.workspace import WorkspaceCalibrationError, WorkspaceTransform
from tools.control.robot_arm_controller import RobotArmController


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Color-based red object grasping app.")
    parser.add_argument("--mode", choices=("manual", "auto"), default=DEFAULT_TRIGGER_MODE)
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--camera-index", type=int, default=CAMERA_INDEX)
    parser.add_argument("--width", type=int, default=CAMERA_WIDTH)
    parser.add_argument("--height", type=int, default=CAMERA_HEIGHT)
    parser.add_argument("--pick-z", type=float, default=PICK_Z_MM)
    parser.add_argument("--cooldown-sec", type=float, default=AUTO_COOLDOWN_SEC)
    parser.add_argument("--stable-frames", type=int, default=AUTO_STABLE_FRAMES)
    parser.add_argument("--stable-jitter-mm", type=float, default=AUTO_STABLE_POSITION_JITTER_MM)
    parser.add_argument("--debug-select", action="store_true", help="打印颜色目标筛选调试日志")
    return parser


def contour_contact_point(contour: np.ndarray) -> tuple[int, int]:
    ys = contour[:, 0, 1]
    max_y = int(np.max(ys))
    bottom_band = contour[ys >= max_y - 3][:, 0]
    contact_u = int(round(float(np.mean(bottom_band[:, 0]))))
    contact_v = max(0, max_y - COLOR_CONTACT_BACKOFF_PX)
    return contact_u, contact_v


def select_best_target(frame, workspace, executor, pick_z_mm: float, debug: bool = False):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array(COLOR_LOWER_RED_1)
    upper_red1 = np.array(COLOR_UPPER_RED_1)
    lower_red2 = np.array(COLOR_LOWER_RED_2)
    upper_red2 = np.array(COLOR_UPPER_RED_2)

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)

    if debug:
        print(f"[ColorSelect] contours={len(contours)} pick_z={pick_z_mm:.1f}")

    for index, contour in enumerate(contours, start=1):
        (center, radius) = cv2.minEnclosingCircle(contour)
        if radius < COLOR_MIN_RADIUS_PX:
            if debug:
                print(
                    f"[ColorSelect] contour#{index} skip: "
                    f"radius={radius:.1f} < min_radius={COLOR_MIN_RADIUS_PX:.1f}"
                )
            continue

        contact_u, contact_v = contour_contact_point(contour)
        target_x_mm, target_y_mm = workspace.pixel_to_table(contact_u, contact_v)
        can_execute = executor.can_execute_target(target_x_mm, target_y_mm, pick_z_mm)
        if debug:
            print(
                f"[ColorSelect] contour#{index} "
                f"radius={radius:.1f} pixel=({contact_u}, {contact_v}) "
                f"table=({target_x_mm:.1f}, {target_y_mm:.1f}, {pick_z_mm:.1f}) "
                f"reachable={can_execute}"
            )
        if not can_execute:
            continue

        target = TargetObservation(
            label=COLOR_LABEL,
            score=float(radius),
            pixel_u=contact_u,
            pixel_v=contact_v,
            x_mm=target_x_mm,
            y_mm=target_y_mm,
        )
        if debug:
            print(
                f"[ColorSelect] selected contour#{index}: "
                f"pixel=({contact_u}, {contact_v}) "
                f"table=({target_x_mm:.1f}, {target_y_mm:.1f}, {pick_z_mm:.1f})"
            )
        return target, contour, mask

    if debug and contours:
        print("[ColorSelect] no selectable contour in this sample window")
    return None, None, mask


def draw_overlay(frame, target, contour, mode: str, stable_frames: int, cooldown_remaining: float, busy: bool) -> None:
    height, width = frame.shape[:2]
    if contour is not None and target is not None:
        cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)
        cv2.circle(frame, (target.pixel_u, target.pixel_v), 5, (0, 0, 255), -1)
        x, y, w, h = cv2.boundingRect(contour)
        cv2.putText(
            frame,
            "red target",
            (x, max(25, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"X={target.x_mm:.1f} Y={target.y_mm:.1f}",
            (x, min(height - 10, y + h + 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
        )

    cv2.putText(
        frame,
        f"mode={mode} stable_frames={stable_frames}",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 0),
        2,
    )
    cv2.putText(
        frame,
        "manual: press g to grab | q: quit",
        (10, height - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
    )
    if mode == "auto":
        cv2.putText(
            frame,
            "auto mode enabled",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 255, 0),
            2,
        )
    if cooldown_remaining > 0:
        cv2.putText(
            frame,
            f"cooldown {cooldown_remaining:.1f}s",
            (width - 260, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 165, 255),
            2,
        )
    if busy:
        cv2.putText(
            frame,
            "BUSY",
            (width - 120, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 0, 255),
            3,
        )


def main() -> int:
    args = build_parser().parse_args()

    try:
        workspace = WorkspaceTransform.load()
    except WorkspaceCalibrationError as exc:
        print(f"[-] {exc}")
        return 1

    arm_ik = PalletizingArmKinematics()
    arm_ctrl = RobotArmController(port=args.port, baud=SERIAL_BAUD)
    arm_ctrl.open()
    executor = GraspExecutor(arm_ctrl, arm_ik)
    tracker = StableTargetTracker(args.stable_frames, args.stable_jitter_mm)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        print("[-] 无法打开摄像头。")
        arm_ctrl.close()
        return 2
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.camera_index != workspace.camera_index:
        print(
            "[!] 当前运行相机索引与标定记录不一致："
            f"运行={args.camera_index}，标定={workspace.camera_index}"
        )

    busy = False
    cooldown_until = 0.0
    resolution_checked = False
    frame_index = 0

    print("[*] 颜色抓取系统启动。")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[-] 摄像头读取失败。")
            break
        frame_index += 1

        if not resolution_checked:
            try:
                workspace.ensure_resolution(frame.shape[1], frame.shape[0])
            except WorkspaceCalibrationError as exc:
                print(f"[-] {exc}")
                break
            resolution_checked = True

        now = time.time()
        cooldown_remaining = max(0.0, cooldown_until - now)
        current_target = None
        current_contour = None
        stable_frames = 0
        stable_target = None

        if not busy:
            debug_select_now = args.debug_select and (frame_index % 10 == 0)
            current_target, current_contour, mask = select_best_target(
                frame,
                workspace,
                executor,
                args.pick_z,
                debug=debug_select_now,
            )
            stable, stable_frames, stable_target = tracker.update(current_target)
        else:
            stable = False
            tracker.reset()
            mask = None

        draw_overlay(frame, current_target, current_contour, args.mode, stable_frames, cooldown_remaining, busy)
        cv2.imshow("Color Red Object Grasp", frame)
        if mask is not None:
            cv2.imshow("Color Mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

        trigger_target = None
        if not busy and cooldown_remaining <= 0:
            if args.mode == "manual" and key == ord("g") and current_target is not None:
                trigger_target = current_target
            elif args.mode == "auto" and stable and stable_target is not None:
                trigger_target = stable_target

        if trigger_target is None:
            continue

        busy = True
        tracker.reset()
        print(
            f"[+] 抓取目标 {trigger_target.label}: "
            f"pixel=({trigger_target.pixel_u}, {trigger_target.pixel_v}) "
            f"table=({trigger_target.x_mm:.1f}, {trigger_target.y_mm:.1f}, {args.pick_z:.1f})"
        )
        try:
            executor.execute_pick_and_place(
                GraspTarget(
                    x_mm=trigger_target.x_mm,
                    y_mm=trigger_target.y_mm,
                    z_mm=args.pick_z,
                    label=trigger_target.label,
                )
            )
            print("[+] 抓取流程完成。")
        except GraspExecutionError as exc:
            print(f"[-] 抓取失败: {exc}")
        finally:
            busy = False
            cooldown_until = time.time() + args.cooldown_sec

    cap.release()
    cv2.destroyAllWindows()
    arm_ctrl.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
