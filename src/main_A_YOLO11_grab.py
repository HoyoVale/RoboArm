from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


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
    DEFAULT_TRIGGER_MODE,
    PICK_Z_MM,
    SERIAL_BAUD,
    SERIAL_PORT,
    YOLO_ALLOWED_CLASSES,
    YOLO_BBOX_FOOT_RATIO,
    YOLO_CONFIDENCE,
    YOLO_MODEL_PATH,
)
from src.grasp_executor import GraspExecutor, GraspExecutionError, GraspTarget
from src.kinematics import PalletizingArmKinematics
from src.target_tracker import StableTargetTracker, TargetObservation
from src.workspace import WorkspaceCalibrationError, WorkspaceTransform
from tools.control.robot_arm_controller import RobotArmController


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="YOLO fruit grasping app.")
    parser.add_argument("--mode", choices=("manual", "auto"), default=DEFAULT_TRIGGER_MODE)
    parser.add_argument("--port", default=SERIAL_PORT)
    parser.add_argument("--camera-index", type=int, default=CAMERA_INDEX)
    parser.add_argument("--width", type=int, default=CAMERA_WIDTH)
    parser.add_argument("--height", type=int, default=CAMERA_HEIGHT)
    parser.add_argument("--pick-z", type=float, default=PICK_Z_MM)
    parser.add_argument("--model", default=str(YOLO_MODEL_PATH))
    parser.add_argument("--conf", type=float, default=YOLO_CONFIDENCE)
    parser.add_argument("--cooldown-sec", type=float, default=AUTO_COOLDOWN_SEC)
    parser.add_argument("--stable-frames", type=int, default=AUTO_STABLE_FRAMES)
    parser.add_argument("--stable-jitter-mm", type=float, default=AUTO_STABLE_POSITION_JITTER_MM)
    return parser


def bbox_contact_point(x1: float, y1: float, x2: float, y2: float) -> tuple[int, int]:
    height = max(1.0, y2 - y1)
    contact_u = int(round((x1 + x2) * 0.5))
    contact_v = int(round(y2 - height * YOLO_BBOX_FOOT_RATIO))
    return contact_u, contact_v


def select_best_target(frame, model, workspace, executor, pick_z_mm: float, confidence_threshold: float):
    best_target = None
    best_box = None
    best_confidence = -1.0

    results = model(frame, verbose=False, conf=confidence_threshold)
    for result in results:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            class_name = model.names[cls_id]
            if class_name not in YOLO_ALLOWED_CLASSES:
                continue

            confidence = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            contact_u, contact_v = bbox_contact_point(float(x1), float(y1), float(x2), float(y2))
            target_x_mm, target_y_mm = workspace.pixel_to_table(contact_u, contact_v)
            if not executor.can_execute_target(target_x_mm, target_y_mm, pick_z_mm):
                continue

            if confidence > best_confidence:
                best_confidence = confidence
                best_box = (int(x1), int(y1), int(x2), int(y2))
                best_target = TargetObservation(
                    label=class_name,
                    score=confidence,
                    pixel_u=contact_u,
                    pixel_v=contact_v,
                    x_mm=target_x_mm,
                    y_mm=target_y_mm,
                )
    return best_target, best_box


def draw_overlay(frame, target, box, mode: str, stable_frames: int, cooldown_remaining: float, busy: bool) -> None:
    height, width = frame.shape[:2]
    if box and target:
        x1, y1, x2, y2 = box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (target.pixel_u, target.pixel_v), 5, (0, 0, 255), -1)
        cv2.putText(
            frame,
            f"{target.label} {target.score:.2f}",
            (x1, max(25, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            frame,
            f"X={target.x_mm:.1f} Y={target.y_mm:.1f}",
            (x1, min(height - 10, y2 + 25)),
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

    print("[*] 正在加载 YOLO 模型...")
    model = YOLO(args.model)
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

    print("[*] YOLO 视觉抓取系统启动。")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[-] 摄像头读取失败。")
            break

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
        current_box = None
        stable_frames = 0
        stable_target = None

        if not busy:
            current_target, current_box = select_best_target(
                frame,
                model,
                workspace,
                executor,
                args.pick_z,
                args.conf,
            )
            stable, stable_frames, stable_target = tracker.update(current_target)
        else:
            stable = False
            tracker.reset()

        draw_overlay(frame, current_target, current_box, args.mode, stable_frames, cooldown_remaining, busy)
        cv2.imshow("YOLO Fruit Grasp", frame)
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
