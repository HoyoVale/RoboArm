from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2] # 获取项目根目录
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# 导入参数配置和工具函数
from src.config import CAMERA_HEIGHT, CAMERA_INDEX, CAMERA_WIDTH, ensure_runtime_dirs


WINDOW_NAME = "Four Point Calibration"
POINT_ORDER_TEXT = "左上 -> 右上 -> 右下 -> 左下"
image_points: list[tuple[int, int]] = [] # 一个图像点的结构体


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Four-point table calibration tool.")
    parser.add_argument("--index", type=int, default=CAMERA_INDEX, help="Camera index.")
    parser.add_argument("--width", type=int, default=CAMERA_WIDTH, help="Capture width.")
    parser.add_argument("--height", type=int, default=CAMERA_HEIGHT, help="Capture height.")
    return parser


def mouse_callback(event, x, y, flags, param) -> None:
    del flags, param
    global image_points
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(image_points) < 4:
            image_points.append((x, y))
            print(f"[*] 记录图像点 {len(image_points)}: (u={x}, v={y})")
        else:
            print("[!] 已经选满 4 个点，按 r 清空后重新选择。")


def prompt_robot_points() -> list[tuple[float, float]]:
    robot_points: list[tuple[float, float]] = []
    print("\n请输入与点击顺序一致的机械臂桌面坐标，单位 mm。")
    print(f"点顺序固定为：{POINT_ORDER_TEXT}")
    for index in range(4):
        while True:
            raw = input(f"第 {index + 1} 个点坐标，格式 x,y: ").strip()
            try:
                x_str, y_str = raw.split(",")
                robot_points.append((float(x_str), float(y_str)))
                break
            except ValueError:
                print("[-] 输入格式错误，请重新输入，例如 200,100")
    return robot_points


def compute_reprojection_errors(
    image_points_array: np.ndarray,
    robot_points_array: np.ndarray,
    homography_matrix: np.ndarray,
) -> tuple[list[float], float, float]:
    projected = cv2.perspectiveTransform(image_points_array.reshape(-1, 1, 2), homography_matrix).reshape(-1, 2)
    errors = np.linalg.norm(projected - robot_points_array, axis=1)
    error_list = [float(value) for value in errors]
    return error_list, float(np.mean(errors)), float(np.max(errors))


def save_outputs(
    camera_index: int,
    width: int,
    height: int,
    homography_matrix: np.ndarray,
    image_points_array: np.ndarray,
    robot_points_array: np.ndarray,
    error_list: list[float],
    mean_error: float,
    max_error: float,
) -> None:
    ensure_runtime_dirs()
    matrix_path = PROJECT_ROOT / "matrixs" / "homography_matrix.npy"
    meta_path = PROJECT_ROOT / "matrixs" / "homography_meta.json"
    np.save(matrix_path, homography_matrix)

    payload = {
        "camera_index": camera_index,
        "image_width": width,
        "image_height": height,
        "point_order": POINT_ORDER_TEXT,
        "image_points": image_points_array.tolist(),
        "robot_points": robot_points_array.tolist(),
        "reprojection_errors_mm": error_list,
        "mean_reprojection_error_mm": mean_error,
        "max_reprojection_error_mm": max_error,
    }
    meta_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    print("\n标定结果已保存")
    print(f"矩阵文件: {matrix_path}")
    print(f"元数据文件: {meta_path}")
    print(f"平均回投误差: {mean_error:.3f} mm")
    print(f"最大回投误差: {max_error:.3f} mm")


def main() -> int:
    global image_points
    args = build_parser().parse_args()
    image_points = []

    # 获取指定相机
    cap = cv2.VideoCapture(args.index)
    if not cap.isOpened():
        print("[-] 无法打开摄像头。")
        return 1

    # 指定分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or args.width
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or args.height

    # 创建窗口并设置鼠标回调
    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, mouse_callback)

    # 调试信息输出
    print("\n" + "=" * 50)
    print("四点标定工具")
    print("=" * 50)
    print(f"相机索引: {args.index}")
    print(f"分辨率: {actual_width}x{actual_height}")
    print(f"点击顺序: {POINT_ORDER_TEXT}")
    print("按 r 清空，按 q 或 Esc 在点满 4 个后进入坐标输入。")
    print("=" * 50 + "\n")

    while True:
        # 获取帧
        ok, frame = cap.read()
        if not ok:
            print("[-] 无法读取摄像头画面。")
            break

        display = frame.copy()
        for idx, point in enumerate(image_points):
            cv2.circle(display, point, 5, (0, 0, 255), -1)
            cv2.putText(
                display,
                str(idx + 1),
                (point[0] + 10, point[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            if idx > 0:
                cv2.line(display, image_points[idx - 1], point, (255, 0, 0), 2)
            if len(image_points) == 4 and idx == 3:
                cv2.line(display, image_points[3], image_points[0], (255, 0, 0), 2)

        cv2.putText(
            display,
            f"Points: {len(image_points)}/4",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 255, 255),
            2,
        )
        cv2.imshow(WINDOW_NAME, display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            image_points = []
            print("[*] 已清空选择的点。")
        elif key == ord("q") or key == 27:
            if len(image_points) == 4:
                break
            print(f"[!] 当前只选了 {len(image_points)} 个点，必须选满 4 个。")
    # 释放资源
    cap.release()
    cv2.destroyAllWindows()

    if len(image_points) != 4:
        return 2

    robot_points = prompt_robot_points()
    image_points_array = np.array(image_points, dtype=np.float32)
    robot_points_array = np.array(robot_points, dtype=np.float32)

    homography_matrix, status = cv2.findHomography(image_points_array, robot_points_array)
    if homography_matrix is None or status is None:
        print("[-] 透视变换矩阵求解失败。")
        return 3

    error_list, mean_error, max_error = compute_reprojection_errors(
        image_points_array,
        robot_points_array,
        homography_matrix,
    )
    save_outputs(
        camera_index=args.index,
        width=actual_width,
        height=actual_height,
        homography_matrix=homography_matrix,
        image_points_array=image_points_array,
        robot_points_array=robot_points_array,
        error_list=error_list,
        mean_error=mean_error,
        max_error=max_error,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
