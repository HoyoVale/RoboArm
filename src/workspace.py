from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from src.config import (
    HOMOGRAPHY_MATRIX_PATH,
    HOMOGRAPHY_META_PATH,
    WORKSPACE_SWAP_XY,
    WORKSPACE_X_OFFSET_MM,
    WORKSPACE_X_SIGN,
    WORKSPACE_Y_OFFSET_MM,
    WORKSPACE_Y_SIGN,
)


class WorkspaceCalibrationError(RuntimeError):
    pass


@dataclass(frozen=True)
class WorkspaceTransform:
    matrix: np.ndarray
    image_width: int
    image_height: int
    camera_index: int
    point_order: str
    image_points: tuple[tuple[float, float], ...]
    robot_points: tuple[tuple[float, float], ...]
    mean_reprojection_error_mm: float
    max_reprojection_error_mm: float

    @classmethod
    def load(
        cls,
        matrix_path: Path = HOMOGRAPHY_MATRIX_PATH,
        meta_path: Path = HOMOGRAPHY_META_PATH,
    ) -> "WorkspaceTransform":
        if not matrix_path.exists():
            raise WorkspaceCalibrationError(
                f"未找到标定矩阵: {matrix_path}。请先运行四点标定工具。"
            )
        if not meta_path.exists():
            raise WorkspaceCalibrationError(
                f"未找到标定元数据: {meta_path}。请重新运行四点标定工具生成 .json。"
            )

        matrix = np.load(matrix_path)
        if matrix.shape != (3, 3):
            raise WorkspaceCalibrationError("标定矩阵尺寸错误，必须是 3x3。")

        payload = json.loads(meta_path.read_text(encoding="utf-8"))
        required_keys = (
            "camera_index",
            "image_width",
            "image_height",
            "image_points",
            "robot_points",
            "point_order",
            "mean_reprojection_error_mm",
            "max_reprojection_error_mm",
        )
        missing_keys = [key for key in required_keys if key not in payload]
        if missing_keys:
            raise WorkspaceCalibrationError(
                f"标定元数据缺少字段: {', '.join(missing_keys)}"
            )

        return cls(
            matrix=matrix.astype(np.float32),
            image_width=int(payload["image_width"]),
            image_height=int(payload["image_height"]),
            camera_index=int(payload["camera_index"]),
            point_order=str(payload["point_order"]),
            image_points=tuple(tuple(point) for point in payload["image_points"]),
            robot_points=tuple(tuple(point) for point in payload["robot_points"]),
            mean_reprojection_error_mm=float(payload["mean_reprojection_error_mm"]),
            max_reprojection_error_mm=float(payload["max_reprojection_error_mm"]),
        )

    def ensure_resolution(self, width: int, height: int) -> None:
        if width != self.image_width or height != self.image_height:
            raise WorkspaceCalibrationError(
                "当前相机分辨率与标定分辨率不一致。"
                f" 当前: {width}x{height}，标定: {self.image_width}x{self.image_height}。"
            )

    def pixel_to_table(self, u: float, v: float) -> tuple[float, float]:
        point = np.array([[[u, v]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(point, self.matrix)
        x_mm, y_mm = transformed[0][0]
        return self._apply_robot_axis_mapping(float(x_mm), float(y_mm))

    @staticmethod
    def _apply_robot_axis_mapping(x_mm: float, y_mm: float) -> tuple[float, float]:
        if WORKSPACE_SWAP_XY:
            x_mm, y_mm = y_mm, x_mm

        x_mm = x_mm * WORKSPACE_X_SIGN + WORKSPACE_X_OFFSET_MM
        y_mm = y_mm * WORKSPACE_Y_SIGN + WORKSPACE_Y_OFFSET_MM
        return float(x_mm), float(y_mm)
