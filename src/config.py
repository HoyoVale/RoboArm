"""项目运行配置。"""

from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]  # 项目根目录
MATRIX_DIR = PROJECT_ROOT / "matrixs"  # 存放标定矩阵的目录
HOMOGRAPHY_MATRIX_PATH = MATRIX_DIR / "homography_matrix.npy"  # 单应矩阵文件
HOMOGRAPHY_META_PATH = MATRIX_DIR / "homography_meta.json"  # 标定元数据
YOLO_MODEL_PATH = PROJECT_ROOT / "yolo11n.pt"  # YOLO 模型文件

# 标定输出坐标 -> 机械臂基底坐标 的二次映射。
# 当四点标定的轴定义与机械臂基底坐标轴不完全一致时，在这里做补偿。
WORKSPACE_SWAP_XY = False
WORKSPACE_X_SIGN = 1.0
WORKSPACE_Y_SIGN = 1.0
WORKSPACE_X_OFFSET_MM = 0.0
WORKSPACE_Y_OFFSET_MM = 0.0

# 主链解析运动学参数。
KINEMATICS_L6_NOMINAL_MM = 34.0  # 末端执行器名义长度
SERVO1_YAW_SCALE = 1.85  # 一号舵机命令角到实际偏航角的比例
SERVO1_YAW_BIAS_DEG = 0.0  # 一号舵机实际偏航角的固定偏置
SERVO2_THETA1_SCALE = 1.70  # 二号舵机命令角到实际 theta1 的比例
SERVO2_THETA1_BIAS_DEG = 0.0  # 二号舵机实际 theta1 的固定偏置
SERVO3_PHI4_SCALE = 1.75  # 三号舵机命令角到实际 phi4 的比例
SERVO3_PHI4_BIAS_DEG = 0.0  # 三号舵机实际 phi4 的固定偏置

# 相机参数。
CAMERA_INDEX = 1
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# 下位机串口参数。
SERIAL_PORT = "COM7"  # 串口名称
SERIAL_BAUD = 9600  # 串口波特率

# 抓取与放置参数。
PICK_Z_MM = 30.0  # 抓取高度，单位 mm
APPROACH_CLEARANCE_MM = 30.0  # 接近物体时的安全高度，单位 mm
LIFT_CLEARANCE_MM = 100.0  # 抓取后提升的安全高度，单位 mm

PLACE_X_MM = 200.0  # 放置位置 X，单位 mm
PLACE_Y_MM = 200.0  # 放置位置 Y，单位 mm
PLACE_Z_MM = 70.0  # 放置高度，单位 mm

DEFAULT_GRAB_PUMP_MS = 500  # 默认吸泵吸取时间，单位 ms
HOME_MOVE_MS = 800  # 回到 home 的移动时间，单位 ms
APPROACH_MOVE_MS = 800  # 接近物体的移动时间，单位 ms
DESCEND_MOVE_MS = 800  # 下降到抓取面的移动时间，单位 ms
LIFT_MOVE_MS = 800  # 抬升物体的移动时间，单位 ms
PLACE_MOVE_MS = 800  # 移动到放置位置的时间，单位 ms
MOVE_SETTLE_SEC = 2.0  # 机械臂移动后稳定时间，单位 s
GRAB_SETTLE_SEC = 2.0  # 抓取后稳定时间，单位 s
RELEASE_SETTLE_SEC = 2.0  # 释放后稳定时间，单位 s

# 触发策略参数。
DEFAULT_TRIGGER_MODE = "manual"  # manual 或 auto
AUTO_STABLE_FRAMES = 10  # 自动模式下要求连续稳定的帧数
AUTO_STABLE_POSITION_JITTER_MM = 10.0  # 自动模式下的位置抖动阈值，单位 mm
AUTO_COOLDOWN_SEC = 3.0  # 自动模式下两次抓取之间的冷却时间，单位 s

# YOLO 识别参数。
YOLO_ALLOWED_CLASSES = ("apple", "orange", "banana")
YOLO_CONFIDENCE = 0.60

# 颜色识别参数。
COLOR_LABEL = "red"
COLOR_MIN_RADIUS_PX = 20.0  # 最小颜色半径，单位 px
COLOR_LOWER_RED_1 = (0, 120, 70)
COLOR_UPPER_RED_1 = (10, 255, 255)
COLOR_LOWER_RED_2 = (170, 120, 70)
COLOR_UPPER_RED_2 = (180, 255, 255)


def ensure_runtime_dirs() -> None:
    MATRIX_DIR.mkdir(parents=True, exist_ok=True)
