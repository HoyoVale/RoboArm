"""
配置文件
"""

from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]              # 项目根目录
MATRIX_DIR = PROJECT_ROOT / "matrixs"                           # 存放标定矩阵的目录
HOMOGRAPHY_MATRIX_PATH = MATRIX_DIR / "homography_matrix.npy"   # 单应矩阵文件路径
HOMOGRAPHY_META_PATH = MATRIX_DIR / "homography_meta.json"      # 标定元数据文件路径
YOLO_MODEL_PATH = PROJECT_ROOT / "yolo11n.pt"                   # YOLO模型文件路径    

# 标定输出坐标 -> 机械臂基底坐标 的二次映射
# 当四点标定的轴定义与机械臂基底坐标轴不完全一致时，在这里做补偿。
WORKSPACE_SWAP_XY = False
WORKSPACE_X_SIGN = 1.0
WORKSPACE_Y_SIGN = 1.0
WORKSPACE_X_OFFSET_MM = 0.0
WORKSPACE_Y_OFFSET_MM = 0.0

# 闭式运动学参数
KINEMATICS_L6_NOMINAL_MM = 44.0
IK_COARSE_STEP_DEG = 2.0
IK_FINE_STEP_DEG = 0.5
IK_MAX_POSITION_ERROR_MM = 3.0

# 相机标定所需参数
CAMERA_INDEX = 1
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720

# 机械臂控制参数
SERIAL_PORT = "COM7"                                    # 串口名称，根据实际情况修改
SERIAL_BAUD = 9600                                      # 串口通信波特率

# 抓取和放置相关参数
PICK_Z_MM = 70.0                                        # 抓取高度，单位 mm
APPROACH_CLEARANCE_MM = 30.0                            # 机械臂接近物体时的安全高度，单位 mm
LIFT_CLEARANCE_MM = 50.0                                # 抓取后提升的安全高度，单位 mm

PLACE_X_MM = 200.0                                      # 放置位置的 X 坐标，单位 mm
PLACE_Y_MM = 120.0                                      # 放置位置的 Y 坐标，单位 mm
PLACE_Z_MM = 70.0                                       # 放置高度，单位 mm

DEFAULT_GRAB_PUMP_MS = 500                              # 默认吸泵吸取时间，单位 ms
HOME_MOVE_MS = 800                                    # 机械臂移动到初始位置的时间，单位 ms
APPROACH_MOVE_MS = 800                                 # 机械臂接近物体的移动时间，单位 ms
DESCEND_MOVE_MS = 800                                  # 机械臂下降到物体表面的移动时间，单位 ms
LIFT_MOVE_MS = 800                                     # 机械臂提升物体的移动时间，单位 ms
PLACE_MOVE_MS = 800                                    # 机械臂移动到放置位置的移动时间，单位 ms
MOVE_SETTLE_SEC = 2.0                                  # 机械臂移动后稳定时间，单位 s
GRAB_SETTLE_SEC = 2.0                                  # 抓取后稳定时间，单位 s
RELEASE_SETTLE_SEC = 2.0                               # 释放后稳定时间，单位 s

DEFAULT_TRIGGER_MODE = "manual"                         # 默认触发模式，manual 或 auto
AUTO_STABLE_FRAMES = 10                                 # 自动模式下连续稳定帧数要求    
AUTO_STABLE_POSITION_JITTER_MM = 10.0                   # 自动模式下位置抖动阈值，单位 mm
AUTO_COOLDOWN_SEC = 3.0                                 # 自动模式下两次抓取之间的冷却时间，单位 s   

YOLO_ALLOWED_CLASSES = ("apple", "orange", "banana")    # YOLO模型允许检测的类别
YOLO_CONFIDENCE = 0.60                                  # YOLO模型置信度阈值
YOLO_BBOX_FOOT_RATIO = 0.10                             # YOLO边界框底部比例

COLOR_LABEL = "red"                                     # 颜色标签
COLOR_MIN_RADIUS_PX = 20.0                              # 最小颜色半径，单位 px
COLOR_CONTACT_BACKOFF_PX = 6                            # 颜色接触后退距离，单位 px
COLOR_LOWER_RED_1 = (0, 120, 70)                        # 红色下限1
COLOR_UPPER_RED_1 = (10, 255, 255)                      # 红色上限1
COLOR_LOWER_RED_2 = (170, 120, 70)                      # 红色下限2
COLOR_UPPER_RED_2 = (180, 255, 255)                     # 红色上限2


def ensure_runtime_dirs() -> None:
    MATRIX_DIR.mkdir(parents=True, exist_ok=True)
