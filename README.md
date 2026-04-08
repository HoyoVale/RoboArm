# 吸盘机械臂视觉抓取项目

本项目用于控制 3 自由度吸盘机械臂完成桌面物体识别与抓取。

当前上位机能力包括：
- 四点标定，将图像像素坐标映射到桌面坐标
- YOLO 水果识别抓取
- 红色物体颜色识别抓取
- 串口控制下位机舵机、气泵和电磁阀

## 目录

- [目录结构](#目录结构)
- [环境准备](#环境准备)
  - [1. 硬件准备](#1-硬件准备)
  - [2. Python 环境](#2-python-环境)
  - [3. 下位机联通性检查](#3-下位机联通性检查)
  - [4. 运动学校准检查](#4-运动学校准检查)
- [四点标定步骤](#四点标定步骤)
  - [1. 运行标定工具](#1-运行标定工具)
  - [2. 选择四个图像点](#2-选择四个图像点)
  - [3. 输入对应的桌面物理坐标](#3-输入对应的桌面物理坐标)
  - [4. 标定输出](#4-标定输出)
- [颜色识别抓取步骤](#颜色识别抓取步骤)
  - [1. 功能说明](#1-功能说明)
  - [2. 手动模式](#2-手动模式)
  - [3. 自动模式](#3-自动模式)
  - [4. 调试模式](#4-调试模式)
- [YOLO 识别抓取步骤](#yolo-识别抓取步骤)
  - [1. 功能说明](#1-功能说明-1)
  - [2. 手动模式](#2-手动模式-1)
  - [3. 自动模式](#3-自动模式-1)
  - [4. 指定模型或阈值](#4-指定模型或阈值)
- [`config.py` 使用说明](#configpy-使用说明)
  - [1. 标定坐标修正](#1-标定坐标修正)
  - [2. 运动学参数](#2-运动学参数)
  - [3. 相机参数](#3-相机参数)
  - [4. 串口参数](#4-串口参数)
  - [5. 抓取与放置参数](#5-抓取与放置参数)
  - [6. 动作时序参数](#6-动作时序参数)
  - [7. 自动触发参数](#7-自动触发参数)
  - [8. YOLO 参数](#8-yolo-参数)
  - [9. 颜色识别参数](#9-颜色识别参数)
- [命令行参数说明](#命令行参数说明)
  - [1. 四点标定工具](#1-四点标定工具)
  - [2. YOLO 抓取脚本](#2-yolo-抓取脚本)
  - [3. 颜色抓取脚本](#3-颜色抓取脚本)
- [推荐联调顺序](#推荐联调顺序)
- [常见问题](#常见问题)
  - [1. 程序提示找不到标定矩阵](#1-程序提示找不到标定矩阵)
  - [2. 能识别目标，但程序不抓](#2-能识别目标但程序不抓)
  - [3. 自动模式能识别但不稳定触发](#3-自动模式能识别但不稳定触发)
  - [4. 抓取点偏差很大](#4-抓取点偏差很大)
- [下位机说明](#下位机说明)

## 目录结构

- `src/config.py`
  运行时配置，包含相机、串口、抓取参数、运动学标定参数
- `src/kinematics.py`
  主链解析运动学与舵机标定层
- `src/grasp_executor.py`
  抓取执行器，负责“到目标上方 -> 下压 -> 吸取 -> 抬起 -> 放置”
- `src/main_A_YOLO11_grab.py`
  YOLO 识别抓取入口
- `src/main_B_color_grab.py`
  颜色识别抓取入口
- `tools/calibration/four_point_calibration.py`
  四点标定工具
- `tools/control/robot_arm_controller.py`
  上位机串口控制模块
- `tools/control/kinematics_probe.py`
  运动学校准与纯动作测试工具
- `matrixs/homography_matrix.npy`
  四点标定生成的单应矩阵
- `matrixs/homography_meta.json`
  四点标定元数据

## 环境准备

### 1. 硬件准备

- 机械臂下位机已烧录可用固件
- 机械臂上电后能正常回到 `home`
- USB 摄像头已连接电脑
- 开发板串口已在设备管理器中识别，例如 `COM7`
- 舵机、气泵、电磁阀接线已确认无误

### 2. Python 环境

推荐使用 Python 3.10。

Windows PowerShell 示例：

```powershell
cd D:\hoyo\Documents\Projects\机械臂视觉抓取
python -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

`requirements.txt` 当前包含：
- `numpy`
- `opencv-python`
- `pyserial`
- `ultralytics`

### 3. 下位机联通性检查

建议先用串口调试脚本确认机械臂能动：

```powershell
python tools\control\arm_controller_sender.py --port COM7 shell
```

进入后可测试：

```text
home 800
angle 1:60 2:65 3:55 time=800
valve close
pump on
pump off
valve open
```

### 4. 运动学校准检查

建议在接入视觉前先确认 `IK/FK` 和落点趋势正常：

```powershell
python tools\control\kinematics_probe.py --port COM7 shell
```

常用命令：

```text
model
fk 60 65 55
ik 200 100 70
goto 200 100 70 time=800
angles 60 65 55 time=800
home time=800
```

## 四点标定步骤

四点标定的作用是把图像像素坐标映射到机械臂桌面坐标。

### 1. 运行标定工具

```powershell
python tools\calibration\four_point_calibration.py --index 1 --width 640 --height 480
```

参数说明：
- `--index`
  相机索引
- `--width`
  标定分辨率宽度
- `--height`
  标定分辨率高度

### 2. 选择四个图像点

按固定顺序点击：

1. 左上
2. 右上
3. 右下
4. 左下

辅助按键：
- `r`
  清空已选点
- `q` 或 `Esc`
  选满 4 个点后进入物理坐标输入

### 3. 输入对应的桌面物理坐标

程序会要求输入 4 个机械臂桌面坐标，格式：

```text
200,100
```

注意：
- 单位是 `mm`
- 这 4 个点必须与点击顺序一一对应
- 坐标系以机械臂底座为准：
  - 原点：`L1` 与地面的接触点
  - `X`：机械臂前方
  - `Y`：机械臂左侧
  - `Z`：竖直向上，桌面为 `0`

### 4. 标定输出

标定完成后会生成：
- `matrixs/homography_matrix.npy`
- `matrixs/homography_meta.json`

运行抓取脚本时会强制检查：
- 当前相机索引
- 当前分辨率

如果运行分辨率和标定分辨率不一致，程序会直接拒绝启动。

## 颜色识别抓取步骤

颜色识别版本用于快速联调，不依赖深度学习模型。

### 1. 功能说明

- 识别对象：红色物体
- 抓取点：轮廓中心
- 支持 `manual` 和 `auto`

### 2. 手动模式

```powershell
python src\main_B_color_grab.py --mode manual --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70
```

操作方式：
- `g`
  对当前识别到的目标执行抓取
- `q`
  退出程序

建议先用手动模式验证：
- 标定是否准确
- 运动学是否准确
- 抓取和放置流程是否正常

### 3. 自动模式

```powershell
python src\main_B_color_grab.py --mode auto --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70
```

自动模式逻辑：
- 目标连续稳定若干帧后自动触发
- 两次抓取之间有冷却时间
- 抓取执行期间不会重复触发

### 4. 调试模式

如果怀疑“颜色识别到了，但不抓”，可以打开筛选调试：

```powershell
python src\main_B_color_grab.py --mode manual --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70 --debug-select
```

它会打印：
- 轮廓数量
- 轮廓半径
- 对应桌面坐标
- 是否被判定为可抓取

## YOLO 识别抓取步骤

YOLO 版本用于识别水果模型并抓取。

### 1. 功能说明

- 默认模型：`yolo11n.pt`
- 默认允许类别：`apple`、`orange`、`banana`
- 抓取点：检测框中心
- 支持 `manual` 和 `auto`

### 2. 手动模式

```powershell
python src\main_A_YOLO11_grab.py --mode manual --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70
```

### 3. 自动模式

```powershell
python src\main_A_YOLO11_grab.py --mode auto --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70
```

### 4. 指定模型或阈值

```powershell
python src\main_A_YOLO11_grab.py --mode auto --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70 --model yolo11n.pt --conf 0.6
```

## `config.py` 使用说明

运行配置文件在 `src/config.py`。

建议先改配置，再运行程序，不要边运行边改。

### 1. 标定坐标修正

```python
WORKSPACE_SWAP_XY = False
WORKSPACE_X_SIGN = 1.0
WORKSPACE_Y_SIGN = 1.0
WORKSPACE_X_OFFSET_MM = 0.0
WORKSPACE_Y_OFFSET_MM = 0.0
```

用途：
- 当四点标定得到的坐标轴与机械臂实际坐标轴不完全一致时，在这里做补偿

含义：
- `WORKSPACE_SWAP_XY`
  是否交换 `X/Y`
- `WORKSPACE_X_SIGN`
  `X` 方向取反
- `WORKSPACE_Y_SIGN`
  `Y` 方向取反
- `WORKSPACE_X_OFFSET_MM`
  `X` 方向平移补偿
- `WORKSPACE_Y_OFFSET_MM`
  `Y` 方向平移补偿

### 2. 运动学参数

```python
KINEMATICS_L6_NOMINAL_MM = 34.0
SERVO1_YAW_SCALE = 1.85
SERVO1_YAW_BIAS_DEG = 0.0
SERVO2_THETA1_SCALE = 1.70
SERVO2_THETA1_BIAS_DEG = 0.0
SERVO3_PHI4_SCALE = 1.75
SERVO3_PHI4_BIAS_DEG = 0.0
```

用途：
- 对 1、2、3 号舵机分别做角度比例和固定偏差标定

当前模型约定：
- `yaw_deg = servo1_deg - servo1_home`
- `theta1_deg = servo2_home - servo2_deg`
- `phi4_deg = servo3_deg - servo3_home`

含义：
- `SERVO1_YAW_SCALE`
  一号舵机命令角到实际偏航角的比例
- `SERVO1_YAW_BIAS_DEG`
  一号舵机偏航固定偏差
- `SERVO2_THETA1_SCALE`
  二号舵机命令角到实际 `theta1` 的比例
- `SERVO2_THETA1_BIAS_DEG`
  二号舵机 `theta1` 固定偏差
- `SERVO3_PHI4_SCALE`
  三号舵机命令角到实际 `phi4` 的比例
- `SERVO3_PHI4_BIAS_DEG`
  三号舵机 `phi4` 固定偏差

### 3. 相机参数

```python
CAMERA_INDEX = 1
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
```

用途：
- 控制默认摄像头和默认分辨率

### 4. 串口参数

```python
SERIAL_PORT = "COM7"
SERIAL_BAUD = 9600
```

用途：
- 指定下位机串口和波特率

### 5. 抓取与放置参数

```python
PICK_Z_MM = 60.0
APPROACH_CLEARANCE_MM = 30.0
LIFT_CLEARANCE_MM = 100.0
PLACE_X_MM = 200.0
PLACE_Y_MM = 200.0
PLACE_Z_MM = 70.0
```

含义：
- `PICK_Z_MM`
  抓取高度，单位 `mm`
  参考零点是桌面 `Z=0`
- `APPROACH_CLEARANCE_MM`
  目标上方预留安全高度
- `LIFT_CLEARANCE_MM`
  抓取后提升高度
- `PLACE_X_MM / PLACE_Y_MM / PLACE_Z_MM`
  默认放置点

注意：
- `PICK_Z_MM`
  决定真正抓取时吸盘中心的目标高度
- 如果你把 `LIFT_CLEARANCE_MM` 改得太高，目标可能会被提前判成“不可达”

### 6. 动作时序参数

```python
DEFAULT_GRAB_PUMP_MS = 500
HOME_MOVE_MS = 800
APPROACH_MOVE_MS = 800
DESCEND_MOVE_MS = 800
LIFT_MOVE_MS = 800
PLACE_MOVE_MS = 800
MOVE_SETTLE_SEC = 2.0
GRAB_SETTLE_SEC = 2.0
RELEASE_SETTLE_SEC = 2.0
```

用途：
- 调整回位、接近、下压、抬起、放置等动作时间
- 调整抓取后和释放后的稳定等待时间

### 7. 自动触发参数

```python
DEFAULT_TRIGGER_MODE = "auto"
AUTO_STABLE_FRAMES = 10
AUTO_STABLE_POSITION_JITTER_MM = 10.0
AUTO_COOLDOWN_SEC = 3.0
```

用途：
- 控制自动抓取模式的触发条件和冷却时间

### 8. YOLO 参数

```python
YOLO_ALLOWED_CLASSES = ("apple", "orange", "banana")
YOLO_CONFIDENCE = 0.60
```

用途：
- 限定允许抓取的类别
- 设置默认置信度阈值

### 9. 颜色识别参数

```python
COLOR_LABEL = "red"
COLOR_MIN_RADIUS_PX = 20.0
COLOR_LOWER_RED_1 = (0, 120, 70)
COLOR_UPPER_RED_1 = (10, 255, 255)
COLOR_LOWER_RED_2 = (170, 120, 70)
COLOR_UPPER_RED_2 = (180, 255, 255)
```

用途：
- 控制红色目标的 HSV 识别范围
- `COLOR_MIN_RADIUS_PX` 用于过滤过小噪声目标

## 命令行参数说明

### 1. 四点标定工具

```powershell
python tools\calibration\four_point_calibration.py --index 1 --width 640 --height 480
```

参数：
- `--index`
  相机索引
- `--width`
  图像宽度
- `--height`
  图像高度

### 2. YOLO 抓取脚本

```powershell
python src\main_A_YOLO11_grab.py --mode auto --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70
```

参数：
- `--mode`
  `manual` 或 `auto`
- `--port`
  串口号，例如 `COM7`
- `--camera-index`
  相机索引
- `--width`
  图像宽度
- `--height`
  图像高度
- `--pick-z`
  抓取高度，单位 `mm`
- `--model`
  YOLO 模型路径
- `--conf`
  置信度阈值
- `--cooldown-sec`
  自动模式冷却时间
- `--stable-frames`
  自动模式判稳所需连续帧数
- `--stable-jitter-mm`
  自动模式位置抖动阈值

### 3. 颜色抓取脚本

```powershell
python src\main_B_color_grab.py --mode auto --port COM7 --camera-index 1 --width 640 --height 480 --pick-z 70
```

参数：
- `--mode`
  `manual` 或 `auto`
- `--port`
  串口号
- `--camera-index`
  相机索引
- `--width`
  图像宽度
- `--height`
  图像高度
- `--pick-z`
  抓取高度，单位 `mm`
- `--cooldown-sec`
  自动模式冷却时间
- `--stable-frames`
  自动模式判稳所需连续帧数
- `--stable-jitter-mm`
  自动模式位置抖动阈值
- `--debug-select`
  打印颜色目标筛选调试信息

## 推荐联调顺序

1. 用 `arm_controller_sender.py` 确认串口通信和气泵阀门正常
2. 用 `kinematics_probe.py` 确认 `IK/FK` 和落点趋势正常
3. 跑四点标定
4. 先跑颜色识别手动模式
5. 再跑 YOLO 手动模式
6. 最后再开自动模式

## 常见问题

### 1. 程序提示找不到标定矩阵

先重新做四点标定，确保以下文件存在：
- `matrixs/homography_matrix.npy`
- `matrixs/homography_meta.json`

### 2. 能识别目标，但程序不抓

常见原因：
- 目标被 `can_execute_target()` 判为不可达
- `pick-z` 不合适
- `LIFT_CLEARANCE_MM` 或放置点设置过高
- 当前分辨率和标定分辨率不一致

### 3. 自动模式能识别但不稳定触发

检查：
- `AUTO_STABLE_FRAMES`
- `AUTO_STABLE_POSITION_JITTER_MM`
- `AUTO_COOLDOWN_SEC`

### 4. 抓取点偏差很大

优先检查：
- 四点标定是否准确
- `WORKSPACE_*` 坐标轴修正
- 三个舵机标定参数
- `PICK_Z_MM`

## 下位机说明

下位机工程位于：

- `stc15_robot_arm_controller/`

编译与刷写说明见：

- `stc15_robot_arm_controller/README.md`
- `tools/control/README.md`
- `docs/机械臂参数/README.md`
