# 上位机通信模块说明

这份说明面向后续要开发视觉识别、策略决策或自动控制程序的人。

## 文件位置

- 可直接 `import` 的通信模块：
  [robot_arm_controller.py](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/tools/control/robot_arm_controller.py)
- 命令行调试脚本：
  [arm_controller_sender.py](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/tools/control/arm_controller_sender.py)
- 运动学校准探针：
  [kinematics_probe.py](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/tools/control/kinematics_probe.py)
- 最小示例脚本：
  [robot_arm_example.py](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/tools/control/robot_arm_example.py)

## 依赖

```bash
python -m pip install pyserial
```

## 串口参数

- 波特率：`9600`
- 数据格式：`8N1`
- 默认端口示例：`COM5`

注意：

- 同一时刻只能有一个程序占用串口
- 正式上位机运行时，不要同时开调试脚本
- 推荐主程序启动后保持串口常开，不要频繁开关

## 模块导入方式

如果你的主程序放在项目根目录下，可以这样导入：

```python
from tools.control.robot_arm_controller import RobotArmController
```

## 已封装接口

`RobotArmController` 提供这些方法：

- `move_angles({1: 60, 2: 65, 3: 55}, 800)`
- `move_pulses({1: 1500, 2: 1600}, 800)`
- `move_angles_precise({1: 60.4, 2: 65.2, 3: 57.9}, 800)`
- `pump_on()`
- `pump_off()`
- `valve_close()`
- `valve_open()`
- `grab(500)`
- `release()`
- `stop()`
- `home(800)`
- `save_home()`
- `send_raw("#1A60T800")`

## 关节定义

- `1号舵机`：底座绕 Z 轴旋转
- `2号舵机`：大臂前伸后缩
- `3号舵机`：小臂上下运动

## 当前默认 Home 位姿

- `1号 = 60°`
- `2号 = 65°`
- `3号 = 55°`

如果已经执行过 `save_home()`，则重启后优先恢复已保存位姿。

## 气泵与电磁阀逻辑

当前硬件验证过的抓取流程：

1. `valve_close()`
2. `pump_on()`
3. 物体吸住后 `pump_off()`
4. 放下时 `valve_open()`

也可以直接使用：

- `grab(500)`：抽气一段时间后自动停泵并保持阀门关闭
- `release()`：打开阀门并释放物体

## 最小示例

```python
from tools.control.robot_arm_controller import RobotArmController

with RobotArmController(port="COM5", baud=9600) as arm:
    arm.home(800)
    arm.move_angles({1: 60, 2: 65, 3: 55}, 800)
    arm.valve_close()
    arm.pump_on()
    arm.pump_off()
    arm.valve_open()
```

## 建议的上位机调用方式

推荐把机械臂控制对象长期保留：

```python
arm = RobotArmController(port="COM5", baud=9600)
arm.open()
```

然后在视觉循环里按决策调用：

- `arm.move_angles(...)`
- `arm.grab(...)`
- `arm.release()`

程序退出前：

```python
arm.close()
```

## 运动学校准建议

如果要先脱离视觉系统校准运动学，建议直接使用：

```bash
python tools/control/kinematics_probe.py --port COM7 shell
```

进入后可用命令：

- `model`
- `ik 200 120 70`
- `fk 60 65 55`
- `goto 200 120 70 time=800`
- `angles 60 65 55 time=800`
- `home time=800`

这个脚本会同时打印：

- 当前模型参数
- `IK` 求出的解析候选解，最多两组
- 每个候选对应的 `θ1 / φ4`
- 每个候选的 `FK` 回代坐标和误差
- 主链模型下的 `yaw / radial / z`
- 最终选中的舵机角和对应 pulse

说明：

- `goto` 这类通过 `IK` 计算得到的动作，当前会优先用 `pulse` 模式下发，以保留小数角带来的精度。
- `angles 60 65 55` 这类你手动输入的命令，仍然保留原来的整数角模式，方便人工调试。
- 当前已经调好的几何标定参数保存在 [config.py](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/src/config.py)：
  - `SERVO1_YAW_SCALE = 1.85`
  - `SERVO2_THETA1_SCALE = 1.75`
  - `SERVO3_PHI4_SCALE = 1.75`
  如果后续又重新校过，请同步改文档。

适合先确认：

- 哪些桌面点可达
- 新主链解析 `IK`/`FK` 是否自洽
- 实际落点与理论落点的误差方向
