# 四点标定工具说明

这套项目使用的是普通 USB 相机，所以第一版只做桌面平面的四点标定，不做相机内参和深度恢复。

## 原理

四点标定求的是图像像素坐标 `(u, v)` 到桌面物理坐标 `(X, Y)` 的单应矩阵 `H`。

它只对同一个平面上的点严格成立，所以这里的标定对象必须是桌面上的 4 个点。

## 标定脚本

- 脚本位置：
  [four_point_calibration.py](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/tools/calibration/four_point_calibration.py)

运行示例：

```bash
python tools/calibration/four_point_calibration.py --index 1 --width 1280 --height 720
```

## 点击顺序

点击顺序固定为：

`左上 -> 右上 -> 右下 -> 左下`

输入机械臂桌面坐标时，也必须按这个顺序输入。

## 输出文件

标定完成后会统一保存到项目根目录的 [matrixs](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/matrixs)：

- [homography_matrix.npy](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/matrixs/homography_matrix.npy)
- [homography_meta.json](/mnt/d/hoyo/Documents/Projects/机械臂视觉抓取/matrixs/homography_meta.json)

其中 `homography_meta.json` 会记录：

- 相机索引
- 标定分辨率
- 点顺序
- 4 个图像点
- 4 个物理点
- 回投误差

## 运行期约束

上位机主程序会强制检查：

- 当前运行分辨率必须和标定分辨率一致
- 当前相机索引如果和标定记录不一致，会给出警告

如果分辨率不一致，程序会直接拒绝运行。

## 注意

- 标定只负责 `像素 -> 桌面 XY`
- 水果有高度，所以抓取点不能直接用目标框中心点
- 当前 YOLO 脚本会取检测框的下缘接触点估计
- 当前颜色脚本会取轮廓最低点再向上回缩少量像素
