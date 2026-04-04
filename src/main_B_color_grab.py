import cv2
import numpy as np
import time
import sys
import os

# 将父目录加入路径以便导入 tools 模块
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.control.robot_arm_controller import RobotArmController
from kinematics import PalletizingArmKinematics

def pixel_to_physical(u, v, H):
    """使用透视变换矩阵将像素坐标转换为物理坐标"""
    pts = np.array([[[u, v]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(pts, H)
    return dst[0][0][0], dst[0][0][1]

def main():
    # 1. 初始化逆运动学和机械臂
    arm_ik = PalletizingArmKinematics()
    
    print("[*] 正在连接机械臂...")
    # 请根据实际情况修改 COM 端口号
    arm_ctrl = RobotArmController(port="COM5") 
    time.sleep(2) # 等待机械臂复位
    
    # 2. 加载标定矩阵
    try:
        matrix_path = os.path.join(os.path.dirname(__file__), "homography_matrix.npy")
        H = np.load(matrix_path)
        print("[+] 成功加载透视变换矩阵！")
    except FileNotFoundError:
        print("[-] 未找到 homography_matrix.npy，请先运行 四点标定工具.py")
        return

    # 3. 打开摄像头
    cap = cv2.VideoCapture(1) # 根据你的环境可能是0或1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 放置区坐标 (抓到东西后放到这里)，请根据实际情况修改
    PLACE_X = 200
    PLACE_Y = 200
    PLACE_Z = 50

    is_grabbing = False # 状态锁，防止重复抓取

    print("\n[*] 系统初始化完成，开始检测红色物体...")
    print("按 'q' 退出程序。")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if not is_grabbing:
            # 转换为 HSV 颜色空间寻找红色
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # 红色的 HSV 范围 (有两个区间)
            lower_red1 = np.array([0, 120, 70])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([170, 120, 70])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
            
            # 形态学操作去噪
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)

            # 寻找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # 找最大的轮廓
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                # 如果面积足够大，认为是目标
                if radius > 20:
                    pixel_u, pixel_v = int(x), int(y)
                    cv2.circle(frame, (pixel_u, pixel_v), int(radius), (0, 255, 255), 2)
                    cv2.circle(frame, (pixel_u, pixel_v), 5, (0, 0, 255), -1)
                    
                    # 触发抓取条件：目标在画面中相对稳定，按 'g' 键手动触发抓取 (为了安全，先用手动触发测试)
                    cv2.putText(frame, f"Target Locked! Press 'g' to grab", (50, 50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('g'):
                        is_grabbing = True
                        print(f"\n[+] 开始抓取！像素坐标: u={pixel_u}, v={pixel_v}")
                        
                        # 1. 转换为物理坐标
                        target_x, target_y = pixel_to_physical(pixel_u, pixel_v, H)
                        print(f"[+] 物理坐标: X={target_x:.1f}, Y={target_y:.1f}")
                        
                        # 2. 逆运动学求解 (假设红色苹果高度需要 z=60mm 才能吸住)
                        TARGET_Z = 60
                        angles = arm_ik.inverse_kinematics(target_x, target_y, TARGET_Z)
                        
                        if angles:
                            print(f"[+] 计算出舵机角度: {angles}")
                            # 3. 执行抓取序列
                            # 移动到目标上方准备
                            # 传入字典 {舵机ID: 角度值}
                            arm_ctrl.move_angles({1: int(angles[0]), 2: int(angles[1]), 3: int(angles[2])}, move_time_ms=1500)
                            time.sleep(2)
                            
                            # 开启气泵，执行下压抓取
                            arm_ctrl.grab()
                            time.sleep(1.5)
                            
                            # 抬起机械臂 (回原点)
                            arm_ctrl.home(move_time_ms=1000)
                            time.sleep(1.5)
                            
                            # 移动到放置区
                            place_angles = arm_ik.inverse_kinematics(PLACE_X, PLACE_Y, PLACE_Z)
                            if place_angles:
                                arm_ctrl.move_angles({1: int(place_angles[0]), 2: int(place_angles[1]), 3: int(place_angles[2])}, move_time_ms=1500)
                                time.sleep(2)
                                # 释放物体
                                arm_ctrl.release()
                                time.sleep(1)
                                
                            # 回原点准备下一次
                            arm_ctrl.home(move_time_ms=1000)
                            time.sleep(1.5)
                            print("[+] 抓取流程完成。")
                        else:
                            print("[-] 目标超出机械臂工作范围！")
                        
                        is_grabbing = False

        cv2.imshow("Color Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    arm_ctrl.close()

if __name__ == "__main__":
    main()