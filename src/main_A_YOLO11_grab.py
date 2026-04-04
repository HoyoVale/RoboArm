import cv2
import numpy as np
import time
import sys
import os
from ultralytics import YOLO

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
    # 1. 初始化 YOLO 模型
    print("[*] 正在加载 YOLO11 模型...")
    # 请确保 yolo11n.pt 放在同级目录或指定绝对路径
    model = YOLO("yolo11n.pt") 
    
    # 2. 初始化逆运动学和机械臂
    arm_ik = PalletizingArmKinematics()
    print("[*] 正在连接机械臂...")
    arm_ctrl = RobotArmController(port="COM5") # 请修改实际COM口
    time.sleep(2)
    
    # 3. 加载标定矩阵
    try:
        matrix_path = os.path.join(os.path.dirname(__file__), "homography_matrix.npy")
        H = np.load(matrix_path)
    except FileNotFoundError:
        print("[-] 未找到 homography_matrix.npy，请先运行标定工具。")
        return

    # 4. 打开摄像头
    cap = cv2.VideoCapture(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 定义不同水果的抓取高度 Z 补偿 (单位: mm)
    # COCO 数据集类别 ID: 47=apple, 49=orange, 46=banana
    HEIGHT_COMPENSATION = {
        'apple': 60,   
        'orange': 50,  
        'banana': 30   
    }

    PLACE_X = 200
    PLACE_Y = 200
    PLACE_Z = 50
    is_grabbing = False

    print("\n[*] YOLO 视觉抓取系统已启动！按 'q' 退出。")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if not is_grabbing:
            # 运行 YOLO 推理 (只检测特定类别可加上 classes=[46, 47, 49])
            results = model(frame, verbose=False, conf=0.6)
            
            target_detected = False
            best_target = None
            
            # 解析检测结果
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    cls_id = int(box.cls[0])
                    class_name = model.names[cls_id]
                    
                    # 如果是我们关心的水果
                    if class_name in HEIGHT_COMPENSATION:
                        # 获取边界框中心点
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        pixel_u = int((x1 + x2) / 2)
                        pixel_v = int((y1 + y2) / 2)
                        
                        best_target = (pixel_u, pixel_v, class_name)
                        target_detected = True
                        
                        # 绘制框和中心点
                        cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                        cv2.circle(frame, (pixel_u, pixel_v), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f"{class_name}", (int(x1), int(y1)-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                        break # 找到一个就退出循环，先抓一个
                if target_detected:
                    break

            if target_detected:
                pixel_u, pixel_v, class_name = best_target
                cv2.putText(frame, f"Found {class_name}! Press 'g' to grab", (50, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('g'):
                    is_grabbing = True
                    target_x, target_y = pixel_to_physical(pixel_u, pixel_v, H)
                    
                    # 获取该类别对应的高度补偿
                    target_z = HEIGHT_COMPENSATION[class_name]
                    print(f"\n[+] 准备抓取 {class_name} | 物理坐标: X={target_x:.1f}, Y={target_y:.1f}, Z={target_z}")
                    
                    angles = arm_ik.inverse_kinematics(target_x, target_y, target_z)
                    
                    if angles:
                        # 运动控制序列
                        # 传入字典 {舵机ID: 角度值}
                        arm_ctrl.move_angles({1: int(angles[0]), 2: int(angles[1]), 3: int(angles[2])}, move_time_ms=1500)
                        time.sleep(2)
                        arm_ctrl.grab() # 使用自带的 grab() 方法更安全
                        time.sleep(1.5)
                        arm_ctrl.home(move_time_ms=1000)
                        time.sleep(1.5)
                        
                        place_angles = arm_ik.inverse_kinematics(PLACE_X, PLACE_Y, PLACE_Z)
                        if place_angles:
                            arm_ctrl.move_angles({1: int(place_angles[0]), 2: int(place_angles[1]), 3: int(place_angles[2])}, move_time_ms=1500)
                            time.sleep(2)
                            arm_ctrl.release() # 使用自带的 release() 方法
                            time.sleep(1)
                            
                        arm_ctrl.home(move_time_ms=1000)
                        time.sleep(1.5)
                    else:
                        print("[-] 目标超出机械臂工作范围！")
                    
                    is_grabbing = False

        cv2.imshow("YOLOv11 Vision Grasping", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    arm_ctrl.close()

if __name__ == "__main__":
    main()