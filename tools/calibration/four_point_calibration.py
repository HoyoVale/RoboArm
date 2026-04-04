import cv2
import numpy as np
import os

# 全局变量
image_points = []  # 存储鼠标点击的图像像素点 (u, v)
window_name = "Camera Calibration (Click 4 points)"

def mouse_callback(event, x, y, flags, param):
    """
    鼠标回调函数：监听鼠标左键点击事件，记录点击的像素坐标
    """
    global image_points
    
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(image_points) < 4:
            image_points.append((x, y))
            print(f"[*] 记录图像点 {len(image_points)}: (u={x}, v={y})")
        else:
            print("[!] 已经选满 4 个点了，如果选错了请按 'r' 键重新选择。")

def main():
    global image_points
    
    # 1. 打开摄像头 (默认是 0，如果是外接 USB 摄像头可能是 1 或 2)
    # 你可以根据实际情况修改这里的设备号
    cap = cv2.VideoCapture(1) 
    
    if not cap.isOpened():
        print("[-] 错误：无法打开摄像头，请检查摄像头是否连接或被其他程序占用。")
        return

    # 设置摄像头分辨率 (建议设置高一点，比如 1280x720，以提高标定精度)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    print("\n" + "="*50)
    print(" 📷 四点标定工具启动")
    print("="*50)
    print("操作说明：")
    print("1. 确保摄像头已经固定好，并且可以看到桌面的抓取区域。")
    print("2. 在弹出的画面中，用鼠标左键依次点击你标记的 4 个点。")
    print("   (建议按顺序点击：左上、右上、右下、左下，形成一个矩形)")
    print("3. 按 'r' 键可以清空已选点，重新选择。")
    print("4. 选满 4 个点后，按 'q' 键或者 'Esc' 键退出画面并进入坐标输入环节。")
    print("="*50 + "\n")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[-] 错误：无法读取摄像头画面。")
            break
            
        display_frame = frame.copy()
        
        # 在画面上绘制已经点击的点
        for i, pt in enumerate(image_points):
            # 画圆点
            cv2.circle(display_frame, pt, 5, (0, 0, 255), -1)
            # 标注序号
            cv2.putText(display_frame, str(i+1), (pt[0]+10, pt[1]-10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            # 如果点大于等于2个，连线形成多边形轮廓
            if i > 0:
                cv2.line(display_frame, image_points[i-1], pt, (255, 0, 0), 2)
            if len(image_points) == 4 and i == 3:
                cv2.line(display_frame, image_points[3], image_points[0], (255, 0, 0), 2)
                
        # 提示文字
        cv2.putText(display_frame, f"Points selected: {len(image_points)}/4", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        cv2.imshow(window_name, display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('r'):
            # 按 'r' 重新选择
            image_points = []
            print("\n[*] 已清空选择的点，请重新点击。")
            
        elif key == ord('q') or key == 27: # 27 是 Esc 键
            # 按 'q' 或 'Esc' 退出画面
            if len(image_points) == 4:
                print("\n[*] 画面关闭，准备计算矩阵。")
                break
            else:
                print(f"\n[!] 警告：你只选了 {len(image_points)} 个点，必须选满 4 个点才能退出计算！")
                print("请继续选择，或按 Ctrl+C 强制结束程序。")

    cap.release()
    cv2.destroyAllWindows()
    
    # -----------------------------
    # 输入物理坐标并计算变换矩阵
    # -----------------------------
    if len(image_points) == 4:
        print("\n" + "="*50)
        print(" 📐 请输入对应的机械臂物理坐标")
        print("="*50)
        print("请依次输入刚才点击的 4 个点在机械臂基坐标系下的 (X, Y) 坐标，单位 mm。")
        print("注意顺序必须与你在画面中点击的顺序完全一致！\n")
        
        robot_points = []
        for i in range(4):
            while True:
                try:
                    user_input = input(f"请输入第 {i+1} 个点的物理坐标 (格式: x,y)，例如 200,100: ")
                    # 简单解析输入的逗号分隔字符串
                    x_str, y_str = user_input.split(',')
                    x = float(x_str.strip())
                    y = float(y_str.strip())
                    robot_points.append((x, y))
                    break
                except ValueError:
                    print("[-] 输入格式错误，请确保输入两个数字并用英文逗号分隔。")
                    
        # 转换为 numpy 数组，格式要求为 float32
        pts_src = np.array(image_points, dtype=np.float32)
        pts_dst = np.array(robot_points, dtype=np.float32)
        
        # 计算透视变换矩阵 (Homography)
        # H 是一个 3x3 的矩阵，可以将图像点映射到物理平面点
        H, status = cv2.findHomography(pts_src, pts_dst)
        
        print("\n" + "="*50)
        print(" ✅ 计算完成！透视变换矩阵 (Homography Matrix):")
        print(H)
        
        # 保存矩阵到文件，方便后续的主程序直接读取
        save_path = os.path.join(os.path.dirname(__file__), "homography_matrix.npy")
        np.save(save_path, H)
        print(f"\n[*] 矩阵已成功保存至: {save_path}")
        print("后续抓取程序可以使用 np.load('homography_matrix.npy') 加载它。")
        print("="*50 + "\n")

if __name__ == "__main__":
    main()