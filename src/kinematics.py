import math

class PalletizingArmKinematics:
    def __init__(self):
        # 机械臂参数 (单位: mm)
        self.L1 = 260.0  # 大臂长度
        self.L2 = 260.0  # 小臂等效长度
        
        # 考虑到机械结构可能存在初始偏置，这些值可以在实际联调时微调
        # 根据物理校准：标准姿态 (大臂垂直向上，小臂水平向前，底座朝前) 对应的硬件角度为:
        # 1号: 60度, 2号: 70度, 3号: 60度
        self.servo1_zero = 60.0
        self.servo2_zero = 70.0
        self.servo3_zero = 60.0
        
        # 机械臂结构特性：
        # 根据图片标注，260mm 是从【底座最底部】一直到【大臂顶端】的总高度。
        # 因此，在我们的数学模型中，大臂旋转的虚拟支点就相当于在 Z=0（桌面）的位置，
        # 不需要再加底座高度偏置了。

    def inverse_kinematics(self, x, y, z):
        """
        逆运动学求解：输入目标空间坐标 (x, y, z)，返回三个舵机的角度 (theta1, theta2, theta3)
        坐标系定义 (物理世界坐标系)：
        - 原点：机械臂底座旋转中心在地面的投影点
        - X轴：正前方 (单位 mm)
        - Y轴：左侧为正 (单位 mm)
        - Z轴：以桌面为 0 的高度 (单位 mm)
        """
        # -----------------------------
        # 1. 计算1号舵机（底座旋转角度）
        # -----------------------------
        # 目标点在 XY 平面上的投影距离
        r = math.sqrt(x**2 + y**2)
        
        # 安全限制：检查是否在工作范围内 (170mm ~ 400mm)
        if r < 170.0 or r > 400.0:
            print(f"警告：目标点水平距离 {r:.1f} mm 超出工作范围 (170-400mm)！")
            return None
            
        # 1号舵机：角度增加 -> 向左旋转。物理零点是 60度。
        # 假设底座旋转角 theta1_rad 在 [-pi/2, pi/2] 之间 (右为负，左为正)
        theta1_rad = math.atan2(y, x)
        # 角度 = 零点 + 计算出的偏移角度
        theta1_deg = self.servo1_zero + math.degrees(theta1_rad)

        # -----------------------------
        # 2. 计算2号和3号舵机（大臂和小臂角度）
        # -----------------------------
        # 在 r-z 平面上，目标点距离原点的直线距离 L3
        L3 = math.sqrt(r**2 + z**2)
        
        # 检查是否超长 (无法到达)
        if L3 > (self.L1 + self.L2):
            print("警告：目标点太远，机械臂无法到达！")
            return None

        # 使用余弦定理求内角
        # a: 大臂与 L3 的夹角
        cos_a = (self.L1**2 + L3**2 - self.L2**2) / (2 * self.L1 * L3)
        # 限制 cos_a 在 [-1, 1] 之间，防止浮点数精度问题导致 math.acos 报错
        cos_a = max(-1.0, min(1.0, cos_a))
        angle_a = math.acos(cos_a)

        # beta: L3 与水平面(r轴)的夹角
        beta = math.atan2(z, r)

        # 计算大臂与水平面的真实仰角
        # 注意：这里有两种解（肘部朝上或肘部朝下），对于这种机械臂通常采用肘部朝上(Elbow Up)的姿态
        theta2_rad = beta + angle_a
        
        # 计算大小臂之间的夹角 (这里算的是小臂连杆与大臂的相对夹角)
        cos_b = (self.L1**2 + self.L2**2 - L3**2) / (2 * self.L1 * self.L2)
        cos_b = max(-1.0, min(1.0, cos_b))
        angle_b = math.acos(cos_b)
        
        # 由于是平行四边形结构，3号舵机控制的是后方拉杆，它等效于控制小臂与水平面的绝对夹角。
        # 几何推导：小臂与水平面的夹角 theta3_rad = theta2_rad - angle_b (或者相对角度，具体取决于3号舵机安装方式)
        # 这里假设 3 号舵机 90 度时小臂水平，且舵机角度与小臂相对水平面的仰角成正比。
        theta3_rad = theta2_rad - angle_b 

        # 转换为角度 (假设大臂水平向前为0度，垂直向上为90度)
        theta2_deg = math.degrees(theta2_rad)
        theta3_deg = math.degrees(theta3_rad)

        # -----------------------------
        # 3. 映射到实际舵机角度 (0-180度)
        # -----------------------------
        # 2号舵机：角度减小 -> 向前倾斜。这意味着算法算出的仰角 theta2_deg 越大，硬件角度越小。
        # 标准姿态时大臂垂直 (算法角度 90度) -> 硬件角度 70度
        # 当大臂向前倾斜 (算法角度 < 90度) -> 硬件角度 > 70度 (因为硬件减小是向前，增加是向后)
        # 映射关系: 硬件角度 = 70 - (算法角度 - 90) = 160 - 算法角度
        final_theta2 = 160.0 - theta2_deg
        
        # 3号舵机：角度增加 -> 向上抬起。
        # 标准姿态时小臂水平 (算法角度 0度) -> 硬件角度 60度
        # 映射关系: 硬件角度 = 60 + 算法角度
        final_theta3 = self.servo3_zero + theta3_deg
        
        # 限制角度范围
        final_theta1 = max(0.0, min(180.0, theta1_deg))
        final_theta2 = max(0.0, min(180.0, final_theta2))
        final_theta3 = max(0.0, min(180.0, final_theta3))
        
        return (round(final_theta1, 1), round(final_theta2, 1), round(final_theta3, 1))

# --- 测试代码 ---
if __name__ == "__main__":
    arm = PalletizingArmKinematics()
    
    # 测试点 1：正前方，水平距离 200mm，高度 50mm
    print("测试点 1 (x=200, y=0, z=50):")
    angles = arm.inverse_kinematics(260, 0, 260)
    if angles:
        print(f"舵机角度 -> 1号(底座): {angles[0]}°, 2号(大臂): {angles[1]}°, 3号(小臂): {angles[2]}°\n")

    # 测试点 2：左前方，x=150, y=150 (距离约212mm), 高度 0mm
    print("测试点 2 (x=150, y=150, z=0):")
    angles = arm.inverse_kinematics(270, 10, 270)
    if angles:
        print(f"舵机角度 -> 1号(底座): {angles[0]}°, 2号(大臂): {angles[1]}°, 3号(小臂): {angles[2]}°\n")