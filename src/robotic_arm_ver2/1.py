import numpy as np
from numpy import sin, cos, arctan2, radians, degrees
import matplotlib.pyplot as plt
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
import time

d = [0, 0, 0]
a = [0, 140, 140]
alpha_ = [90, -180, 90]

T0 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 105.5],
    [0, 0, 0, 1]
])

past_theta = [0,0,0]
past_dxl_val = [0,0,0]
DEVICENAME        = '/dev/ttyUSB0'
# DEVICENAME        = 'COM6'
BAUDRATE          = 1000000
PROTOCOL_VERSION  = 2.0


ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116        # 2 byte
ADDR_MOVING_SPEED     = 112        # 2 byte
ADDR_PRESENT_POSITION = 132        # 2 byte
ADDR_PRESENT_SPEED    = 128        # 2 byte


CW_Angle_Limit = 48      
CCW_Angle_Limit    = 52      
Moving_Speed = 32

TORQUE_ENABLE   = 1
TORQUE_DISABLE  = 0

ADDR_MX_TORQUE_LIMIT = 38
TORQUE_LIMIT = 1023

# Joint IDs
JOINT_IDS = [1, 2, 3]

connect_pass = False

# Convert radians to Dynamixel position units
# def deg_to_dxl(deg: float) -> int:
#     deg = max(0.0, min(300.0, deg))
#     return int(deg / 300.0 * 1023)

def dxl_to_deg(val: int) -> float:
    return val / 4096.0 * 360.0

class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        # Port & packet
        if connect_pass == False:
            self.port = PortHandler(device)
            if not self.port.openPort():
                raise IOError(f"Failed to open port {device}")
            if not self.port.setBaudRate(baud):
                raise IOError(f"Failed to set baudrate {baud}")
            self.packet = PacketHandler(PROTOCOL_VERSION)
            self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 4)
            
            for i in range(4):
                self.packet.write2ByteTxRx(self.port, i+1, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)

        else:
            pass

    def CW_Limit(self, dxl_id, dx_val):
        self.packet.write2ByteTxRx(self.port, dxl_id, CW_Angle_Limit, dx_val)

    def CCW_Limit(self, dxl_id, dx_val):
        self.packet.write2ByteTxRx(self.port, dxl_id, CCW_Angle_Limit, dx_val)

    def motor_speed(self, dxl_id, speed_val): # Moving_Speed -> ADDR_MOVING_SPEED 
        self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, speed_val)
        pass

    def enable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        pass

    def disable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_goal_position(self, dxl_val):
        self.sync_write.clearParam()
        for idx, dxl_id in enumerate(JOINT_IDS):
            # print(int(dxl_val[idx]))
            pos = int(dxl_val[idx])  # 각 모터 목표 위치 값, int로 변환            
            param = [pos & 0xFF, (pos >> 8) & 0xFF,
                     (pos >> 16) & 0xFF, (pos >> 24) & 0xFF]
            self.sync_write.addParam(dxl_id, bytes(param))
        result = self.sync_write.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError("SyncWrite 통신 오류")
        pass

    def get_present_position(self, dxl_id):
        val, _, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return val
        pass

    def close(self):
        self.port.closePort()

def dh_transform(theta, d_, a_, alpha):
    theta_rad = radians(theta)
    alpha_rad = radians(alpha)
    return np.array([
        [cos(theta_rad), -sin(theta_rad)*cos(alpha_rad),  sin(theta_rad)*sin(alpha_rad), a_*cos(theta_rad)],
        [sin(theta_rad),  cos(theta_rad)*cos(alpha_rad), -cos(theta_rad)*sin(alpha_rad), a_*sin(theta_rad)],
        [0,               sin(alpha_rad),                cos(alpha_rad),               d_],
        [0,               0,                            0,                           1]
    ])

# 평행 링크 보정
def get_a_end(theta2_deg, theta3_deg):
    theta4_deg = -(theta2_deg + theta3_deg)
    return dh_transform(theta4_deg, 0, 0, 0)

def forward_kinematics(thetas):
    theta1, theta2, theta3 = thetas
    A1 = dh_transform(theta1, d[0], a[0], alpha_[0])
    A2 = dh_transform(theta2+90, d[1], a[1], alpha_[1])
    A3 = dh_transform(theta3+90, d[2], a[2], alpha_[2])
    A_end = get_a_end(theta2, theta3)
    T = T0 @ A1 @ A2 @ A3 @ A_end
    return T, [A1, A2, A3, A_end]

def inverse_kinematics_numeric(target_pos, initial_guess=[0, 0, 0], max_iter=1000, lr=0.1, tol=1e-2): #수치미분법
    theta = np.array(initial_guess, dtype=float)
    for i in range(max_iter):
        T, _ = forward_kinematics(theta)
        pos = T[:3, 3]
        error = target_pos - pos
        if np.linalg.norm(error) < tol:
            return theta, True
        
        # 단순 수치 미분
        grad = np.zeros(3)
        delta = 1e-3
        for j in range(3):
            theta_temp = np.copy(theta)
            theta_temp[j] += delta
            pos_plus = forward_kinematics(theta_temp)[0][:3, 3]
            grad[j] = ((pos_plus - pos) @ error) / delta

        theta += lr * grad

    return theta, False

def plot_arm(A_matrices, target_pos=None, title="3D 로봇팔 시각화"):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = [0], [0], [0]
    T = T0
    for Ai in A_matrices:
        T = T @ Ai
        pos = T[:3, 3]
        xs.append(pos[0])
        ys.append(pos[1])
        zs.append(pos[2])

    ax.plot(xs, ys, zs, '-o', linewidth=3, markersize=8, label="Arm")
    ax.scatter(*target_pos, color='red', s=100, label='Target')

    ax.set_xlim([-300, 300])
    ax.set_ylim([-300, 300])
    ax.set_zlim([0, 400])
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(title)
    ax.legend()
    plt.grid(True)
    plt.show()
    
def deg2dxl_theta(theta_deg): #DH파라미터 전용
    theta_deg = np.array(theta_deg)  # 배열로 변환    
    theta_link = theta_deg
    theta_link[2] = theta_link[2] - theta_link[1] 
    motor_val = (theta_deg * (4096 / 360)).astype(int)
    # motor_val[2] = motor_val[2] - motor_val[1]
    
    motor_val = 2048 + motor_val
    return motor_val

def deg2dxl_array(theta_deg): #일반 배열 전용
    theta_deg = np.array(theta_deg)  # 배열로 변환
    theta_deg = theta_deg 
    motor_val = (theta_deg * (4096 / 360)).astype(int)
    motor_val = 2048 + motor_val
    return motor_val

def deg2dxl(theta_deg): #숫자 전용
    motor_val = int(theta_deg * (4096 / 360))
    motor_val = 2048 + motor_val
    return motor_val

def theta_condition(thetas):
    # if -thetas[2] >= thetas[1]:
    #     thetas = thetas + [0,90,180]
    return thetas

# 메인 실행
if __name__ == "__main__":
    Dxl = DXLController()

    Speed = [50,50,50]

    for i in range(len(Speed)):
        Dxl.enable_torque(1+i)
        Dxl.motor_speed(1+i,Speed[i])
    
    # init_pos = deg2dxl_array([0,0,0])
    init_pos = [2048,1400,2048]
    Dxl.set_goal_position(init_pos)

    print("이동 중")
    time.sleep(2)

    print("\n 목표 위치와 반지름을 입력 (X,Y,Z,R 단위 : mm)")
    target_str = input("목표 위치: ")
    x, y, z, r = map(float, target_str.strip().split(","))
    target_pos = np.array([x, y, z])

    eclipse_a = 50
    eclipse_b = 30

    point_num = 60
    theta = np.linspace(0, 2 * np.pi, point_num,endpoint=False)  # 0부터 2π까지 균등하게 나누기
    # circle_points = np.zeros((point_num, 3))
    # circle_points[:, 0] = x + r * np.cos(theta)
    # circle_points[:, 1] = y + r * np.sin(theta)  
    # circle_points[:, 2] = z         
    circle_points = np.column_stack([
        x + eclipse_a * np.cos(theta),
        y + eclipse_b * np.sin(theta),
        np.full(point_num, z)
    ])   

    # circle_points = np.array(circle_points).astype(int)
    for j, point in enumerate(circle_points):
        thetas, success = inverse_kinematics_numeric(point)
        print("\n결과:")

        thetas = theta_condition(thetas)
        print("θ1: {:.2f}°\nθ2: {:.2f}°\nθ3: {:.2f}°".format(*thetas))

        dxl_val = deg2dxl_theta(thetas)
        print("motor1: {:.2f} motor2: {:.2f} motor3: {:.2f}".format(*dxl_val))

        T_final, A_matrices = forward_kinematics(thetas)
        end_effector_pos = T_final[:3, 3]

        
        print("\n엔드이펙터 위치:", end_effector_pos)
        print("목표 위치:", circle_points[j])

        error = np.linalg.norm(end_effector_pos - circle_points[j])
        print("위치 오차 (유클리드 거리): {:.3f} mm".format(error))

        if error > 10 and j == 0:
            print("오류")
            # plot_arm(A_matrices, target_pos=circle_points[j], title="그래프")
            thetas = past_theta

        # if np.abs((dxl_val[2]-512) * (dxl_val[1]-512)) < 0 and np.abs((dxl_val[2]) * (dxl_val[1])) < 50:
        #     print("충돌 위험")
        #     dxl_val = past_dxl_val

        # plot_arm(A_matrices, target_pos=target_pos, title="그래프")
        
        Speed = [250,250,250]

        for i in range(len(Speed)):
            Dxl.enable_torque(1+i)
            Dxl.motor_speed(1+i,Speed[i])

        go2_pos = dxl_val
        Dxl.set_goal_position(go2_pos)
        for i in range(len(go2_pos)):
            print("이동중_원 위치")
            print(Dxl.get_present_position(1+i))
            print("목표1: {:.2f} 목표2: {:.2f} 목표3: {:.2f}".format(*go2_pos))
            print("현재1: {:.2f} 현재2: {:.2f} 현재3: {:.2f}".format(*[Dxl.get_present_position(1),Dxl.get_present_position(2),Dxl.get_present_position(3)]))
            # time.sleep(1)

        

        past_theta = thetas
        past_dxl_val = dxl_val

int