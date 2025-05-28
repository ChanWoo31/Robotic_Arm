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

# DEVICENAME        = '/dev/ttyUSB0'
DEVICENAME        = 'COM6'
BAUDRATE          = 1000000
PROTOCOL_VERSION  = 1


ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30        # 2 byte
ADDR_MOVING_SPEED     = 32        # 2 byte
ADDR_PRESENT_POSITION = 36        # 2 byte
ADDR_PRESENT_SPEED    = 38        # 2 byte


CW_Angle_Limit = 6        
CCW_Angle_Limit    = 8        
Moving_Speed = 32

TORQUE_ENABLE   = 1
TORQUE_DISABLE  = 0

# Joint IDs
JOINT_IDS = [1, 2, 3, 4]

# Convert radians to Dynamixel position units
def deg_to_dxl(deg: float) -> int:
    deg = max(0.0, min(300.0, deg))
    return int(deg / 300.0 * 1023)

def dxl_to_deg(val: int) -> float:
    return val / 1023.0 * 300.0

class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        # Port & packet
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"Failed to open port {device}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Failed to set baudrate {baud}")
        self.packet = PacketHandler(PROTOCOL_VERSION)

    # def CW_Limit(self, dxl_id, dx_val):
    #     self.packet.write2ByteTxRx(self.port, dxl_id, CW_Angle_Limit, dx_val)

    # def CCW_Limit(self, dxl_id, dx_val):
    #     self.packet.write2ByteTxRx(self.port, dxl_id, CCW_Angle_Limit, dx_val)

    def motor_speed(self, dxl_id, speed_val):
        self.packet.write2ByteTxRx(self.port, dxl_id, Moving_Speed, speed_val)

    def enable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_goal_position(self, dxl_id, dxl_val):
        self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, dxl_val)

    def get_present_position(self, dxl_id):
        val, _, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return dxl_to_deg(val)

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
    A2 = dh_transform(theta2, d[1], a[1], alpha_[1])
    A3 = dh_transform(theta3, d[2], a[2], alpha_[2])
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
   
    # theta_deg = theta_deg - [0,90,90] # DH파라미터를 모터에 맞게 영점 조절
    theta_deg = [theta_deg[0]* (1023 / 300),-(90-theta_deg[1])* (1023 / 300),(90-theta_deg[2])* (1023 / 300)]  
    theta_deg = np.array(theta_deg)  # 배열로 변환
    motor_val = (theta_deg).astype(int)
    motor_val = 512 + motor_val
    return motor_val

def deg2dxl_array(theta_deg): #일반 배열 전용
    theta_deg = np.array(theta_deg)  # 배열로 변환

    theta_deg = theta_deg 
    motor_val = (theta_deg * (1023 / 300)).astype(int)
    motor_val = 512 + motor_val
    return motor_val

def deg2dxl(theta_deg): #숫자 전용
    motor_val = int(theta_deg * (1023 / 300))
    motor_val = 512 + motor_val
    return motor_val

def theta_condition(thetas):
    if -thetas[2] >= thetas[1]:
        thetas = thetas + [0,90,180]
    return thetas

# 메인 실행
if __name__ == "__main__":
    Dxl = DXLController()

    print("\n 목표 위치를 입력 (X,Y,Z 단위 : mm)")
    target_str = input("목표 위치: ")
    x, y, z = map(float, target_str.strip().split(","))
    target_pos = np.array([x, y, z])

    thetas, success = inverse_kinematics_numeric(target_pos)

    print("\n결과:")

    # thetas = thetas + [0,90,180] # 보정치 => 시작 위치 조정
    thetas = theta_condition(thetas)
    print("θ1: {:.2f}°\nθ2: {:.2f}°\nθ3: {:.2f}°".format(*thetas))

    dxl_val = deg2dxl_theta(thetas)
    print("motor1: {:.2f} motor2: {:.2f} motor1: {:.2f}".format(*dxl_val))

    T_final, A_matrices = forward_kinematics(thetas)
    end_effector_pos = T_final[:3, 3]

    print(end_effector_pos)

    
    print("\n엔드이펙터 위치:", end_effector_pos)
    print("목표 위치:", target_pos)

    error = np.linalg.norm(end_effector_pos - target_pos)
    print("위치 오차 (유클리드 거리): {:.3f} mm".format(error))

    # if error > 10:
    #     print("오류")
    #     thetas = 0

    # if np.abs((dxl_val[2]-512) * (dxl_val[1]-512)) < 0 and np.abs((dxl_val[2]) * (dxl_val[1])) < 50:
    #     print("충돌위험")
    #     dxl_val = 0

    plot_arm(A_matrices, target_pos=target_pos, title="그래프")
    


    Speed = [100,100,100]

    for i in range(len(Speed)):
        Dxl.enable_torque(1+i)
        Dxl.motor_speed(1+i,Speed[i])
    
    init_pos = deg2dxl_array([0,0,0])
    for i in range(len(init_pos)):
        print(init_pos)
        Dxl.set_goal_position(1+i,init_pos[i])
        print("ssss")


    
    print("delay")
    time.sleep(3)

    go2_pos = dxl_val
    for i in range(len(go2_pos)):
        Dxl.set_goal_position(1+i,go2_pos[i])
        print(go2_pos[i])

    # Dxl.set_goal_position(3,300)
    time.sleep(10)
    # while(True):


