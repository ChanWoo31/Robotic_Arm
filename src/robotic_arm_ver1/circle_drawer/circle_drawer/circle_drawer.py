import time
import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS
import sys
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink

JOINT_IDS = [1, 2, 3, 4]
JOINT_ZERO_OFFSET = [150.0, 150.0, 150.0, 150.0]
JOINT_DIRECTION = [1, 1, 1, 1]
JOINT_LIMIT_MIN = [0.0, 0.0, 0.0, 0.0]
JOINT_LIMIT_MAX = [300.0, 300.0, 300.0, 300.0]

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

d1, a2, a3, a4 = 0.135, 0.125, 0.175, 0.055

def deg2dxl(deg: float) -> int:
    deg = max(0.0, min(300.0, deg))
    return int(deg / 300.0 * 1023)

def dxl2deg(val: int) -> float:
    return val / 1023.0 * 300.0


class Controller:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"Fail")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Fail")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 2)

    def enable_torque(self, ids):
        for dxl_id in ids:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, ids):
        for dxl_id in ids:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_goal_positions(self, deg_list):
        self.sync_write.clearParam()
        for idx, dxl_id in enumerate(JOINT_IDS):
            pos = deg2dxl(deg_list[idx])
            param = [pos & 0xFF, (pos >> 8) & 0xFF]
            self.sync_write.addParam(dxl_id, bytes(param))
        result = self.sync_write.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError("SyncWrite 통신 오류")
        
    def get_present_position(self, dxl_id):
        val, comm_result, error = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        if comm_result != COMM_SUCCESS:
            raise RuntimeError(f"Dynamixel 통신 오류 (ID={dxl_id})")
        if error != 0:
            raise RuntimeError(f"Dynamixel 에러 (ID={dxl_id}, error={error})")
        return dxl2deg(val)
    
    def close(self):
        self.port.closePort()

def DH_matrix(theta, d, a, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(q):
    theta1, theta2, theta3, theta4 = q
    A1 = DH_matrix(-theta1, d1, 0, np.deg2rad(90))
    A2 = DH_matrix(-theta2+np.deg2rad(90), 0, a2, 0)
    A3 = DH_matrix(-theta3, 0, a3, 0)
    A4 = DH_matrix(-theta4, 0, a4, 0)
    T04 = A1 @ A2 @ A3 @ A4
    pos = T04[0:3, 3]
    roll = -(theta2 + theta3 + theta4) - np.pi/2
    return pos, roll

def cubic_coeff(theta_i, theta_f, dtheta_i=0.0, dtheta_f=0.0, T=0.0):
    a0 = theta_i
    a1 = dtheta_i
    a2 = (3 * (theta_f - theta_i) - (2 * dtheta_i + dtheta_f) * T) / (T ** 2)
    a3 = (2 * (theta_i - theta_f) + (dtheta_i + dtheta_f) * T) / (T ** 3)
    return a0, a1, a2, a3

def eval_cubic(coeffs, t):
    a0, a1, a2, a3 = coeffs
    return a0 + a1 * t + a2 * t ** 2 + a3 * t ** 3

def inverse_kinematics(x, y, z, roll, d1, a2, a3, a4):
    # 1. q1 (베이스 회전)
    q1 = np.arctan2(y, x)
    
    # 2. Wrist 중심 좌표
    x_w = x - a4 * np.cos(roll) * np.cos(q1)
    y_w = y - a4 * np.cos(roll) * np.sin(q1)
    z_w = z - a4 * np.sin(roll)
    
    r = np.hypot(x_w, y_w)
    s = z_w - d1
    
    # 3. q3 (엘보)
    D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
    D = np.clip(D, -1.0, 1.0)  # 실수 허용범위
    
    q3 = np.arctan2(-np.sqrt(1 - D**2), D)  # elbow-up
    
    # 4. q2 (숄더)
    q2 = np.arctan2(s, r) - np.arctan2(a3 * np.sin(q3), a2 + a3 * np.cos(q3)) - np.pi/2
    
    # 5. q4 (엔드이펙터 pitch)
    q4 = roll - q2 - q3 + np.pi/2
    
    # DH 파라미터에 맞게 부호 및 오프셋 조정 필요할 수 있음
    return np.array([q1, q2, q3, q4])

if __name__ == '__main__':
    global controller
    controller = Controller()
    try:
        controller.enable_torque(JOINT_IDS)
        time.sleep(0.1)

        controller.set_goal_positions(JOINT_ZERO_OFFSET)
        time.sleep(2.0)

        q_current = []
        for idx, j in enumerate(JOINT_IDS):
            deg = controller.get_present_position(j)
            q_cur = JOINT_DIRECTION[idx] * np.deg2rad(deg - JOINT_ZERO_OFFSET[idx])
            q_current.append(q_cur)
        q_current = np.array(q_current)

        # (단위: m, deg)
        while True:
            try:
                vals = input("목표 위치 입력 (x[m], y[m], z[m], roll[deg]) : ").split()
                if len(vals) != 4:
                    print("4개 값(x y z roll)을 입력하세요.")
                    continue
                x_d, y_d, z_d, roll_d = map(float, vals)
                roll_d = np.deg2rad(roll_d)
                break
            except Exception as e:
                print("error")

        # 목표 조인트각 (rad)
        q_goal = inverse_kinematics(x_d, y_d, z_d, roll_d, d1, a2, a3, a4)

        # 트레젝토리 설정
        T = 6.0
        dt = 0.02
        t_steps = np.arange(0, T + dt/2, dt)
        coeffs = [cubic_coeff(qc, qg, 0, 0, T) for qc, qg in zip(q_current, q_goal)]

        print("trajectory")
        start = time.time()
        for step, t in enumerate(t_steps):
            rads = [eval_cubic(c, t) for c in coeffs]
            deg_list = []
            for idx in range(4):
                deg = JOINT_ZERO_OFFSET[idx] + JOINT_DIRECTION[idx]*np.rad2deg(rads[idx])
                deg = max(JOINT_LIMIT_MIN[idx], min(JOINT_LIMIT_MAX[idx], deg))
                deg_list.append(deg)
            controller.set_goal_positions(deg_list)

            # FK 입력 각도는, deg_list(모터각)을 다시 내부 수학 각도로 환산
            FK_input_rads = [
                JOINT_DIRECTION[i] * np.deg2rad(deg_list[i] - JOINT_ZERO_OFFSET[i])
                for i in range(4)
            ]
            pos, roll = forward_kinematics(FK_input_rads)
            print(f"[{t:.2f}s] joint_deg = {[round(d,2) for d in deg_list]}, EE = {np.round(pos,4)}, roll(deg) = {round(np.rad2deg(roll),2)}")
            # print(f"[{t:.2f}s] joint_deg = {[round(d, 2) for d in deg_list]}")
            # 정확한 시간 동기화
            target_time = start + step*dt
            now = time.time()
            to_wait = target_time - now
            if to_wait > 0:
                time.sleep(to_wait)

        print("목표 위치 도달!")

    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        controller.disable_torque(JOINT_IDS)
        controller.close()
        print("Controller shutdown")