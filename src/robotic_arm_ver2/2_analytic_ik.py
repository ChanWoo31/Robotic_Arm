import numpy as np
from numpy import sin, cos, radians, degrees
import time
import math

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite

# ------------------ 기존 파라미터 정의부 ------------------
# (단위: mm, deg)
d = [0, 0, 0]
a = [0, 140, 140]
alpha_ = [90, -180, 90]

# 베이스(어깨) z-offset: 실제 측정값 = 118 mm
T0_z = 118  # mm

# ------------------ 모터 “영 영(Zero) 위치” 보정값 ------------------
Theta2_offset = 0.0   # <--- 측정한 “어깨(2번 모터) 오프셋(°)”를 넣으세요.
Theta3_offset = 0.0   # <--- 측정한 “엘보(3번 모터) 오프셋(°)”를 넣으세요.
# -----------------------------------------------------------------------------------

DEVICENAME        = '/dev/ttyUSB0'
BAUDRATE          = 1000000
PROTOCOL_VERSION  = 2.0

ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116        # 2 byte
ADDR_MOVING_SPEED     = 112        # 2 byte
ADDR_PRESENT_POSITION = 132        # 2 byte

CW_Angle_Limit   = 48      
CCW_Angle_Limit  = 52      
Moving_Speed     = 32

TORQUE_ENABLE   = 1
TORQUE_DISABLE  = 0

ADDR_MX_TORQUE_LIMIT = 38
TORQUE_LIMIT         = 1023

# Joint IDs
JOINT_IDS = [1, 2, 3]

# ----------------- DXLController 클래스 (원본 그대로) -----------------
class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"Failed to open port {device}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Failed to set baudrate {baud}")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 4)
        # 모터 토크 리밋 설정
        for i in range(3):
            self.packet.write2ByteTxRx(self.port, JOINT_IDS[i], ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)

    def motor_speed(self, dxl_id, speed_val):
        self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, speed_val)

    def enable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_goal_position(self, dxl_positions):
        """
        dxl_positions: [pos1, pos2, pos3], 각 pos는 32-bit 정수
         → 4바이트 little-endian으로 분해해서 GroupSyncWrite로 전송
        """
        self.sync_write.clearParam()
        for idx, dxl_id in enumerate(JOINT_IDS):
            val = int(dxl_positions[idx])
            param = [
                val & 0xFF,
                (val >> 8) & 0xFF,
                (val >> 16) & 0xFF,
                (val >> 24) & 0xFF
            ]
            self.sync_write.addParam(dxl_id, bytes(param))

        result = self.sync_write.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError("SyncWrite 통신 오류 발생")

    def get_present_position(self, dxl_id):
        val, _, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return val

    def close(self):
        self.port.closePort()
# ---------------------------------------------------------------------


# ------------------ DH 순기구학 함수 ------------------
def dh_transform(theta, d_, a_, alpha):
    theta_rad = radians(theta)
    alpha_rad = radians(alpha)
    return np.array([
        [cos(theta_rad), -sin(theta_rad)*cos(alpha_rad),  sin(theta_rad)*sin(alpha_rad), a_*cos(theta_rad)],
        [sin(theta_rad),  cos(theta_rad)*cos(alpha_rad), -cos(theta_rad)*sin(alpha_rad), a_*sin(theta_rad)],
        [0,               sin(alpha_rad),                cos(alpha_rad),                 d_],
        [0,               0,                              0,                              1]
    ])

def forward_kinematics(thetas):
    """
    thetas: [θ1, θ2, θ3_rel] (deg)
    1) A1 = dh_transform(theta1, d[0], a[0], alpha_[0])   # 베이스→어깨 (a[0]=0)
    2) A2 = dh_transform(theta2+90, d[1], a[1], alpha_[1])# 어깨→팔꿈치 (a[1]=140, α=-180)
    3) A3 = dh_transform(theta3_rel+90, d[2], a[2], alpha_[2]) # 팔꿈치→끝단 (a[2]=140, α=+90)
    4) T_sim = A1 @ A2 @ A3
    → end_sim = T_sim[:3, 3]
    """
    theta1, theta2, theta3_rel = thetas

    A1 = dh_transform(theta1,     d[0], a[0], alpha_[0])   # a[0] = 0
    A2 = dh_transform(theta2 + 90, d[1], a[1], alpha_[1])
    A3 = dh_transform(theta3_rel + 90, d[2], a[2], alpha_[2])

    T_sim = A1 @ A2 @ A3
    end_sim = T_sim[:3, 3]
    return end_sim

def forward_kinematics_with_offset(thetas):
    """
    end_sim = forward_kinematics(thetas)
    실제 예상 좌표 end_real = end_sim + offset_vec
    offset_vec = [260, 0, 135]  # 실제 영점(θ1=θ2=θ3_rel=0 시 측정값)
    """
    end_sim = forward_kinematics(thetas)
    offset_vec = np.array([260.0, 0.0, 135.0])  # **여기를 변경**
    end_real = end_sim + offset_vec
    return end_real
# ----------------------------------------------------------------------------


# ------------------ 해석적(analytic) IK 함수 ------------------
def analytic_inverse_kinematics(sim_target):
    """
    sim_target: [x', y', z'] (mm) – 이미 base 높이(T0_z)와 오프셋이 제거된 시뮬레이션 좌표
    반환: [θ1, θ2, θ3_rel] (deg) or None
    """
    x, y, z = sim_target
    L1 = a[1]  # 140 mm
    L2 = a[2]  # 140 mm

    # 1) base yaw
    if x == 0 and y == 0:
        return None
    theta1 = math.degrees(math.atan2(y, x))

    # 2) planar 거리 계산
    r = math.hypot(x, y)
    z_rel = z
    D = math.hypot(r, z_rel)
    if D > (L1 + L2) or D < abs(L1 - L2):
        return None

    # 3) law of cosines (엘보 각)
    cos_elbow = (L1*L1 + L2*L2 - D*D) / (2 * L1 * L2)
    cos_elbow = max(-1.0, min(1.0, cos_elbow))
    elbow_angle = math.acos(cos_elbow)
    theta3_rel = math.degrees(math.pi - elbow_angle) - 90.0

    # 4) law of cosines (숄더 각)
    cos_shoulder = (L1*L1 + D*D - L2*L2) / (2 * L1 * D)
    cos_shoulder = max(-1.0, min(1.0, cos_shoulder))
    shoulder_offset = math.acos(cos_shoulder)
    gamma = math.atan2(z_rel, r)
    theta2 = math.degrees(gamma + shoulder_offset) - 90.0

    return [theta1, theta2, theta3_rel]
# --------------------------------------------------------------------------------


def inverse_kinematics_with_offset(real_target_pos_mm):
    """
    real_target_pos_mm: [x_real, y_real, z_real] (mm) – 사용자가 입력한 실제 좌표
    1) sim_target = real_target_pos_mm - offset_vec (offset_vec = [260, 0, 135])
    2) analytic_inverse_kinematics(sim_target)
    """
    offset_vec = np.array([260.0, 0.0, 135.0])  # **여기를 동일하게 변경**
    sim_target = real_target_pos_mm - offset_vec
    return analytic_inverse_kinematics(sim_target)
# --------------------------------------------------------------------------------


# ------------------ DH 결과 → DXL 값 변환 (모터 오프셋 + 방향 반전) ------------------
def deg2dxl_theta(theta_deg):
    """
    theta_deg: IK가 반환한 [θ1_calc, θ2_calc, θ3_rel] (deg)

    (1) 모터 2번 오프셋 적용: theta2_phys = θ2_calc + Theta2_offset
    (2) 모터 3번 회전 방향 반전 & 오프셋:
        θ3_phys = - (θ2_calc + θ3_rel) + Theta3_offset
    (3) 모터 1번(offset 없음 가정: θ1_phys = θ1_calc)

    그 뒤, 12-bit(0~4095) DXL 값으로 변환:
      motor_val = 2048 + int(theta_motor * (4096/360))
    """
    theta1_calc, theta2_calc, theta3_rel = theta_deg

    theta1_phys = theta1_calc
    theta2_phys = theta2_calc + Theta2_offset
    theta3_phys = -(theta2_calc + theta3_rel) + Theta3_offset

    theta_motor = np.array([theta1_phys, theta2_phys, theta3_phys])
    motor_val = (theta_motor * (4096.0 / 360.0)).astype(int)
    motor_val = 2048 + motor_val
    return motor_val
# ----------------------------------------------------------------------------------------


# ============================================
#                메인 실행부
# ============================================
if __name__ == "__main__":
    Dxl = DXLController()

    # 1) 각 모터 토크 활성화 및 속도 세팅 (초기)
    Speed_init = [50, 50, 50]
    for i in range(3):
        Dxl.enable_torque(JOINT_IDS[i])
        Dxl.motor_speed(JOINT_IDS[i], Speed_init[i])

    # 2) 초기 위치 [0, 0, 0] 세팅
    init_pos = deg2dxl_theta([0.0, 0.0, 0.0])
    Dxl.set_goal_position(init_pos)
    time.sleep(1.0)

    # 3) 사용자로부터 “원” 정보 입력
    print("\n 목표 위치와 반지름을 입력 (X, Y, Z, R 단위: mm, 예시: 160,0,100,0)")
    target_str = input("목표 위치: ")
    x, y, z, r = map(float, target_str.strip().split(","))
    target_pos = np.array([x, y, z])

    # 4) 원 위의 포인트 생성 (100개). R=0이면 하나만 계산됩니다.
    point_num = 100 if r != 0 else 1
    theta_vals = np.linspace(0, 2 * np.pi, point_num)
    circle_points = np.zeros((point_num, 3), dtype=float)
    circle_points[:, 0] = x + r * np.cos(theta_vals)
    circle_points[:, 1] = y + r * np.sin(theta_vals)
    circle_points[:, 2] = z

    prev_thetas = [0.0, 0.0, 0.0]
    prev_dxl_vals = None

    # 5) 원 따라 이동(또는 한 점 이동) 루프
    for j in range(point_num):
        px, py, pz = circle_points[j]
        real_pt = np.array([px, py, pz])

        # 5-1) IK 계산 (Backward Compensation)
        ik_result = inverse_kinematics_with_offset(real_pt)
        if ik_result is None:
            print(f"[경고] {j:03d}번째 점 ({px:.2f}, {py:.2f}, {pz:.2f}) unreachable.")
            continue

        thetas = ik_result  # [θ1, θ2, θ3_rel]

        print("\n결과 (θ1, θ2, θ3_rel):", np.round(thetas, 2))

        # 5-2) 시뮬 FK → 오프셋 보정 → 오차 확인
        end_real = forward_kinematics_with_offset(thetas)
        error = np.linalg.norm(end_real - real_pt)
        print(f"엔드 위치 (실제 예상): {np.round(end_real,2)},  목표(actual): {np.round(real_pt,2)},  오차: {error:.3f} mm")
        if error > 10:
            print("[오류] 오차 > 10 mm, 명령 생략")
            continue

        # 5-3) 모터 값으로 변환
        dxl_vals = deg2dxl_theta(thetas)
        if prev_dxl_vals is not None and np.all(np.abs(dxl_vals - prev_dxl_vals) < 5):
            # 지난 명령값과 크게 차이 없으면 생략
            continue

        # 5-4) 속도 세팅 후 목표 위치 전송
        Speed_move = [300, 300, 300]
        for i in range(3):
            Dxl.enable_torque(JOINT_IDS[i])
            Dxl.motor_speed(JOINT_IDS[i], Speed_move[i])

        Dxl.set_goal_position(dxl_vals)
        prev_dxl_vals = dxl_vals.copy()

        print("이동중_점 위치 → DXL 명령:", dxl_vals)
        time.sleep(0.02)  # 20 ms 간격

    # 6) 완료 후 초기 위치(0,0,0)로 복귀
    time.sleep(1.0)
    Dxl.close()
