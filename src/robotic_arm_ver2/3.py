# 시도해볼것

import numpy as np
from numpy import sin, cos, arctan2, radians
import matplotlib.pyplot as plt
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
import time

# ——————————————————————————————————————————————————
# 로봇팔 매개변수 및 제어 설정
# ——————————————————————————————————————————————————

# DH 파라미터 (단위: mm, deg)
d = [0, 0, 0]
a = [0, 140, 140]
alpha_ = [90, -180, 90]

# 베이스부터 엔드이펙터까지 변환 행렬 초기값
T0 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 115],   # 엔드이펙터 기준 높이 115mm
    [0, 0, 0, 1]
])

# Dynamixel 통신 설정
DEVICENAME       = '/dev/ttyUSB0'
BAUDRATE         = 1000000
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116  # 4 byte
ADDR_MOVING_SPEED     = 112  # 4 byte
ADDR_PRESENT_POSITION = 132  # 4 byte
ADDR_MX_TORQUE_LIMIT  = 38

TORQUE_ENABLE   = 1
TORQUE_DISABLE  = 0
TORQUE_LIMIT    = 1023

JOINT_IDS = [1, 2, 3]

# 속도 설정 (예시)
DEFAULT_SPEED = [300, 300, 300]

# ——————————————————————————————————————————————————
# DXLController 클래스 (로봇 제어용)
# ——————————————————————————————————————————————————

class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"Failed to open port {device}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Failed to set baudrate {baud}")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        # SyncWrite 설정: 목표위치(ADDR_GOAL_POSITION) 4바이트
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 4)

        # 모든 모터 토크 리밋 설정
        for dxl_id in JOINT_IDS:
            self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)

    def enable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_speed(self, dxl_id, speed_val):
        self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, int(speed_val))

    def set_goal_position(self, dxl_positions):
        """
        dxl_positions: [pos1, pos2, pos3] (각 pos는 32-bit 정수)
        GroupSyncWrite로 한 번에 전송
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
            if not self.sync_write.addParam(dxl_id, bytes(param)):
                raise RuntimeError(f"Failed to add Param for ID {dxl_id}")
        result = self.sync_write.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError("SyncWrite 통신 오류 발생")

    def get_present_position(self, dxl_id):
        val, _, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return val

    def close(self):
        for dxl_id in JOINT_IDS:
            self.disable_torque(dxl_id)
        self.port.closePort()

# ——————————————————————————————————————————————————
# DH 변환 행렬 계산 함수
# ——————————————————————————————————————————————————

def dh_transform(theta, d_, a_, alpha):
    """
    입력 각도 theta(deg), DH 파라미터 (d_, a_, alpha)
    회전/병진 변환 행렬 반환
    """
    theta_rad = radians(theta)
    alpha_rad = radians(alpha)
    return np.array([
        [ cos(theta_rad), -sin(theta_rad)*cos(alpha_rad),  sin(theta_rad)*sin(alpha_rad), a_*cos(theta_rad)],
        [ sin(theta_rad),  cos(theta_rad)*cos(alpha_rad), -cos(theta_rad)*sin(alpha_rad), a_*sin(theta_rad)],
        [          0.0,               sin(alpha_rad),               cos(alpha_rad),              d_],
        [          0.0,                        0.0,                        0.0,              1.0]
    ])

def get_a_end(theta2_deg, theta3_deg):
    """
    평행 링크 보정을 위해 4번째 자유도(가상) 계산
    """
    theta4_deg = -(theta2_deg + theta3_deg)
    return dh_transform(theta4_deg, 0, 0, 0)

def forward_kinematics(thetas):
    """
    thetas: [theta1, theta2, theta3] (deg)
    로봇팔의 전체 순방향 운동학 계산
    반환: (4x4 변환 행렬 T, [A1, A2, A3] 각 링크별 변환 행렬 리스트)
    """
    theta1, theta2, theta3 = thetas
    A1 = dh_transform(theta1,        d[0], a[0], alpha_[0])
    A2 = dh_transform(theta2 + 90.0, d[1], a[1], alpha_[1])
    A3 = dh_transform(theta3 + 90.0, d[2], a[2], alpha_[2])
    # 평행 링크 보정은 생략하고 A_end는 T0에 포함하지 않음
    T = T0 @ A1 @ A2 @ A3
    return T, [A1, A2, A3]

# ——————————————————————————————————————————————————
# 관절각 <→ Dynamixel 값 변환 함수
# ——————————————————————————————————————————————————

def deg2dxl(theta_deg):
    """
    단일 각도(deg)를 Dynamixel 0~4095 범위(360deg)로 변환
    """
    raw = int(theta_deg * (4096.0 / 360.0))
    return 2048 + raw

def deg2dxl_array(theta_deg_list):
    """
    리스트 형태의 각도(deg) 배열을 Dynamixel 값 배열로 변환
    """
    arr = np.array(theta_deg_list, dtype=float)
    raw = (arr * (4096.0 / 360.0)).astype(int)
    return (2048 + raw).tolist()

# ——————————————————————————————————————————————————
# 수치 역기구학 함수 (기존 로직 유지)
# ——————————————————————————————————————————————————

def inverse_kinematics_numeric(target_pos, tol=0.1, max_iter=1400):
    """
    target_pos: [x, y, z] (mm)
    단순 수치방식으로 theta1, theta2 계산 후 theta3 도출 (deg)
    반환: (thetas 배열 [theta1, theta2, theta3], success 여부)
    """
    x, y, z = target_pos
    origin = np.array([0, 0, 115.0])

    # 1) theta1: arctan2 계산
    theta1 = np.rad2deg(np.arctan2(y, x))
    flag_theta2 = False
    theta2_deg = 0.0

    # 2) theta2 검색 (0~ ...)
    if theta1 is not None:
        for _ in range(max_iter):
            t2 = np.deg2rad(theta2_deg)
            point1 = np.array([140.0 * np.cos(t2), 140.0 * np.sin(t2)])
            point2 = np.array([np.sqrt(x**2 + y**2), z - origin[2]])
            dist = np.linalg.norm(point1 - point2)
            if abs(dist - 140.0) < tol:
                flag_theta2 = True
                break
            theta2_deg += 0.1

    # 3) 보조: 뒤집힌 x축(θ1+180)
    if not flag_theta2:
        theta1 += 180.0
        theta2_deg = 0.0
        for _ in range(max_iter):
            t2 = np.deg2rad(theta2_deg)
            point1 = np.array([140.0 * np.cos(t2), 140.0 * np.sin(t2)])
            point2 = np.array
