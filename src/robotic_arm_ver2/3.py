import numpy as np
import math
import time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite

# ————— DXLController 정의 —————
DEVICENAME       = '/dev/ttyUSB0'
BAUDRATE         = 1_000_000
PROTOCOL_VERSION = 1.0

ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30  # 2 byte
ADDR_MOVING_SPEED     = 32  # 2 byte
ADDR_PRESENT_POSITION = 36  # 2 byte
TORQUE_ENABLE         = 1
TORQUE_DISABLE        = 0

JOINT_IDS = [1, 2, 3]

class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        # 포트 열기
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"포트 열기 실패: {device}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Baudrate 설정 실패: {baud}")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        # GroupSyncWrite 세팅 (Goal Position)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 2)
        # 토크 한계치(필요시)
        # for i in JOINT_IDS:
        #     self.packet.write2ByteTxRx(self.port, i, ADDR_MX_TORQUE_LIMIT, 1023)

    def enable_torque(self, dxl_id):
        self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def motor_speed(self, dxl_id, speed):
        self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_MOVING_SPEED, int(speed))

    def set_goal_position(self, dxl_vals):
        self.sync_write.clearParam()
        for j, dxl_id in enumerate(JOINT_IDS):
            val = int(dxl_vals[j])
            low  = val & 0xFF
            high = (val >> 8) & 0xFF
            self.sync_write.addParam(dxl_id, bytes([low, high]))
        result = self.sync_write.txPacket()
        if result != COMM_SUCCESS:
            raise RuntimeError("SyncWrite 통신 오류")

    def close(self):
        self.port.closePort()


# ————— 로봇 기구학 함수 —————
# DH 변환
def dh_transform(theta_deg, d, a, alpha_deg):
    th = math.radians(theta_deg)
    al = math.radians(alpha_deg)
    return np.array([
        [ math.cos(th), -math.sin(th)*math.cos(al),  math.sin(th)*math.sin(al), a*math.cos(th)],
        [ math.sin(th),  math.cos(th)*math.cos(al), -math.cos(th)*math.sin(al), a*math.sin(th)],
        [           0.0,             math.sin(al),             math.cos(al),            d],
        [           0.0,                      0.0,                      0.0,          1.0]
    ])

# 사용자 정의 링크 파라미터
d  = [0,   0,   0]
a  = [0, 140, 140]
alpha_ = [90, -180, 90]
T0 = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,105.5],[0,0,0,1]])

# 순방향 FK (엔드 이펙터 좌표 계산용)
def forward_kinematics(thetas):
    A1 = dh_transform(thetas[0], d[0], a[0], alpha_[0])
    A2 = dh_transform(thetas[1]+90, d[1], a[1], alpha_[1])
    A3 = dh_transform(thetas[2]+90, d[2], a[2], alpha_[2])
    # 평행 링크 보정
    A4 = dh_transform(-(thetas[1]+thetas[2]), 0, 0, 0)
    T = T0 @ A1 @ A2 @ A3 @ A4
    return T

# 수치적 IK (작업 공간 → 관절 공간)
def inverse_kinematics_numeric(target, initial=[0,0,0], lr=0.1, tol=0.1, max_iter=5000):
    theta = np.array(initial, dtype=float)
    for _ in range(max_iter):
        T = forward_kinematics(theta)
        pos = T[:3,3]
        error = target - pos
        if np.linalg.norm(error) < tol:
            return theta, True
        # 수치 미분
        grad = np.zeros(3)
        delta = 1e-3
        for j in range(3):
            temp = theta.copy()
            temp[j] += delta
            grad[j] = ((forward_kinematics(temp)[:3,3] - pos) @ error) / delta
        theta += lr * grad
    return theta, False

# 관절각(deg) → Dynamixel 티크 값
def deg2dxl_theta(thetas_deg):
    arr = np.array(thetas_deg)
    # 링크 보정: 세 번째 관절 각도 보정
    arr[2] = arr[2] - arr[1]
    ticks = (arr * (1023/300)).astype(int) + 512
    return ticks


# ————— 메인 실행부 —————
if __name__ == "__main__":
    # 1) 컨트롤러 초기화
    dxl = DXLController()
    for jid in JOINT_IDS:
        dxl.enable_torque(jid)
        dxl.motor_speed(jid, 200)   # 속도 조절 (0~1023)

    # 2) 원의 중심과 반지름 입력 (예: 150,0,40,50)
    cx, cy, cz, r = map(float, input("cx, cy, cz, r 입력: ").split(","))

    # 3) 원 궤적 좌표 생성 (예: 72점)
    point_num = 72
    angles = np.linspace(0, 2*math.pi, point_num, endpoint=False)

    # 4) 궤적 순차 실행
    current_guess = [0,0,0]
    dt = 0.05  # 50ms 간격
    for theta in angles:
        tx = cx + r * math.cos(theta)
        ty = cy + r * math.sin(theta)
        tz = cz

        # IK 계산
        current_guess, success = inverse_kinematics_numeric(np.array([tx, ty, tz]), current_guess)
        if not success:
            print("IK 수렴 실패, 건너뜀:", [tx, ty, tz])
            continue

        # 모터 티크 값 계산 및 동시 이동
        tick_vals = deg2dxl_theta(current_guess)
        dxl.set_goal_position(tick_vals)
        time.sleep(dt)

    # 5) 원 그리기 종료 후 홈(0,0,0) 복귀
    home_ticks = deg2dxl_theta([0,0,0])
    dxl.set_goal_position(home_ticks)
    time.sleep(1)

    dxl.close()
