import numpy as np
import time
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS

# ————— 설정 —————
DEVICENAME   = '/dev/ttyUSB0'   # Dynamixel USB 어댑터 포트
BAUDRATE     = 1_000_000        # 1 Mbps
PROTOCOL_VER = 1.0

DXL_IDS      = [1, 2, 3]        # [베이스(yaw), 어깨(pitch), 팔꿈치(pitch)]
ADDR_TORQUE  = 24               # Torque 제어 레지스터
ADDR_GOAL_POS= 30               # Goal Position 레지스터
TORQUE_ON    = 1

# — Robot 기하학 (m, ° 단위)
L1       = 0.140  # 링크1 길이 (어깨→팔꿈치)
L2       = 0.140  # 링크2 길이 (팔꿈치→툴센터)
Z_OFFSET = 0.080  # 베이스에서 어깨 축 높이

# ————— 초기화 함수 —————
def init_dxl():
    port = PortHandler(DEVICENAME)
    packet = PacketHandler(PROTOCOL_VER)
    if not port.openPort():    raise IOError("포트 열기 실패")
    if not port.setBaudRate(BAUDRATE): raise IOError("Baudrate 설정 실패")
    for dxl_id in DXL_IDS:
        err, _ = packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE, TORQUE_ON)
        if err != COMM_SUCCESS:
            raise RuntimeError(f"ID {dxl_id} 토크 활성화 실패: {packet.getTxRxResult(err)}")
    return port, packet

# ————— 각도 → raw 변환 —————
def deg2raw(deg):
    """0°~300° → 0~1023"""
    d = np.clip(deg, 0.0, 300.0)
    return int(d / 300.0 * 1023)

# ————— 역기구학 —————
def ikine(x, y, z):
    """
    입력: x, y, z (m)
    출력: θ1(base yaw), θ2(shoulder pitch), θ3(elbow pitch) in degrees
    """
    theta1 = np.rad2deg(np.arctan2(y, x))
    r      = np.hypot(x, y)
    s      = z - Z_OFFSET

    # Law of Cosines for θ3
    D      = (r**2 + s**2 - L1**2 - L2**2) / (2 * L1 * L2)
    D      = np.clip(D, -1.0, 1.0)
    theta3 = np.rad2deg(np.arccos(D))

    # Shoulder θ2
    alpha  = np.arctan2(s, r)
    beta   = np.arctan2(L2 * np.sin(np.deg2rad(theta3)),
                        L1 + L2 * np.cos(np.deg2rad(theta3)))
    theta2 = np.rad2deg(alpha - beta)

    return theta1, theta2, theta3

# ————— 동기 제어 준비 —————
def make_group(port, packet):
    return GroupSyncWrite(port, packet, ADDR_GOAL_POS, 2)

def sync_write(group, angles_deg):
    """
    angles_deg: iterable of [θ1, θ2, θ3] in degrees
    """
    group.clearParam()
    raws = [deg2raw(a) for a in angles_deg]
    for dxl_id, raw in zip(DXL_IDS, raws):
        lo, hi = raw & 0xFF, (raw >> 8) & 0xFF
        group.addParam(dxl_id, bytearray([lo, hi]))
    group.txPacket()

# ————— 원 궤적 그리기 —————
def draw_circle(group, cx, cy, cz, radius, steps=100, delay=0.05):
    """
    cx, cy, cz, radius in meters
    steps: 분할 개수
    delay: 각 스텝 간 대기 시간 (s)
    """
    for deg in np.linspace(0, 360, steps+1):
        x = cx + radius * np.cos(np.deg2rad(deg))
        y = cy + radius * np.sin(np.deg2rad(deg))
        θ1, θ2, θ3 = ikine(x, y, cz)
        sync_write(group, (θ1, θ2, θ3))
        time.sleep(delay)

# ————— 메인 실행부 —————
if __name__ == "__main__":
    port, packet = init_dxl()
    group = make_group(port, packet)

    try:
        # 홈 자세: 완전 펼침 (0°, 0°, 0°)
        sync_write(group, ikine(L1 + L2, 0, Z_OFFSET))
        time.sleep(1)

        # 예시: 중심(0.10, 0.00, Z_OFFSET), 반지름 0.05 m 원 그리기
        draw_circle(group, cx=0.10, cy=0.00, cz=Z_OFFSET, radius=0.05,
                    steps=120, delay=0.03)
    finally:
        port.closePort()
