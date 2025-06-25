import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

d1 = 112.25 + 45
a2, a3, a4 = 140.0, 140.0, 63.5

DIRECTION = [ 1, 1, 1, 1]

JOINT_IDS = [1, 2, 3, 4]
JOINT_IDS = np.array(JOINT_IDS).astype(int)

DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116  # 4 byte
ADDR_MOVING_SPEED = 112  # 4 byte
ADDR_PRESENT_POSITION = 132  # 4 byte
ADDR_MX_TORQUE_LIMIT = 38
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

Moving_Speed = 32
z_offset = 43

def deg2dxl(deg: float) -> int:
    ang = ((deg + 180) % 360) - 180
    ratio = (ang + 180) / 360
    raw = int(ratio * 4096)
    return max(0, min(4096, raw))

def dxl2deg(raw: int) -> float:
    return raw / 4096 * 360 - 180

robot_chain = Chain(name='4dof_dxl', links=[
    OriginLink(),
    DHLink(d=d1, a=0, alpha=np.deg2rad(90), theta=np.deg2rad(54.4)),

    DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(90)),

    DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(-90)),

    DHLink(d=0, a=a4, alpha=0, theta=0),

])

class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"Failed to open port {device}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Failed to set baudrate {baud}")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 4)
        self.sync_speed = GroupSyncWrite(self.port, self.packet,
                                         ADDR_MOVING_SPEED, 4)

    def enable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def enable_torque_single(self, id):
        self.packet.write1ByteTxRx(self.port, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_positions(self, deg_list):
        self.sync_write.clearParam()
        for idx, j in enumerate(JOINT_IDS):
            angle = deg_list[idx] * DIRECTION[idx]
            raw = deg2dxl(angle)
            # 4 바이트 little endian
            param = [
                raw & 0xFF,
                (raw >> 8) & 0xFF,
                (raw >> 16) & 0xFF,
                (raw >> 24) & 0xFF,
            ]   
            self.sync_write.addParam(j, bytes(param))
        self.sync_write.txPacket()
        if self.sync_write.txPacket() != COMM_SUCCESS:
            raise RuntimeError("SyncWrite error")

    def get_position(self, j):
        val, comm, err = self.packet.read2ByteTxRx(self.port, j, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError(f"Dxl error ID{j}")
        return dxl2deg(val)

    def close(self):
        self.port.closePort()

    def motor_speed(self, dxl_id, speed_val):
        self.packet.write2ByteTxRx(self.port, dxl_id, Moving_Speed, speed_val)

    def set_speed(self, speed):
        self.sync_speed.clearParam()
        for j in JOINT_IDS:
            # 4바이트 little endian으로 분할
            param = [
                speed        & 0xFF,
                (speed >> 8) & 0xFF,
                (speed >> 16)& 0xFF,
                (speed >> 24)& 0xFF,
            ]
            if not self.sync_speed.addParam(j, bytes(param)):
                raise RuntimeError(f"Speed addParam 실패 ID={j}")
        if self.sync_speed.txPacket() != COMM_SUCCESS:
            raise RuntimeError("Speed SyncWrite error")
        self.sync_speed.clearParam()

    def move_joint(self, joint_id: int, position_deg: float, speed: int = None):
        raw_pos = deg2dxl(position_deg)
        self.packet.write4ByteTxRx(self.port, joint_id, ADDR_GOAL_POSITION, raw_pos)
    
def move_to(x, y, z, speed):
    current_deg = [dxl.get_position(j) for j in JOINT_IDS]
    current_rad = [0.0] + [np.deg2rad(d) for d in current_deg]
    T_current = robot_chain.forward_kinematics(current_rad)
    p_current = T_current[:3, 3]

    
    z = z + z_offset

    ik_full = robot_chain.inverse_kinematics(
        target_position=[x, y, z],
        orientation_mode='Y',
        target_orientation=[0, 0, 1]  # Z축과 정렬
    )
    target_rad = ik_full[1:len(JOINT_IDS) + 1]
    target_deg = [np.rad2deg(ang) for ang in target_rad]

    dxl.set_speed(speed)
    dxl.set_positions(target_deg)

    dist=np.linalg.norm(np.array([x, y, z]) - p_current)
    # time.sleep(dist/speed)

# def circle_drawer(x, y, z, r, steps=100, speed=50):
#     arc_len = 2 * np.pi * r / steps
#     dt = arc_len / speed

#     z = z + z_offset

#     for i in range(steps):
#         theta = 2.1 * np.pi * i / steps
#         px = x + r * np.cos(theta)
#         py = y + r * np.sin(theta)
#         pz = z
        
#         ik_results = robot_chain.inverse_kinematics(
#             target_position=[px, py, pz],
#             orientation_mode='Y',
#             target_orientation=[0, 0, 1]  # Z축과 정렬
#         )

#         degs = [np.rad2deg(ang) for ang in ik_results[1:len(JOINT_IDS) + 1]]

#         dxl.set_positions(degs)
#         time.sleep(dt)

#     px = x + r * np.cos(0)
#     py = y + r * np.sin(0)
#     pz = z

#     ik_results = robot_chain.inverse_kinematics([px, py, pz])
#     degs = [np.rad2deg(ang) for ang in ik_results[1:len(JOINT_IDS) + 1]]
#     dxl.set_positions(degs)

# def circle_drawer(x, y, z, r, steps=200, speed=50): #2번
#     arc_len = 4 * np.pi * r / steps   # 2바퀴 (4π)
#     dt = arc_len / speed

#     z = z + z_offset

#     for i in range(steps):
#         theta = -2 * np.pi + 4 * np.pi * i / steps  
#         px = x + r * np.cos(theta)
#         py = y + r * np.sin(theta)
#         pz = z

#         ik_results = robot_chain.inverse_kinematics(
#             target_position=[px, py, pz],
#             orientation_mode='Y',
#             target_orientation=[0, 0, 1]
#         )

#         degs = [np.rad2deg(ang) for ang in ik_results[1:len(JOINT_IDS) + 1]]
#         dxl.set_positions(degs)
#         time.sleep(dt)
   
#     px = x + r * np.cos(0)
#     py = y + r * np.sin(0)
#     pz = z

#     ik_results = robot_chain.inverse_kinematics([px, py, pz])
#     degs = [np.rad2deg(ang) for ang in ik_results[1:len(JOINT_IDS) + 1]]
#     dxl.set_positions(degs)

measured_radii = {
    10: {
        0.0:           7.5,
        np.pi/4:       7.0,
        np.pi/2:       8,
        3*np.pi/4:     10,
        np.pi:         12.5,
        5*np.pi/4:     12,
        3*np.pi/2:     10,
        7*np.pi/4:     9.9
    },
    20: {
        0.0:          17.5,
        np.pi/4:      16,
        np.pi/2:      15,
        3*np.pi/4:    20,
        np.pi:        22.5,
        5*np.pi/4:    24,
        3*np.pi/2:    20.0,
        7*np.pi/4:    19.9
    },
    25: {
        0.0:          22,
        np.pi/4:      21,
        np.pi/2:      20,
        3*np.pi/4:    22,
        np.pi:        26,
        5*np.pi/4:    25,
        3*np.pi/2:    25.0,
        7*np.pi/4:    24
    },
    30: {
        0.0:          27,
        np.pi/4:      27,
        np.pi/2:      26,
        3*np.pi/4:    30,
        np.pi:        30,
        5*np.pi/4:    32,
        3*np.pi/2:    30.0,
        7*np.pi/4:    30
    },
    40: {
        0.0:          37.0,
        np.pi/4:      36.0,
        np.pi/2:      35.0,
        3*np.pi/4:    40.0,
        np.pi:        39,
        5*np.pi/4:    42,
        3*np.pi/2:    40.5,
        7*np.pi/4:    40
    },
    50: {
        0.0:          47.0,
        np.pi/4:      45.0,
        np.pi/2:      45.0,
        3*np.pi/4:    48.0,
        np.pi:        47,
        5*np.pi/4:    50,
        3*np.pi/2:    50,
        7*np.pi/4:    50
    },
    60: {
        0.0:          55.0,
        np.pi/4:      54.0,
        np.pi/2:      55.0,
        3*np.pi/4:    59.0,
        np.pi:        58.5,
        5*np.pi/4:    60,
        3*np.pi/2:    60,
        7*np.pi/4:    60
    }
}

def circle_drawer(x, y, z, r, steps=200, speed=25):

    if r == 10:
        speed = 25
        z=5
    
    z_cmd = z + z_offset
    arc_len = 2 * np.pi * r / steps
    dt = arc_len / speed
    start_x, start_y, safe_z = x+r, y, z+20
    move_to(start_x, start_y, safe_z, speed)
    time.sleep(1.0)

    # move_to(start_x, start_y, z, speed)
    # time.sleep(0.5)

    

    # 보정 데이터가 있으면, 4방향 wrap 보간 테이블 준비
    if r in measured_radii:
        dir_dict = measured_radii[r]
        thetas = np.array(sorted(dir_dict.keys()))
        radii = np.array([dir_dict[t] for t in thetas])
        thetas_ext = np.concatenate([thetas, [thetas[0] + 2*np.pi]])
        radii_ext = np.concatenate([radii, [radii[0]]])

        def get_scale(theta):
            th = theta % (2*np.pi)
            r_meas = np.interp(th, thetas_ext, radii_ext)
            return r / r_meas
    else:
        # 보정 불가: scale = 1
        def get_scale(theta):
            return 1.0
    
    #ㅇ곳에 보정된 시작점 위치ㅇㅔ서 위에서 살작 떨어져 있ㄴㅡㄴ 곳으로 움직이고 싶어.
    s0 = get_scale(0.0)
    start_x = x + r * s0
    start_y = y
    # z축 안전고도
    move_to(start_x, start_y, safe_z, speed)
    time.sleep(1.0)

    # 궤적 실행
    for i in range(steps + 1):
        theta = 2.3 * np.pi * i / steps
        s = get_scale(theta)

        dx = r * np.cos(theta) * s
        dy = r * np.sin(theta) * s
        px, py = x + dx, y + dy

        ik = robot_chain.inverse_kinematics(
            target_position=[px, py, z_cmd],
            orientation_mode='Y',
            target_orientation=[0, 0, 1]
        )
        degs = [np.rad2deg(a) for a in ik[1:len(JOINT_IDS)+1]]
        dxl.set_positions(degs)
        time.sleep(dt)


if __name__ == "__main__":

    dxl = DXLController()
    dxl.enable_torque(JOINT_IDS)

    # 초기 위치 설정
    initial_positions = [-54.4, 0, 0, 0]
    dxl.set_positions(initial_positions)
    time.sleep(1)

    target_str = input("반지름 확인 x만: ")
    x = float(target_str.strip().split(",")[0])  # 첫 번째 값만 사용
    target_pos = np.array([230, 0, 10.0])
    print(target_pos)
    move_to(*target_pos,30)
    time.sleep(1)

    print("\n 목표 위치와 반지름을 입력 (X,Y,Z,R 단위 : mm)")
    target_str = input("목표 위치: ")
    x, y, z, r = map(float, target_str.strip().split(","))

    print("\n 2nd 목표 위치와 반지름을 입력 (X,Y,Z,R 단위 : mm)")
    target_str2 = input("목표 위치: ")
    x2, y2, z2, r2 = map(float, target_str2.strip().split(","))

    dxl.move_joint(2,300)
    time.sleep(1)
    # move_to(x+2*r, y, 20, speed=50)
    # time.sleep(1)
    # target_pos = np.array([x+r, y, 20])
    # move_to(*target_pos,30)
    # time.sleep(1)
    move_to(x+r, y, z+40, speed=25)
    time.sleep(2)
    circle_drawer(x, y, z, r,  steps=200, speed=50)
    time.sleep(1)

    dxl.move_joint(2,300)
    time.sleep(1)

    initial_positions = [-54.4, 0, 0, 0]
    dxl.set_positions(initial_positions)
    time.sleep(3)

    dxl.move_joint(4,-300)
    time.sleep(2)

    dxl.move_joint(2,300)
    time.sleep(2)

    

    circle_drawer(x2, y2, z2, r2,  steps=200, speed=50)
    time.sleep(1)

    dxl.move_joint(2,300)
    time.sleep(1)

    initial_positions = [-54.4, 0, 0, 0]
    dxl.set_positions(initial_positions)
    time.sleep(1)

    # dxl.disable_torque(JOINT_IDS)
    dxl.close()
