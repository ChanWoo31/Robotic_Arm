import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

d1 = 110.0
a2, a3, a4 = 140.0, 140.0, 80.0

JOINT_IDS = [1, 2, 3, 4]
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

def deg2dxl(deg: float) -> int:
    ang = ((deg + 180) % 360) - 180
    ratio = (ang + 180) / 360
    raw = int(ratio * 4095)
    return max(0, min(4095, raw))

def dxl2deg(raw: int) -> float:
    return raw / 4095 * 360 - 180

robot_chain = Chain(name='4dof_dxl', links=[
    OriginLink(),
    DHLink(d=d1, a=0, alpha=np.deg2rad(90), offset=0),

    DHLink(d=0, a=a2, alpha=0, offset=np.deg2rad(90)),

    DHLink(d=0, a=a3, alpha=0, offset=0),

    DHLink(d=0, a=a4, alpha=0, offset=0),

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

    def enable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


    def set_positions(self, deg_list):
        self.sync_write.clearParam()
        for idx, j in enumerate(JOINT_IDS):
            raw = deg2dxl(deg_list[idx])
            # 4 바이트 little endian
            param = [
                raw & 0xFF,
                (raw >> 8) & 0xFF,
                (raw >> 16) & 0xFF,
                (raw >> 24) & 0xFF,
            ]
            self.sync_write.addParam(j, bytes(param))
        if self.sync_write.txPacket() != COMM_SUCCESS:
            raise RuntimeError("SyncWrite error")

    def get_position(self, j):
        val, comm, err = self.packet.read2ByteTxRx(self.port, j, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError(f"Dxl error ID{j}")
        return dxl2deg(val)

    def close(self):
        self.port.closePort()

def move_to(x, y, z, speed=50):
    current_deg = [dxl.get_position(j) for j in JOINT_IDS]
    current_rad = [0.0] + [np.deg2rad(d) for d in current_deg]
    T_current = robot_chain.forward_kinematics(current_rad)
    p_current = T_current[:3, 3]

    ik_full = robot_chain.inverse_kinematics([x, y, z, 1])
    target_rad = ik_full[1:len(JOINT_IDS) + 1]
    target_deg = [np.rad2deg(ang) for ang in target_rad]

    dxl.set_positions(target_deg)

    dist=np.linalg.norm(np.array([x, y, z]) - p_current)
    time.sleep(dist/speed)

def circle_drawer(x, y, z, r, steps=100, speed=50):
    arc_len = 2 * np.pi * r / steps
    dt = arc_len / speed

    for i in range(steps):
        theta = 2 * np.pi * i / steps
        px = x + r * np.cos(theta)
        py = y + r * np.sin(theta)
        pz = z
        
        ik_results = robot_chain.inverse_kinematics([px, py, pz, 1])

        degs = [np.rad2deg(ang) for ang in ik_results[1:len(JOINT_IDS) + 1]]

        dxl.set_positions(degs)
        time.sleep(dt)

    px = x + r * np.cos(0)
    py = y + r * np.sin(0)
    pz = z

    ik_results = robot_chain.inverse_kinematics([px, py, pz])
    degs = [np.rad2deg(ang) for ang in ik_results[1:len(JOINT_IDS) + 1]]
    dxl.set_positions(degs)


if __name__ == "__main__":
    dxl = DXLController()
    dxl.enable_torque(JOINT_IDS)

    # 초기 위치 설정
    initial_positions = [0, 0, 0, 0]
    dxl.set_positions(initial_positions)
    time.sleep(1)

    print("\n 목표 위치와 반지름을 입력 (X,Y,Z,R 단위 : mm)")
    target_str = input("목표 위치: ")
    x, y, z, r = map(float, target_str.strip().split(","))

    target_pos = np.array([x, y, z])

    move_to(x, y, z, speed=50)

    

    dxl.disable_torque(JOINT_IDS)
    dxl.close()