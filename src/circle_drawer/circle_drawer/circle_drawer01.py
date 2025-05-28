import time
import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS
from ikpy.chain import Chain
from ikpy.link import OriginLink, DHLink

# ==== Parameters ====
JOINT_IDS       = [1, 2, 3, 4]

JOINT_ZERO_OFF  = [150.0, 150.0, 150.0, 150.0]
JOINT_DIR       = [1, 1, 1, 1]
JOINT_MIN       = [0.0, 0.0, 0.0, 0.0]
JOINT_MAX       = [300.0, 300.0, 300.0, 300.0]

DEVICENAME      = '/dev/ttyUSB0'
BAUDRATE        = 1000000
PROTOCOL_VERSION= 1.0

ADDR_TORQUE     = 24
ADDR_GOAL_POS   = 30
ADDR_PRESENT_POS= 36
TORQUE_ON       = 1
TORQUE_OFF      = 0

# robot geometry (meters)
d1, a2, a3, a4 = 0.135, 0.125, 0.175, 0.055

# ==== Dynamixel conversion ====
def deg2dxl(deg: float) -> int:
    deg = max(0.0, min(300.0, deg))
    return int(deg / 300.0 * 1023)

def dxl2deg(val: int) -> float:
    return val / 1023.0 * 300.0

# ==== Build IKPY chain ====##
chain = Chain(name='robot_arm', links=[
    OriginLink(),
    DHLink(d=d1,   a=0.0, alpha=np.deg2rad(90),  theta=0),
    # 숄더 링크: 실제 0° 위치와 DH 기준 프레임 차이를 +90°로 보정
    DHLink(d=0.0,  a=a2,  alpha=0.0,             theta=np.deg2rad(90)),
    DHLink(d=0.0,  a=a3,  alpha=0.0,             theta=0),
    DHLink(d=0.0,  a=a4,  alpha=0.0,             theta=0),
])

# ==== Controller class ====
class Controller:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        self.port = PortHandler(device)
        if not self.port.openPort(): raise IOError("Port open fail")
        if not self.port.setBaudRate(baud): raise IOError("Set baudrate fail")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POS, 2)

    def enable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE, TORQUE_ON)

    def disable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE, TORQUE_OFF)

    def set_positions(self, deg_list):
        self.sync_write.clearParam()
        for idx, j in enumerate(JOINT_IDS):
            pos = deg2dxl(deg_list[idx])
            param = [pos & 0xFF, (pos >> 8) & 0xFF]
            self.sync_write.addParam(j, bytes(param))
        res = self.sync_write.txPacket()
        if res != COMM_SUCCESS:
            raise RuntimeError("SyncWrite error")

    def get_position(self, j):
        val, comm, err = self.packet.read2ByteTxRx(self.port, j, ADDR_PRESENT_POS)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError(f"Dxl error ID{j}")
        return dxl2deg(val)

    def close(self):
        self.port.closePort()

# ==== Main Execution ====##
if __name__ == '__main__':
    ctrl = Controller()
    try:
        ctrl.enable_torque(JOINT_IDS)
        time.sleep(0.1)
        # Move to zero offsets
        ctrl.set_positions(JOINT_ZERO_OFF)
        time.sleep(2.0)

        # Read current joint config
        q0 = []
        for idx, j in enumerate(JOINT_IDS):
            deg = ctrl.get_position(j)
            rad = np.deg2rad(deg - JOINT_ZERO_OFF[idx]) * JOINT_DIR[idx]
            q0.append(rad)
        q0 = np.array(q0)

        # Input target
        vals = input("Input target (x y z roll_deg): ").split()
        x, y, z, roll_deg = map(float, vals)
        target_frame = [x, y, z]

        # IK computation
        ik_solution = chain.inverse_kinematics(
            target_frame + [1], initial_position=[0] + list(q0)
        )
        # ik_solution returns [base, q1, q2, q3, q4]
        q_goal = ik_solution[1:5]

        # Convert to degrees + zero offset
        deg_goal = [JOINT_ZERO_OFF[i] + np.rad2deg(q_goal[i]) * JOINT_DIR[i] for i in range(4)]

        # Send goal
        ctrl.set_positions(deg_goal)
        print("Goal positions sent:", deg_goal)

    finally:
        ctrl.disable_torque(JOINT_IDS)
        ctrl.close()
        print("Shutdown")
