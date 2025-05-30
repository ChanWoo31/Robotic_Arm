from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain
import numpy as np
import sys
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, COMM_SUCCESS

# 변환
def deg2dxl(deg: float) -> int:
    deg = max(0.0, min(300.0, deg))
    return int(deg / 300.0 * 1023)

def dxl2deg(val: int) -> float:
    return val / 1023.0 * 300.0

def rad2dxl(rad):
    deg = np.rad2deg(rad)
    deg = np.clip(deg, 0, 300)
    return deg2dxl(deg)

def read_input():
    x_target, y_target, z_target = sys.stdin.readline("x, y, z : ").strip().split(',')
    x_target = float(x_target.strip())
    y_target = float(y_target.strip())
    z_target = float(z_target.strip())
    return x_target, y_target, z_target

# parameters
d1, a2, a3 = 0.135, 0.14, 0.14

# offset(degrees)
offset_degree = [150.0, 150.0, 150.0]

# joint direction
joint_directions = [1, 1, -1]

JOINT_IDS = [1, 2, 3]
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
ADDR_PRESENT_POSITION = 36
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

robot_chain = Chain(name='3dof_dxl', links = [
    OriginLink(),
    #link 1
    DHLink(d=d1, a=0.0, alpha=np.deg2rad(90), offset=np.deg2rad(offset_degree[0])),
    #link2: theta2 + 90 + offset2
    DHLink(d=0.0, a=a2, alpha = np.deg2rad(180), offset=np.deg2rad(offset_degree[1])),
    #link3: theta3 + 90 + offset3
    DHLink(d=0.0, a=a3, alpha=np.deg2rad(90), offset=np.deg2rad(offset_degree[2])),
           
])

# ==== Controller class ====
class Controller:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        self.port = PortHandler(device)
        if not self.port.openPort(): raise IOError("Port open fail")
        if not self.port.setBaudRate(baud): raise IOError("Set baudrate fail")
        self.packet = PacketHandler(PROTOCOL_VERSION)
        self.sync_write = GroupSyncWrite(self.port, self.packet, ADDR_GOAL_POSITION, 2)

    def enable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

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
        val, comm, err = self.packet.read2ByteTxRx(self.port, j, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError(f"Dxl error ID{j}")
        return dxl2deg(val)

    def close(self):
        self.port.closePort()

if __name__ == "__main__":
    control = Controller(DEVICENAME, BAUDRATE)
    control.enable_torque(JOINT_IDS)

    x_target, y_target, z_target = read_input()

    ik_solution = robot_chain.inverse_kinematics(
    target_position = [x_target, y_target, z_target],
    initial_position = [0, 0, 0],
    
    )
    joint_thetas = ik_solution[1:]

    motor_degs = []
    for i, theta in enumerate(joint_thetas):
        motor_deg = rad2dxl(theta) * joint_directions[i]
        motor_degs.append(motor_deg + offset_degree[i])
    
    print(" IK 결과 (deg): ", motor_degs)

    control.set_positions(motor_degs)


