import time
import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Control table address (Protocol 2.0)
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4
BAUDRATE                = 1000000
DEVICENAME              = '/dev/ttyUSB0'
PROTOCOL_VERSION        = 2.0
TORQUE_ENABLE           = 1
TORQUE_DISABLE          = 0

# Joint IDs
JOINT_IDS = [1, 2, 3, 4]

# Convert radians to Dynamixel position units
def rad_to_dxl(rad):
    # 0 ~ 2*pi maps to 0 ~ 4095
    pos = int((rad % (2*np.pi)) * 4096.0 / (2*np.pi))
    return pos

# Convert Dynamixel position units to radians
def dxl_to_rad(val):
    return val * (2*np.pi) / 4096.0

class DXLController:
    def __init__(self, device=DEVICENAME, baud=BAUDRATE):
        # Port & packet
        self.port = PortHandler(device)
        if not self.port.openPort():
            raise IOError(f"Failed to open port {device}")
        if not self.port.setBaudRate(baud):
            raise IOError(f"Failed to set baudrate {baud}")
        self.packet = PacketHandler(PROTOCOL_VERSION)

    def enable_torque(self, ids):
        for dxl_id in ids:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, ids):
        for dxl_id in ids:
            self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def set_goal_position(self, dxl_id, rad):
        pos = rad_to_dxl(rad)
        self.packet.write4ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, pos)

    def get_present_position(self, dxl_id):
        val, _, _ = self.packet.read4ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return dxl_to_rad(val)

    def close(self):
        self.port.closePort()

# robot hardware parameter
L1, L2, L3, L4 = 1, 1, 1, 1


def DH_matrix(theta, d, a, alpha):
    return np.array(
        [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
         [0, np.sin(alpha), np.cos(alpha), d],
         [0, 0, 0, 1]]
    )

def get_robot_pos(matrix):
    return np.array(matrix[0:2, 3])

def get_robot_orientation(matrix):
    return np.array(matrix[0:2, 0:2])

if __name__ == '__main__':
    controller = DXLController()
    try:
        # Enable torque on all joints
        controller.enable_torque(JOINT_IDS)
        time.sleep(0.1)

        # Move to zero position
        print("Homing joints...")
        for j in JOINT_IDS:
            controller.set_goal_position(j, 0.0)
        time.sleep(2.0)

        # Read and print positions
        print("Present positions:")
        for j in JOINT_IDS:
            pos = controller.get_present_position(j)
            print(f" Joint {j}: {pos:.3f} rad")

        # Sweep test: move joints back and forth
        for angle in [0.0, np.pi/4, -np.pi/4, 0.0]:
            print(f"Moving to {angle:.2f} rad")
            for j in JOINT_IDS:
                controller.set_goal_position(j, angle)
            time.sleep(1.0)

    finally:
        # Disable torque and close port
        controller.disable_torque(JOINT_IDS)
        controller.close()
        print("Controller shutdown.")


