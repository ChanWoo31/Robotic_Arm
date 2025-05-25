import time
import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

DEVICENAME        = '/dev/ttyUSB0'
BAUDRATE          = 1000000
PROTOCOL_VERSION  = 1


ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30        # 2 byte
ADDR_MOVING_SPEED     = 32        # 2 byte
ADDR_PRESENT_POSITION = 36        # 2 byte
ADDR_PRESENT_SPEED    = 38        # 2 byte

TORQUE_ENABLE   = 1
TORQUE_DISABLE  = 0

# Joint IDs
JOINT_IDS = [1, 2, 3, 4]

# Convert radians to Dynamixel position units
def deg_to_dxl(deg: float) -> int:
    deg = max(0.0, min(300.0, deg))
    return int(deg / 300.0 * 1023)

def dxl_to_deg(val: int) -> float:
    return val / 1023.0 * 300.0

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

    def set_goal_position(self, dxl_id, deg):
        pos = deg_to_dxl(deg)
        self.packet.write2ByteTxRx(self.port, dxl_id, ADDR_GOAL_POSITION, pos)

    def get_present_position(self, dxl_id):
        val, _, _ = self.packet.read2ByteTxRx(self.port, dxl_id, ADDR_PRESENT_POSITION)
        return dxl_to_deg(val)

    def close(self):
        self.port.closePort()

# robot hardware parameter(m)
d1, a2, a3, a4 = 0.135, 0.125, 0.175, 0.055


def DH_matrix(theta, d, a, alpha):
    return np.array(
        [[np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
         [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
         [0, np.sin(alpha), np.cos(alpha), d],
         [0, 0, 0, 1]]
    )

def forward_kinematics(q):
    theta1, theta2, theta3, theta4 = q
    A1 = DH_matrix(theta1, d1, 0, 0)
    A2 = DH_matrix(theta2, 0, a2, 0)
    A3 = DH_matrix(theta3, 0, a3, 0)
    A4 = DH_matrix(theta4, 0, a4, 0)
    T04 = A1 @ A2 @ A3 @ A4
    pos = T04[0:3, 3]
    roll = theta2 + theta3 + theta4
    return pos, roll

def cubic_coeff(theta_i, theta_f, dtheta_i = 0.0, dtheta_f = 0.0, T = 2.0):
    #3rd 
    a0 = theta_i
    a1 = dtheta_i
    a2 = (3 * (theta_f - theta_i) - (2 * dtheta_i + dtheta_f) * T) / (T**2)
    a3 = (2 * (theta_i - theta_f) + (dtheta_i + dtheta_f) * T) / (T**3)
    return a0, a1, a2, a3

def eval_cubic(coeffs, t):
    a0, a1, a2, a3 = coeffs
    return a0 + a1 * t + a2 * t**2 + a3 * t**3

def inverse_kinematics(x, y, z, roll):
    theta1 = np.arctan2(y, x)
    x_w = x - a4 * np.cos(roll) * np.cos(theta1)
    y_w = y - a4 * np.cos(roll) * np.sin(theta1)
    z_w = z - a4 * np.sin(roll)
    r = np.hypot(x_w, y_w)
    s = z_w - d1
    D = (r**2 + s**2 - a2**2 - a3**2) / (2 * a2 * a3)
    D = np.clip(D, -1.0, 1.0)
    theta3 = np.arctan2(np.sqrt(1 - D**2), D)
    theta2 = np.arctan2(s, r) - np.arctan2(a3 * np.sin(theta3), a2 + a3 * np.cos(theta3))
    theta4 = roll - theta2 - theta3
    return np.array([theta1, theta2, theta3, theta4])

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
        for j in JOINT_IDS:
            controller.set_goal_position(j, 0.0)
        time.sleep(2.0)

        # IK and FK
        x_d, y_d, z_d, roll_d = input("Enter desired position (x, y, z) and roll angle: ").split()
        q_goal = inverse_kinematics(float(x_d), float(y_d), float(z_d), float(roll_d))

        # coeff
        T = 4.0
        dt = 0.02
        t_steps = np.arange(0, T + dt/2, dt)
        coeffs = [cubic_coeff(0, float(np.rad2deg(qg)), 0,0,T) for qg in q_goal]

        # trajectory generation
        for t in t_steps:
            degs = [eval_cubic(c, t) for c in coeffs]
            for idx, j in enumerate(JOINT_IDS):
                controller.set_goal_position(j, degs[idx])
                time.sleep(dt)

            time.sleep(0.5)

    finally:
        # Disable torque and close port
        controller.disable_torque(JOINT_IDS)
        controller.close()
        print("Controller shutdown.")


