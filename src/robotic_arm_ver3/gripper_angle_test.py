import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

d1 = 162.25
a2, a3, a4 = 140.0, 140.0, 103.5

DIRECTION = [ 1, 1, 1, 1]

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
ADDR_OPERATING_MODE = 11
ADDR_GOAL_CURRENT = 102

GRIPPER_ID = 5
GRIPPER_OPEN_POSITION = 10
GRIPPER_BOX_GRIP_POSITION = -40
GRIPPER_SEORAP_GRIP_POSITION = -75

#drawer
drawer_height = 43

#support
support_height = 70

#box size
box_half_size = 12.5 
box_full_size = 25

def deg2dxl(deg: float) -> int:
    ang = ((deg + 180) % 360) - 180
    ratio = (ang + 180) / 360
    raw = int(ratio * 4096)
    return max(0, min(4096, raw))

def dxl2deg(raw: int) -> float:
    return raw / 4096 * 360 - 180

robot_chain = Chain(name='4dof_dxl', links=[
    OriginLink(),
    DHLink(d=d1, a=0, alpha=np.deg2rad(90), theta=0),

    DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(90)),

    DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(-90)),

    DHLink(d=0, a=a4, alpha=0, theta=np.deg2rad(90)),

])

robot_chain_90 = Chain(name='4dof_dxl_90', links=[
    OriginLink(),
    DHLink(d=d1, a=0, alpha=np.deg2rad(90), theta=0),

    DHLink(d=0, a=a2, alpha=0, theta=np.deg2rad(90)),

    DHLink(d=0, a=a3, alpha=0, theta=np.deg2rad(-90)),

    DHLink(d=0, a=a4, alpha=0, theta=np.deg2rad(0)),

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
        self.sync_current = GroupSyncWrite(self.port,
                                      self.packet,
                                      102,
                                      4)
    
    def enable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def disable_torque(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

    def enable_operating_mode(self, ids):
        for j in ids:
            self.packet.write1ByteTxRx(self.port, j, ADDR_OPERATING_MODE, 3)
    
    def enable_gripper_operating_mode(self, GRIPPER_ID):
        self.packet.write1ByteTxRx(self.port, GRIPPER_ID, ADDR_OPERATING_MODE, 5)

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
        if self.sync_write.txPacket() != COMM_SUCCESS:
            raise RuntimeError("SyncWrite error")

    def get_position(self, j):
        val, comm, err = self.packet.read2ByteTxRx(self.port, j, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS or err != 0:
            raise RuntimeError(f"Dxl error ID{j}")
        return dxl2deg(val)
    
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

    

    def set_gripper_current(self, current_mA: int):
        lo = current_mA & 0xFF
        hi = (current_mA >> 8) & 0xFF
        self.packet.write2ByteTxRx(
            self.port,
            GRIPPER_ID,
            ADDR_GOAL_CURRENT,
            current_mA
        )

    def grip_with_current(self, position_deg: float, current_mA: int, speed: int=20):
        self.packet.write1ByteTxRx(self.port,
                                   GRIPPER_ID,
                                   ADDR_OPERATING_MODE,
                                   5)

        self.set_gripper_current(current_mA)

        self.packet.write4ByteTxRx(self.port,
                                   GRIPPER_ID,
                                   ADDR_MOVING_SPEED,
                                   speed)

        raw_pos = deg2dxl(position_deg)
        self.packet.write4ByteTxRx(self.port,
                                   GRIPPER_ID,
                                   ADDR_GOAL_POSITION,
                                   raw_pos)



    def close(self):
        self.port.closePort()


def move_to(x, y, z, speed=50):
    current_deg = [dxl.get_position(j) for j in JOINT_IDS]
    current_rad = [0.0] + [np.deg2rad(d) for d in current_deg]
    T_current = robot_chain.forward_kinematics(current_rad)
    p_current = T_current[:3, 3]

    ik_full = robot_chain.inverse_kinematics(
        target_position=[x, y, z],
        orientation_mode='Y',
        target_orientation=[0, 0, 1]  # Z축과 정렬
    )
    target_rad = ik_full[1:len(JOINT_IDS) + 1]
    target_deg = [np.rad2deg(ang) for ang in target_rad]

    dxl.set_positions(target_deg)

    dist=np.linalg.norm(np.array([x, y, z]) - p_current)
    time.sleep(dist/speed)

def move_to_90(x, y, z, speed=50):
    current_deg = [dxl.get_position(j) for j in JOINT_IDS]
    current_rad = [0.0] + [np.deg2rad(d) for d in current_deg]
    T_current = robot_chain_90.forward_kinematics(current_rad)
    p_current = T_current[:3, 3]

    ik_full = robot_chain_90.inverse_kinematics(
        target_position=[x, y, z],
        orientation_mode='Y',
        target_orientation=[0, 0, 1]  # Z축과 정렬
    )
    target_rad = ik_full[1:len(JOINT_IDS) + 1]
    target_deg = [np.rad2deg(ang) for ang in target_rad]

    dxl.set_positions(target_deg)

    dist=np.linalg.norm(np.array([x, y, z]) - p_current)
    time.sleep(dist/speed)


def generate_linear_path(start, end, num_points=10):
    return np.linspace(start, end, num_points)

def move_along_path(path, speed=50):
    for pt in path:
        move_to(*pt, speed=speed)
        time.sleep(0.5)

def move_along_path_90(path, speed=50):
    for pt in path:
        move_to_90(*pt, speed=speed)
        time.sleep(0.5)

# def Read_my_position():
#     DXL_pos = dxl.get_position(JOINT_IDS)
#     DXL_pos = np.array(DXL_pos)
#     end_effector_frame = robot_chain.forward_kinematics(DXL_pos)

#     position = end_effector_frame[:3, 3]
#     print("엔드이펙스 위치 :", position)


if __name__ == "__main__":
    dxl = DXLController()
    dxl.enable_torque(JOINT_IDS)
    dxl.enable_torque([GRIPPER_ID])

    dxl.enable_operating_mode(JOINT_IDS)
    dxl.enable_gripper_operating_mode(GRIPPER_ID)

    # 초기 위치 설정
    dxl.set_speed(25)
    initial_positions = [0, 0, 0, 0]
    dxl.set_positions(initial_positions)
    time.sleep(1)
    
    # grip init
    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)

    print("\n 목표 L1, L2, ang1, ang2 입력 (L1, L2, ang1, ang2 단위 : mm)")

    target_str = input("목표 위치: ")
    tar_L1, tar_L2, ang1, ang2 = map(float, target_str.strip().split(","))
    L1 = tar_L1 + 103.5
    L2 = tar_L2 + 103.5
    offset = 103.5

    # support
    x1, y1, z1 = L1*np.cos(np.deg2rad(ang1)), L1 * np.sin(np.deg2rad(ang1)), support_height
    # drawer
    x2, y2, z2 = L2*np.cos(np.deg2rad(ang2)), L2 * np.sin(np.deg2rad(ang2)), drawer_height

    # 서랍 size = 130mm
    # 서랍 열고 닫기
    drawer_close = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)),(L2 - 30) * np.sin(np.deg2rad(ang2)), 25+offset])
    drawer_close_high = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)),(L2 - 30) * np.sin(np.deg2rad(ang2)), 25 + 4*box_full_size+offset])
    drawer_close_high_high = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)),(L2 - 30) * np.sin(np.deg2rad(ang2)), 25 + 6*box_full_size+offset])
    drawer_open = np.array([(L2 - 100) * np.cos(np.deg2rad(ang2)),(L2 - 100) * np.sin(np.deg2rad(ang2)), 25+offset])
    drawer_open_high = np.array([(L2 - 100) * np.cos(np.deg2rad(ang2)),(L2 - 100) * np.sin(np.deg2rad(ang2)), 25 + 6*box_full_size+offset])
    
    opening_drawer = generate_linear_path(drawer_close, drawer_open, num_points=10)

    # block 위치
    box_1 = np.array([(L2 + 2 * box_full_size + 0.5 * box_half_size) * np.cos(np.deg2rad(ang2)), (L2 + 2 * box_full_size + 0.5 * box_half_size) * np.sin(np.deg2rad(ang2)), 130+offset])
    box_1_high_front = np.array([(L2 +  box_full_size + 0.5 * box_half_size) * np.cos(np.deg2rad(ang2)), (L2 + box_full_size + 0.5 * box_half_size) * np.sin(np.deg2rad(ang2)), 130 + 1 * box_full_size + offset])
    box_1_high = np.array([(L2 + 2 * box_full_size + 0.5 * box_half_size) * np.cos(np.deg2rad(ang2)), (L2 + 2 * box_full_size + 0.5 * box_half_size) * np.sin(np.deg2rad(ang2)), 130 + 1 * box_full_size + offset])
    box_2 = np.array([(L2 + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 130 - box_full_size+offset])
    box_2_high = np.array([(L2 + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 130+offset])
    box_3 = np.array([(L2 + 3 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 3 * box_full_size) * np.sin(np.deg2rad(ang2)), 130 - box_full_size+offset])
    box_3_high = np.array([(L2 + 3 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 3 * box_full_size) * np.sin(np.deg2rad(ang2)), 130+offset])
    box_throw = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)), (L2 - 30) * np.sin(np.deg2rad(ang2)), drawer_height + 2 * box_full_size+offset])
    box_throw_high = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)), (L2 - 30) * np.sin(np.deg2rad(ang2)), drawer_height + 4 * box_full_size+offset])

    # # block 1, 2, 3 위치 + 90도 회전
    # # block 위치
    # box_1_90 = np.array([(tar_L2 + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 130])
    # box_1_high_90 = np.array([(tar_L2 + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 130 + 2 * box_full_size])
    # box_1_front_90 = np.array([(tar_L2 + box_full_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + box_full_size) * np.sin(np.deg2rad(ang2)), 130 - 2*box_full_size])
    # box_1_high_front_90 = np.array([(tar_L2 + box_full_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + box_full_size) * np.sin(np.deg2rad(ang2)), 130 + 2* box_full_size])
    # box_2_90 = np.array([(tar_L2 + 80 - box_half_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + 80 - box_half_size) * np.sin(np.deg2rad(ang2)), 130 - box_full_size])
    # box_2_high_90 = np.array([(tar_L2 + 80 - box_half_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + 80 - box_half_size) * np.sin(np.deg2rad(ang2)), 130])
    # box_3_90 = np.array([(tar_L2 + 80 + box_half_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + 80 + box_half_size) * np.sin(np.deg2rad(ang2)), 130 - box_full_size])
    # box_3_high_90 = np.array([(tar_L2 + 80 + box_half_size) * np.cos(np.deg2rad(ang2)), (tar_L2 + 80 + box_half_size) * np.sin(np.deg2rad(ang2)), 130])
    # box_throw_90 = np.array([(tar_L2 - 20) * np.cos(np.deg2rad(ang2)), (tar_L2 - 20) * np.sin(np.deg2rad(ang2)), drawer_height + 2 * box_full_size])
    # box_throw_high_90 = np.array([(tar_L2 - 20) * np.cos(np.deg2rad(ang2)), (tar_L2 - 20) * np.sin(np.deg2rad(ang2)), drawer_height + 4 * box_full_size])

    # 서랍 열기
    # move_to(*drawer_close_high_high, speed=50)
    # time.sleep(0.5)

    # move_to(*drawer_close_high, speed=50)
    # time.sleep(0.5)

    # move_to(*drawer_close, speed=50)
    # time.sleep(0.5)

    # dxl.grip_with_current(position_deg = GRIPPER_SEORAP_GRIP_POSITION, current_mA = 500, speed = 30)
    # time.sleep(2)

    # move_along_path(opening_drawer, speed=50)
    # time.sleep(1)

    # dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    # time.sleep(1)

    # drawer_open_high = generate_linear_path(drawer_open, drawer_open_high, num_points=3)
    # move_along_path(drawer_open_high, speed=50)
    # time.sleep(1)

    # 서랍에서 블록 1 집기
    # move_to(*box_1_high_front, speed=50)
    # time.sleep(1.5)
    # move_to(*box_1_high, speed=50)
    # time.sleep(1.5)
    # box1_path = generate_linear_path(box_1_high_front, box_1_high, num_points=4)
    # move_along_path(box1_path, speed=50)
    # time.sleep(1.5)
    # # move_to(*box_1, speed=50)
    # # time.sleep(1)
    # box_1_grip_path = generate_linear_path(box_1_high, box_1, num_points=4)
    # move_along_path(box_1_grip_path, speed=50)
    # time.sleep(1)

    dxl.grip_with_current(position_deg = GRIPPER_BOX_GRIP_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)
    

    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)
    

    dxl.grip_with_current(position_deg = GRIPPER_BOX_GRIP_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)
    

    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)
    

    dxl.grip_with_current(position_deg = GRIPPER_BOX_GRIP_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)
    
    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)









    # move_to(*drawer_open, speed=50)
    # time.sleep(0.5)



    # dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    # time.sleep(1)

    # move_to(*drawer_open, speed=50)
    # time.sleep(1)

    # move_to(*block_1_high, speed=50)
    # time.sleep(1)

    # move_to(*block_1, speed=50)
    # time.sleep(1)

    # dxl.grip_with_current(position_deg = GRIPPER_BOX_GRIP_POSITION, current_mA = 500, speed = 30)
    # time.sleep(1)

    # move_to(*block_1_high, speed=50)
    # time.sleep(1)

    # move_to(*block_throw, speed=50)
    # time.sleep(1)

    # dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    # time.sleep(1)









    # dxl.grip_with_current(position_deg = GRIPPER_SEORAP_GRIP_POSITION, current_mA = 500, speed = 30)
    # time.sleep(3)
    # dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    # time.sleep(1)

    

    # dxl.disable_torque(JOINT_IDS)
    dxl.close()