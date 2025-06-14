import numpy as np
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
import time
from ikpy.link import OriginLink, DHLink
from ikpy.chain import Chain

# d1 = 112.25
d1 = 112.25 + 50
a2, a3, a4 = 140.0, 140.0, 103.5

DIRECTION = [ 1, 1, 1, 1]

JOINT_IDS = [1, 2, 3, 4]
DEVICENAME = 'COM6'
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
GRIPPER_OPEN_POSITION = 0
GRIPPER_BOX_GRIP_POSITION = -65
GRIPPER_SEORAP_GRIP_POSITION = -110

last_deg = []
flag = True
flag_2 = False
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

    # def grip(self, speed=20):
        
    #     self.set_speed(speed)
    #     raw = deg2dxl(GRIPPER_CLOSE_POSITION)
    #     param = [
    #         raw        & 0xFF,
    #         (raw >>  8)& 0xFF,
    #         (raw >> 16)& 0xFF,
    #         (raw >> 24)& 0xFF,
    #     ]
    #     self.packet.write4ByteTxRx(self.port,
    #                                GRIPPER_ID,
    #                                ADDR_GOAL_POSITION,
    #                                raw)
    
    # def box_grip(self, speed=20):

    #     self.set_speed(speed)
    #     raw = deg2dxl(GRIPPER_CLOSE_POSITION)
    #     param = [
    #         raw        & 0xFF,
    #         (raw >>  8)& 0xFF,
    #         (raw >> 16)& 0xFF,
    #         (raw >> 24)& 0xFF,
    #     ]
    #     self.packet.write4ByteTxRx(self.port,
    #                                GRIPPER_ID,
    #                                ADDR_GOAL_POSITION,
    #                                raw)


    # def ungrip(self, speed=20):
    #     self.set_speed(speed)
    #     raw = deg2dxl(GRIPPER_OPEN_POSITION)
    #     self.packet.write4ByteTxRx(self.port,
    #                                GRIPPER_ID,
    #                                ADDR_GOAL_POSITION,
    #                                raw)

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


    def move_joint(self, joint_id: int, position_deg: float, speed: int = None):
        raw_pos = deg2dxl(position_deg)
        self.packet.write4ByteTxRx(self.port, joint_id, ADDR_GOAL_POSITION, raw_pos)

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
    global flag
    global flag_2
    global last_deg
    if flag == True:
        last_deg = target_deg
        flag = False

    if  flag_2 == True:
        if np.linalg.norm(np.array(target_deg) - np.array(last_deg)) > 10:
            print("----------------error----------------------")
            pass
        else:
            dxl.set_positions(target_deg)

            dist=np.linalg.norm(np.array([x, y, z]) - p_current)
            print(np.array([x, y, z]) - p_current)
            time.sleep(dist/speed)
    else:
        dxl.set_positions(target_deg)

        dist=np.linalg.norm(np.array([x, y, z]) - p_current)
        # print(np.array([x, y, z]) - p_current)
        time.sleep(dist/speed)
    print(target_deg)
    last_deg = target_deg
    flag_2 = False

def generate_linear_path(start, end, num_points):
    global flag_2
    flag_2 = True
    return np.linspace(start, end, num_points)

def move_along_path(path, speed):
    for pt in path:
        move_to(*pt, speed=speed)
        # time.sleep(0.5)

def Read_my_position():
    DXL_pos = dxl.get_position(JOINT_IDS)
    DXL_pos = np.array(DXL_pos)
    end_effector_frame = robot_chain.forward_kinematics(DXL_pos)

    position = end_effector_frame[:3, 3]
    print("엔드이펙스 위치 :", position)

# def box_to_throw():
#     move_to(*temp_high, speed=150)

#     move_to(*L1_direction,speed= 150)

#     move_to(*box_1_high_front, speed=50)

#     move_to(*box_1, speed=50)
#     time.sleep(1)

#     dxl.grip_with_current(position_deg = GRIPPER_BOX_GRIP_POSITION, current_mA = 500, speed = 30)
    
#     move_to(*box_1_high, speed=50)
#     time.sleep(0.5)

#     move_to(*box_throw, speed=50)
#     time.sleep(0.5)

#     dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
#     time.sleep(1)

#     move_to(*box_throw_high, speed=70)


def box_to_throw(temp_high,L1_direction,box_num_high_front,box_num,box_num_high,L2_direction,box_throw,box_throw_high):
    move_to(*temp_high, speed=150)

    move_to(*L1_direction,speed= 150)

    move_to(*box_num_high_front, speed=50)

    move_to(*box_num, speed=50)
    time.sleep(1)

    dxl.grip_with_current(position_deg = GRIPPER_BOX_GRIP_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)

    linear_high = generate_linear_path(box_num,box_num_high,30)
    move_along_path(linear_high,50)
    # move_to(*box_num_high, speed=50)
    time.sleep(0.5)

    move_to(*L1_direction,speed= 150)
    
    move_to(*L2_direction,speed= 150)

    move_to(*box_throw_high, speed=70)

    move_to(*box_throw, speed=50)
    
    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)

    move_to(*box_throw_high, speed=70)

def box_to_throw2_precious(temp_high,L1_direction,box_num_high_front,box_num,L2_direction,box_place_num_high,box_place_num):
    move_to(*temp_high, speed=150)

    move_to(*L1_direction,speed= 150)

    move_to(*box_num_high_front, speed=50)

    linear_down = generate_linear_path(box_num_high_front,box_num,30)
    move_along_path(linear_down,50)
    time.sleep(1)

    dxl.grip_with_current(position_deg = GRIPPER_BOX_GRIP_POSITION, current_mA = 500, speed = 30)
    time.sleep(1)

    linear_high = generate_linear_path(box_num,box_num_high_front,30)
    move_along_path(linear_high,50)
    # move_to(*box_num_high, speed=50)
    time.sleep(0.5)

    move_to(*L1_direction,speed= 150)
    time.sleep(1)

    move_to(*L2_direction,speed= 150)
    time.sleep(1)
    
    move_to(*box_place_num_high, speed=70)
    time.sleep(1)

    linear_down = generate_linear_path(box_place_num_high,box_place_num,30)
    move_along_path(linear_down,50)
    print(linear_down)

    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 100)
    time.sleep(1)

    dxl.move_joint(2,400)
    time.sleep(0.5)

    move_to(*box_place_num_high, speed=70)
    # linear_high = generate_linear_path(box_place_num,box_place_num_high,30)
    # move_along_path(linear_high,50)

    move_to(*L2_direction,speed= 150)
    time.sleep(0.5)

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
    L1, L2, ang1, ang2 = map(float, target_str.strip().split(","))
    L1 += 103.5
    L2 += 103.5
    offset = 103.5

    # support
    x1, y1, z1 = L1*np.cos(np.deg2rad(ang1)), L1 * np.sin(np.deg2rad(ang1)), support_height
    # drawer
    x2, y2, z2 = L2*np.cos(np.deg2rad(ang2)), L2 * np.sin(np.deg2rad(ang2)), drawer_height

    # 서랍 열고 닫기
    drawer_close = np.array([(L2 - 10) * np.cos(np.deg2rad(ang2)),(L2 - 10) * np.sin(np.deg2rad(ang2)), 25+offset])
    drawer_close_high = np.array([(L2 - 10) * np.cos(np.deg2rad(ang2)),(L2 - 10) * np.sin(np.deg2rad(ang2)), 25 + 2*box_full_size+offset])

    drawer_close_high_edit = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)),(L2 - 30) * np.sin(np.deg2rad(ang2)), 100+offset])

    drawer_open = np.array([(L2 - 60) * np.cos(np.deg2rad(ang2)),(L2 - 60) * np.sin(np.deg2rad(ang2)), 25+offset])
    drawer_open_high = np.array([(L2 - 60) * np.cos(np.deg2rad(ang2)),(L2 - 60) * np.sin(np.deg2rad(ang2)), 25 + 2*box_full_size+offset])

    drawer_open_high_edit = np.array([(L2 - 60) * np.cos(np.deg2rad(ang2)),(L2 - 60) * np.sin(np.deg2rad(ang2)), 100+offset])

    temp_high = np.array([(L2 - 60) * np.cos(np.deg2rad(ang2)),(L2 - 60) * np.sin(np.deg2rad(ang2)), 200+offset])
    opening_drawer = generate_linear_path(drawer_close, drawer_open, num_points=100)
    close_drawer = generate_linear_path(drawer_open, drawer_close, num_points=100)

    # top_box_height = 70 + 4 * box_full_size - box_full_size/2
    top_box_height = 70 + 3 * box_full_size
    L1_direction = np.array([(L1 - 80) * np.cos(np.deg2rad(ang1)),(L1 - 80) * np.sin(np.deg2rad(ang1)), 250+offset])
    L2_direction = np.array([(L2 - 40) * np.cos(np.deg2rad(ang2)),(L2 - 40) * np.sin(np.deg2rad(ang2)), 250+offset])

    # block 위치
    box_1 = np.array([(L1 + 2 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + 2 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height+offset])
    box_1_high = np.array([(L1 + 2 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + 2 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height + 1 * box_full_size + offset])
    box_1_high_front = np.array([(L1 +  box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height + offset + 1 * box_full_size])


    box_2 = np.array([(L1 + box_full_size + box_half_size) * np.cos(np.deg2rad(ang1)), (L1 + box_full_size + box_half_size) * np.sin(np.deg2rad(ang1)), top_box_height+offset - 1 * box_full_size])
    box_2_high = np.array([(L1 + box_full_size + box_half_size) * np.cos(np.deg2rad(ang1)), (L1 + box_full_size + box_half_size) * np.sin(np.deg2rad(ang1)), top_box_height+offset])
    box_2_high_front = np.array([(L1 +  box_half_size) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size) * np.sin(np.deg2rad(ang1)), top_box_height+offset])

    box_3 = np.array([(L1 + 2 * box_full_size + box_half_size) * np.cos(np.deg2rad(ang1)), (L1 + 2 * box_full_size + box_half_size) * np.sin(np.deg2rad(ang1)), top_box_height+offset - 1 * box_full_size])
    box_3_high = np.array([(L1 + 2 * box_full_size + box_half_size) * np.cos(np.deg2rad(ang1)), (L1 + 2 * box_full_size + box_half_size) * np.sin(np.deg2rad(ang1)), top_box_height+offset])
    box_3_high_front = np.array([(L1 + box_full_size + box_half_size) * np.cos(np.deg2rad(ang1)), (L1 + box_full_size + box_half_size) * np.sin(np.deg2rad(ang1)), top_box_height+offset])



    box_throw = np.array([(L2 - 20) * np.cos(np.deg2rad(ang2)), (L2 - 20) * np.sin(np.deg2rad(ang2)), drawer_height +offset])
    box_throw_high = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)), (L2 - 30) * np.sin(np.deg2rad(ang2)), drawer_height + 4 * box_full_size+offset])

    box_4 = np.array([(L1 + box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 2 * box_full_size+offset])
    box_4_high = np.array([(L1 + box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 1 * box_full_size+offset])
    box_4_high_front = np.array([(L1) * np.cos(np.deg2rad(ang1)), (L1 ) * np.sin(np.deg2rad(ang1)),top_box_height - 1 * box_full_size+offset])

    box_5 = np.array([(L1 + 2 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + 2 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 2 * box_full_size+offset])
    box_5_high = np.array([(L1 + 2 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + 2 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 1 * box_full_size+offset])
    box_5_high_front = np.array([(L1 + box_full_size ) * np.cos(np.deg2rad(ang1)), (L1 + box_full_size) * np.sin(np.deg2rad(ang1)),top_box_height - 1 * box_full_size+offset])

    box_6 = np.array([(L1 + 3 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + 3 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 2 * box_full_size+offset])
    box_6_high = np.array([(L1 + 3 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + 3 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 1 * box_full_size+offset])
    box_6_high_front = np.array([(L1 + 2 * box_full_size ) * np.cos(np.deg2rad(ang1)), (L1 + 2 * box_full_size) * np.sin(np.deg2rad(ang1)),top_box_height - 1 * box_full_size+offset])

    first_floor_offset = 3
    box_7 = np.array([(L1 + box_half_size + 0 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size + 0 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 3 * box_full_size+offset +first_floor_offset])
    box_7_high = np.array([(L1 + box_half_size + 0 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size + 0 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 2 * box_full_size+offset])
    box_7_high_front = np.array([(L1 + box_half_size - 1 * box_full_size ) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size - 1 * box_full_size) * np.sin(np.deg2rad(ang1)),top_box_height - 2 * box_full_size+offset])

    box_8 = np.array([(L1 + box_half_size + 1 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size + 1 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 3 * box_full_size+offset +first_floor_offset])
    box_8_high = np.array([(L1 + box_half_size + 1 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size + 1 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 2 * box_full_size+offset])
    box_8_high_front = np.array([(L1 + box_half_size - 0 * box_full_size ) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size - 0 * box_full_size) * np.sin(np.deg2rad(ang1)),top_box_height - 2 * box_full_size+offset])

    box_9 = np.array([(L1 + box_half_size + 2 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size + 2 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 3 * box_full_size+offset +first_floor_offset])
    box_9_high = np.array([(L1 + box_half_size + 2 * box_full_size) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size + 2 * box_full_size) * np.sin(np.deg2rad(ang1)), top_box_height - 2 * box_full_size+offset])
    box_9_high_front = np.array([(L1 + box_half_size + 1 * box_full_size ) * np.cos(np.deg2rad(ang1)), (L1 + box_half_size + 1 * box_full_size) * np.sin(np.deg2rad(ang1)),top_box_height - 2 * box_full_size+offset])

    # place

    place_default = box_half_size
    place_offset_1st_floor = 3
    place_offset_2nd_floor = 8
    box_4_place = np.array([(L2 + place_default + 3 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 3 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+offset + place_offset_1st_floor])
    box_4_place_high = np.array([(L2 + place_default + 3 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 3 * box_full_size) * np.sin(np.deg2rad(ang2)), 42 + 1 * box_full_size+offset])
    box_4_place_high_front = np.array([(L2 + place_default + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42 + 1 * box_full_size+offset])

    box_5_place = np.array([(L2 + place_default + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+offset + place_offset_1st_floor])
    box_5_place_high = np.array([(L2 + place_default + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42 + 1 * box_full_size+offset])
    box_5_place_high_front = np.array([(L2 + place_default + 1 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 1 * box_full_size) * np.sin(np.deg2rad(ang2)), 42 + 1 * box_full_size+offset])

    box_6_place = np.array([(L2 + place_default + 1 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 1 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+offset + place_offset_1st_floor])
    box_6_place_high = np.array([(L2 + place_default + 1 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 1 * box_full_size) * np.sin(np.deg2rad(ang2)), 42 + 1 * box_full_size+offset])
    box_6_place_high_front = np.array([(L2 + place_default + 0 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + box_half_size + 0 * box_full_size) * np.sin(np.deg2rad(ang2)), 42 + 1 * box_full_size+offset])

    box_7_place = np.array([(L2 + 3 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 3 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 1 * box_full_size + offset + place_offset_2nd_floor])
    box_7_place_high = np.array([(L2 + 3 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 3 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 2 * box_full_size + offset])
    box_7_place_high_front = np.array([(L2 + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 2 * box_full_size + offset])

    box_8_place = np.array([(L2 + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 1 * box_full_size + offset + place_offset_2nd_floor])
    box_8_place_high = np.array([(L2 + 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 2 * box_full_size + offset])
    box_8_place_high_front = np.array([(L2 + 1 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2 + 1 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 2 * box_full_size + offset])

    box_9_place = np.array([(L2  +place_default+ 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2  +place_default+ 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 2 * box_full_size + offset + place_offset_2nd_floor])
    box_9_place_high = np.array([(L2  +place_default+ 2 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2  +place_default+ 2 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 3 * box_full_size + offset])
    box_9_place_high_front = np.array([(L2  +place_default+ 1 * box_full_size) * np.cos(np.deg2rad(ang2)), (L2  +place_default+ 1 * box_full_size) * np.sin(np.deg2rad(ang2)), 42+ 3 * box_full_size + offset])

    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 100)
    time.sleep(1)

    move_to(*drawer_open, speed=50)

    move_to(*drawer_close, speed=50)

    dxl.grip_with_current(position_deg = GRIPPER_SEORAP_GRIP_POSITION, current_mA = 500, speed = 30)

    move_along_path(opening_drawer, speed=50)
    time.sleep(1)

    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)


    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    time.sleep(0.5)

    drawer_open_high = generate_linear_path(drawer_open, drawer_open_high_edit, num_points=10)
    print(drawer_open_high)
    move_along_path(drawer_open_high, speed=150)

    box_to_throw(temp_high,L1_direction,box_1_high_front,box_1,box_1_high,L2_direction,box_throw,box_throw_high)
    time.sleep(0.5)
    box_throw = np.array([(L2 - 20) * np.cos(np.deg2rad(ang2+10)), (L2 - 20) * np.sin(np.deg2rad(ang2+10)), drawer_height +offset])
    box_throw_high = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2+10)), (L2 - 30) * np.sin(np.deg2rad(ang2+10)), drawer_height + 4 * box_full_size+offset])
    box_to_throw(temp_high,L1_direction,box_2_high_front,box_2,box_2_high,L2_direction,box_throw,box_throw_high)
    time.sleep(0.5)
    box_throw = np.array([(L2 - 20) * np.cos(np.deg2rad(ang2-10)), (L2 - 20) * np.sin(np.deg2rad(ang2-10)), drawer_height +offset])
    box_throw_high = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2-10)), (L2 - 30) * np.sin(np.deg2rad(ang2-10)), drawer_height + 4 * box_full_size+offset])
    box_to_throw(temp_high,L1_direction,box_3_high_front,box_3,box_3_high,L2_direction,box_throw,box_throw_high)
    time.sleep(0.5)
    
    box_throw = np.array([(L2 - 20) * np.cos(np.deg2rad(ang2)), (L2 - 20) * np.sin(np.deg2rad(ang2)), drawer_height +offset])
    box_throw_high = np.array([(L2 - 30) * np.cos(np.deg2rad(ang2)), (L2 - 30) * np.sin(np.deg2rad(ang2)), drawer_height + 4 * box_full_size+offset])
    
    dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 100)
    time.sleep(1)
    move_to(*drawer_open, speed=50)

    move_to(*drawer_close, speed=50)
    time.sleep(1)

    # dxl.grip_with_current(position_deg = GRIPPER_SEORAP_GRIP_POSITION, current_mA = 500, speed = 30)
    # move_along_path(close_drawer, speed=50)
    # time.sleep(1)

    drawer_close_high = generate_linear_path(drawer_close, drawer_close_high_edit, num_points=10)
    print(drawer_close_high)
    move_along_path(drawer_close_high, speed=100)

    move_to(*L2_direction,speed=50)
    time.sleep(1)

    box_to_throw2_precious(temp_high,L1_direction,box_4_high_front,box_4,L2_direction,box_4_place_high_front,box_4_place)
    time.sleep(0.5)
    box_to_throw2_precious(temp_high,L1_direction,box_5_high_front,box_5,L2_direction,box_5_place_high_front,box_5_place)
    time.sleep(0.5)
    box_to_throw2_precious(temp_high,L1_direction,box_6_high_front,box_6,L2_direction,box_6_place_high_front,box_6_place)
    time.sleep(0.5)
    box_to_throw2_precious(temp_high,L1_direction,box_7_high_front,box_7,L2_direction,box_7_place_high_front,box_7_place)
    time.sleep(0.5)
    box_to_throw2_precious(temp_high,L1_direction,box_8_high_front,box_8,L2_direction,box_8_place_high_front,box_8_place)
    time.sleep(0.5)
    box_to_throw2_precious(temp_high,L1_direction,box_9_high_front,box_9,L2_direction,box_9_place_high_front,box_9_place)
    time.sleep(0.5)






    # dxl.grip_with_current(position_deg = GRIPPER_SEORAP_GRIP_POSITION, current_mA = 500, speed = 30)
    # time.sleep(3)
    # dxl.grip_with_current(position_deg = GRIPPER_OPEN_POSITION, current_mA = 500, speed = 30)
    # time.sleep(1)

    

    # dxl.disable_torque(JOINT_IDS)
    dxl.close()