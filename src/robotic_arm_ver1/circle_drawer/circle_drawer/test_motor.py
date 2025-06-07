
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

JOINT_IDS       = [1, 2, 3, 4]
moving_speed    = 256        
goal_position   = int(input("목표 위치 (0~300도): "))

def deg_to_dxl(deg: float) -> int:
    deg = max(0.0, min(300.0, deg))
    return int(deg / 300.0 * 1023)

def dxl_to_deg(val: int) -> float:
    return val / 1023.0 * 300.0

def enable_torque(port, packet_handler, dxl_id):
    packet_handler.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def disable_torque(port, packet_handler, dxl_id):
    packet_handler.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

def set_moving_speed(port, packet_handler, dxl_id, speed):
    packet_handler.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, speed)

def set_goal_position(port, packet_handler, dxl_id, pos):
    packet_handler.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, pos)

def get_present_position(port, packet_handler, dxl_id):
    val, _, _ = packet_handler.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    return dxl_to_deg(val)

def get_present_speed(port, packet_handler, dxl_id):
    val, _, _ = packet_handler.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_SPEED)
    return val

def main():
    port_handler   = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    if not port_handler.openPort():
        print("포트 열기 실패")
        return
    if not port_handler.setBaudRate(BAUDRATE):
        print("Baudrate 설정 실패")
        return

    # 토크 인에이블 및 목표값 세팅
    # for dxl_id in JOINT_IDS:
    #     enable_torque(port_handler, packet_handler, dxl_id)
    #     set_moving_speed(port_handler, packet_handler, dxl_id, moving_speed)
    #     set_goal_position(port_handler, packet_handler, dxl_id, goal_position)
    #     time.sleep(0.05)

    dxl_id = JOINT_IDS[1]  # 3 번째 모터 test
    enable_torque(port_handler, packet_handler, dxl_id)
    set_moving_speed(port_handler, packet_handler, dxl_id, moving_speed)
    set_goal_position(port_handler, packet_handler, dxl_id, deg_to_dxl(goal_position))

    # 동작 완료 대기
    time.sleep(1.0)

    # 현재 상태 출력
    for dxl_id in JOINT_IDS:
        pos = get_present_position(port_handler, packet_handler, dxl_id)
        spd = get_present_speed(port_handler, packet_handler, dxl_id)
        print(f"ID {dxl_id} → 위치: {pos:.1f}°, 속도: {spd}")

    # 종료 처리
    for dxl_id in JOINT_IDS:
        disable_torque(port_handler, packet_handler, dxl_id)
    port_handler.closePort()

if __name__ == "__main__":
    main()
