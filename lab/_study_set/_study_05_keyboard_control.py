import time
import sys
import tty
import termios
import select
from dynamixel_sdk import * # Uses Dynamixel SDK library


from _settings import *
import _settings

# 리눅스에서 비동기 키입력을 받을 수 있도록 하는 함수
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # 0.1초 동안 키 입력 기다림
        if rlist:
            ch = sys.stdin.read(1)
            return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None  # 키가 없으면 None 반환


dxl_lead = 3
dxl_follow = 4


# PortHandler 및 PacketHandler 객체 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


_settings.start(portHandler, BAUDRATE)

#_settings.mode(portHandler, packetHandler, dxl_lead, position_control) # 돌리는 셀 
_settings.mode(portHandler, packetHandler, dxl_follow, velocity_control) # 따라가는 셀

# lead 셀 토크 온
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_lead, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# follow 셀 토크 오프
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 설정 오류")
else:
    print(" 1 : Dynamixel has been successfully connected")





while True:
    key = get_key()
    
    if key == chr(27):  # ESC 키가 눌리면 종료
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        print("Exiting program...")
        break
    
    elif key == "A" or key == "a":  # 왼쪽 화살표 키
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 100)
        dxl_present_position_7, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"\n7번셀 현재 위치 : {dxl_present_position_7} \n")
        print("Left arrow key pressed!")
    
    elif key == "D" or key == "d":  # 오른쪽 화살표 키
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, -100)
        dxl_present_position_7, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"\n7번셀 현재 위치 : {dxl_present_position_7} \n")
        print("Right arrow key pressed!")
    
    else:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 0)

    time.sleep(0.01)  # 0.01초 대기

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_lead, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 비활성화 오류")

# Close port
portHandler.closePort()
