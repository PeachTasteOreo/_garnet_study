
import _settings
import os
import time
#import keyboard
import select

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * # Uses Dynamixel SDK library

#********* DYNAMIXEL Model definition *********
MY_DXL = 'X_SERIES' #OK       # X330 (5.0 V recommended), X430, X540, 2X4

# Control table address
ADDR_TORQUE_ENABLE          = 64 #OK
ADDR_GOAL_POSITION          = 116 #OK
ADDR_PRESENT_POSITION       = 132 #OK
DXL_MINIMUM_POSITION_VALUE  = 0 #OK         # Refer to the Minimum Position Limit of product eManual 
DXL_MAXIMUM_POSITION_VALUE  = 4095 #OK      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 57600 #OK
ADDR_GOAL_VELOCITY = 104

PROTOCOL_VERSION            = 2.0 #OK  

DXL_ID                      = 1 #OK

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = "/dev/ttyUSB0"  #OK

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 10    # Dynamixel moving status threshold 
                                    #>0부터 1023까지 있음.

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

portHandler = PortHandler(DEVICENAME)

packetHandler = PacketHandler(PROTOCOL_VERSION)



# 필요한 변수 정의
DEVICENAME = "/dev/ttyUSB0"   # 사용 중인 포트 이름 (예: "COM3")
BAUDRATE = 57600    # 설정할 통신 속도
DXL_ID = 1          # 다이나믹셀 ID
ADDR_TORQUE_ENABLE = 64  # 토크 활성화 주소
TORQUE_ENABLE = 1        # 토크 활성화 값

# PortHandler 및 PacketHandler 객체 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(2.0)  # 사용 중인 프로토콜 버전


ADDR_OPERATE_MODE = 11
current_control = 0
velocity_control = 0x01
position_control = 3
extended_position_control = 4










_settings.start(portHandler, BAUDRATE)

_settings.mode(portHandler, packetHandler, 1, position_control) # 돌리는 셀 
_settings.mode(portHandler, packetHandler, 7, position_control) # 따라가는 셀



# 7번 셀 토크 온
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 7, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# 1번 셀 토크 오프
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, 1, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

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

if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 설정 오류")
else:
    print("Dynamixel has been successfully connected")

while True:
    # ESC 키가 눌렸는지 체크
    key = get_key()
    if key == chr(27):  # ESC 키가 눌리면 종료
        print("프로그램 종료")
        break
    
    # 1번 셀 위치 가져오기
    dxl_present_position_1, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 1, ADDR_PRESENT_POSITION)
    print(f"\n1번셀 현재 위치 : {dxl_present_position_1} \n")
    
    # 7번 셀 위치 가져오기
    dxl_present_position_7, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 7, ADDR_PRESENT_POSITION)
    print(f"\n7번셀 현재 위치 : {dxl_present_position_7} \n")
    
    # 1번 셀의 위치를 7번셀 목표로 지정 
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 7, ADDR_GOAL_POSITION, dxl_present_position_1)
    if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
        print("목표 위치 설정 오류")

    #time.sleep(0.01) 
    

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 비활성화 오류")

# Close port
portHandler.closePort()
