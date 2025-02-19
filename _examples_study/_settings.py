#********************************************************************************************
#
#
#                                   시스템 기본 설정
#
#
#********************************************************************************************
import os
from dynamixel_sdk import *


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










#********************************************************************************************
#
#
#                                   다이나믹셀 기본 설정
#
#
#********************************************************************************************
# ***** DYNAMIXEL Control Table Addresses *****
ADDR_TORQUE_ENABLE = 64           
ADDR_GOAL_POSITION = 116          
LEN_GOAL_POSITION = 4             
ADDR_PRESENT_POSITION = 132       
LEN_PRESENT_POSITION = 4          
ADDR_GOAL_VELOCITY = 104          
ADDR_OPERATE_MODE = 11            

# ***** Torque Values *****
TORQUE_ENABLE = 1                 
TORQUE_DISABLE = 0                

# ***** Position and Motion Values *****
DXL_MINIMUM_POSITION_VALUE = 0    
DXL_MAXIMUM_POSITION_VALUE = 4095 
DXL_MOVING_STATUS_THRESHOLD = 10  

# ***** Communication Settings *****
BAUDRATE = 57600                  
PROTOCOL_VERSION = 2.0            

# ***** Device and ID Information *****
DEVICENAME = "/dev/ttyACM0"               
DXL_ID = 1                       

# ***** Operating Modes *****
current_control = 0              
velocity_control = 1           
position_control = 3              
extended_position_control = 4     


# 포트와 핸들러 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)










#********************************************************************************************
#
#
#                                     메인 함수 설정
#
#
#********************************************************************************************



# 포트와 보드레이트 설정하는 함수
# _settings.start(portHandler, BAUDRATE)

def start(portHandler, BAUDRATE):

    if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
        print("Port opened and baudrate set **_settings")
        return True
    else:
        print("Failed to open port or set baudrate **_settings")
        return False


#모드 설정함수
# _settings.mode(portHandler, packetHandler, 1, position_control)
def mode(portHandler, packetHandler, id, mode_number):
    # Torque Disable
    packetHandler.write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    
    # Set Operating Mode
    result, error = packetHandler.write2ByteTxRx(portHandler, id , ADDR_OPERATE_MODE, mode_number)
    if result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(result))
        print(" 모터 모드 설정 오류! : 결과값 반환 실패 ")
    elif error != 0:
        print(packetHandler.getRxPacketError(error))
        print(" 모터 모드 설정 오류! : 에러 발생 ")
    else:
        print("Operating mode set successfully")
        return True

        
