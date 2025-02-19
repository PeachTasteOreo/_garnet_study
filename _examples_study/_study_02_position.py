from dynamixel_sdk import *  
import os

#입력값 반환 함수
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

ADDR_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095

BAUDRATE = 57600    
PROTOCOL_VERSION = 2.0
DEVICENAME = "/dev/ttyACM0" 
#"/dev/ttyUSB*"
#COM3
DXL_MOVING_STATUS_THRESHOLD = 10

ADDR_GOAL_VELOCITY = 104

ADDR_OPERATE_MODE = 11
current_control = 0
velocity_control = 1
position_control = 3
extended_position_control = 4

DXL_ID = 4

index = 0
goal_position = 1000
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)


        


#포트 열기
portHandler.openPort() and portHandler.setBaudRate(BAUDRATE) 

# 보드 레이트 연결하기
portHandler.setBaudRate(BAUDRATE)


#포지션 모드로 설정
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATE_MODE, position_control)


#토크 켜기
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

print("qqq")




#속도 모드로 설정
#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATE_MODE, velocity_control)


while 1 :
    #움직이는 코드
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)

    #현재 위치 가져오는 코드
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

    print(dxl_present_position)
    
    if not abs(goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
        break
    

    


#토크 다시 0 주고
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)


#포트 닫기
portHandler.closePort()




#print("\n 1 : ",dxl_comm_result, "\n\n ")
