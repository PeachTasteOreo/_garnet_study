from dynamixel_sdk import *
import _settings
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
DEVICENAME = "/dev/ttyUSB0" 

DXL_MOVING_STATUS_THRESHOLD = 20

ADDR_GOAL_VELOCITY = 104

ADDR_OPERATE_MODE = 11
current_control = 0
velocity_control = 0x01
position_control = 3
extended_position_control = 4

# ID 여러개 변경 가능
# dxl_ids = [1, 7]  

DXL_ID = 1

index = 0
goal_position = 2048
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

#_settings.start()
        
#모드 설정함수
def select_mode(mode_number):
    
    packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    
    result, error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_OPERATE_MODE, mode_number)
    if result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(result))
    elif error != 0:
        print(packetHandler.getRxPacketError(error))
    else :
        print("ok")
    
        
        

# 포트 및 보드 연결
portHandler.openPort() and portHandler.setBaudRate(BAUDRATE) 


#_settings.select_mode(velocity_control)
select_mode(velocity_control)

#토크 켜기
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


i =0


dxl_present_position_first, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

while 1 :
    # 속도 설정해주기
    # 움직이는 코드
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, 50)

    #현재 위치 가져오는 코드
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

    print("현재위치 : ", dxl_present_position)
    
    print(" 000 : ", abs(dxl_present_position_first - dxl_present_position))
    
    
    #설정한 위치(회전수)보다는 더 회전하지 않음 
    # 높은 회전 속도에서는 정확하지 않음
    if abs(dxl_present_position_first - dxl_present_position) > goal_position :
        break 
    
    
    
    #디버깅용, 최대 200번 실행되고 종료 
    print("\n1 : 목표값 : ", goal_position)
    print("2 : 현재위치값 : ", dxl_present_position)
    print("3 : 상대적인 위치 차이 : ", abs(goal_position - dxl_present_position)%DXL_MAXIMUM_POSITION_VALUE)
    print("4 : 현재 반복 횟수 : ", i,"번째 동작중\n")
    i += 1
    if i > 200 :
        break
    #time.sleep(0.2)
    

    


#토크 0
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)

#포트 닫기
portHandler.closePort()




#print("\n 1 : ",dxl_comm_result, "\n\n ")
