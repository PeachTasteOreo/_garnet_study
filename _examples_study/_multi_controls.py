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

# adress table 값들들 변경 확인

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
DEVICENAME = "COM3"

DXL_MOVING_STATUS_THRESHOLD = 10

# ID 여러개 변경 가능
dxl_ids = [1, 7]  # Dynamixel IDs (example list)
index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]

# 포트 핸들러 시작
portHandler = PortHandler(DEVICENAME)
print(type(dxl_ids))
print(type(portHandler))
print(portHandler)


packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

#패킷을 제대로 불러왔는지 확인
def check_result(result, error, success_msg, fail_msg):
    if result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(result))
        print(fail_msg)
    elif error != 0:
        print(packetHandler.getRxPacketError(error))
        print(fail_msg)
    else:
        print(success_msg)

def enable_torque(dxl_id):
    result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print("11111",result)
    print("22222",error)
    check_result(result, error, f"Dynamixel#{dxl_id} torque enabled", f"Failed to enable torque for Dynamixel#{dxl_id}")

def disable_torque(dxl_id):
    result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    check_result(result, error, f"Dynamixel#{dxl_id} torque disabled", f"Failed to disable torque for Dynamixel#{dxl_id}")

def add_param_to_read(dxl_id):
    if not groupSyncRead.addParam(dxl_id):
        print(f"[ID:{dxl_id:03d}] groupSyncRead addparam failed")
        quit()

def sync_write_goal_position(goal_position):
    param_goal_position = [
        DXL_LOBYTE(DXL_LOWORD(goal_position)),
        DXL_HIBYTE(DXL_LOWORD(goal_position)),
        DXL_LOBYTE(DXL_HIWORD(goal_position)),
        DXL_HIBYTE(DXL_HIWORD(goal_position)),
    ]

    for dxl_id in dxl_ids:
        if not groupSyncWrite.addParam(dxl_id, param_goal_position):
            print(f"[ID:{dxl_id:03d}] groupSyncWrite addparam failed")
            quit()
    if groupSyncWrite.txPacket() != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(groupSyncWrite.txPacket()))
    groupSyncWrite.clearParam()

def read_present_position():
    if groupSyncRead.txRxPacket() != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(groupSyncRead.txRxPacket()))
    positions = {}
    for dxl_id in dxl_ids:
        if not groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
            print(f"[ID:{dxl_id:03d}] groupSyncRead getdata failed")
            quit()
        positions[dxl_id] = groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
    return positions



#*************************************************************************************************************
#
#
#
#
#                                            메인 함수 시작
#
#
#
#
#*************************************************************************************************************

# Main script
if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
    print("Port opened and baudrate set")
else:
    print("Failed to open port or set baudrate")
    quit()

# Enable torque and add parameters for all IDs
for dxl_id in dxl_ids:
    enable_torque(dxl_id)
    add_param_to_read(dxl_id)

while True:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):  # ESC key
        break

    sync_write_goal_position(dxl_goal_position[index])

    while True:
        positions = read_present_position()
        for dxl_id in dxl_ids:
            print(f"[ID:{dxl_id:03d}] GoalPos:{dxl_goal_position[index]:03d}  PresPos:{positions[dxl_id]:03d}")
        if all(abs(dxl_goal_position[index] - positions[dxl_id]) <= DXL_MOVING_STATUS_THRESHOLD for dxl_id in dxl_ids):
            break
    index = 1 - index

# Disable torque and close port
for dxl_id in dxl_ids:
    disable_torque(dxl_id)
portHandler.closePort()
