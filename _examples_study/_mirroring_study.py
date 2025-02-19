import os
from dynamixel_sdk import *  # Uses Dynamixel SDK library

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

# Constants
MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE = 64
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_GOAL_POSITION = 116
LEN_GOAL_POSITION = 4
BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DEVICENAME = "COM3"
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# User-defined IDs
dxl_source_id = 1  # Dynamixel ID for the source (rotated manually)
dxl_target_id = 7  # Dynamixel ID for the target (follows the source)

# Initialize instances
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Functions
def check_result(result, error, success_msg, fail_msg):
    if result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(result))
    elif error != 0:
        print(packetHandler.getRxPacketError(error))
    else:
        print(success_msg)

def enable_torque(dxl_id):
    result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    check_result(result, error, f"Dynamixel#{dxl_id} torque enabled", f"Failed to enable torque for Dynamixel#{dxl_id}")

def disable_torque(dxl_id):
    result, error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    check_result(result, error, f"Dynamixel#{dxl_id} torque disabled", f"Failed to disable torque for Dynamixel#{dxl_id}")

def get_present_position(dxl_id):
    pre_position, result, error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    if result != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(result))
    elif error != 0:
        print(packetHandler.getRxPacketError(error))
    return result

def set_goal_position(dxl_id, goal_position):
    param_goal_position = [
        DXL_LOBYTE(DXL_LOWORD(goal_position)),
        DXL_HIBYTE(DXL_LOWORD(goal_position)),
        DXL_LOBYTE(DXL_HIWORD(goal_position)),
        DXL_HIBYTE(DXL_HIWORD(goal_position)),
    ]
    if not groupSyncWrite.addParam(dxl_id, param_goal_position):
        print(f"[ID:{dxl_id:03d}] groupSyncWrite addparam failed")
        quit()
    if groupSyncWrite.txPacket() != COMM_SUCCESS:
        print(packetHandler.getTxRxResult(groupSyncWrite.txPacket()))
    groupSyncWrite.clearParam()

# Main script
if portHandler.openPort() and portHandler.setBaudRate(BAUDRATE):
    print("Port opened and baudrate set")
else:
    print("Failed to open port or set baudrate")
    quit()

# Enable torque for both motors
#enable_torque(dxl_source_id)
disable_torque(dxl_source_id)
enable_torque(dxl_target_id)

try:
    print("Press ESC to quit.")
    while True:
        # Get current position of the source Dynamixel
        source_position = get_present_position(dxl_source_id)
        if source_position is None:
            print("Failed to read source position. Retrying...")
            continue

        print(f"Source Position: {source_position}")

        # Set the target Dynamixel to match the source position
        set_goal_position(dxl_target_id, source_position)

        # Check for ESC key press
        if getch() == chr(0x1b):  # ESC key
            break
finally:
    # Disable torque and close port
    disable_torque(dxl_source_id)
    disable_torque(dxl_target_id)
    portHandler.closePort()
    print("Program terminated.")

