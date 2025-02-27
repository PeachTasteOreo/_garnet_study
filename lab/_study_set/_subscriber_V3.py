import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk import *  # Dynamixel SDK

from _settings import *
import _settings

dxl_lead = 3
dxl_follow = 4

# PortHandler 및 PacketHandler 객체 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

_settings.start(portHandler, BAUDRATE)
_settings.mode(portHandler, packetHandler, dxl_follow, velocity_control)

# lead 셀 토크 온
#dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_lead, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# follow 셀 토크 온
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 설정 오류")
else:
    print("Dynamixel 연결 성공!")

def _settings_v(velocity):
    packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, velocity)
    dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
    print(f"현재 위치: {dxl_present_position}")
    


def callback(msg):
    key = msg.data.strip()  # 공백 제거
    print(f"입력: {key}")

    if key.lower() == "esc":  # ESC 입력 처리
        print("Exiting program...")
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("exiting error")
        else:
            pass
            print("ss")
        


    elif key.lower() == "a":  # 왼쪽 이동
        # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 100)
        # dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        # print(f"현재 위치: {dxl_present_position}")
        _settings_v(100)

    elif key.lower() == "d":  # 오른쪽 이동
        # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 0xFFFFFF9C)  # 음수 속도
        # dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        # print(f"현재 위치: {dxl_present_position}")
        _settings_v(-100)

    elif key.lower() == "s":  # 정지
        print("정지 명령 실행")
        # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 0)
        # dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        # print(f"현재 위치: {dxl_present_position}")
        _settings_v(0)

    else:
        print("잘못된 입력")
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"현재 위치: {dxl_present_position}")

if __name__ == '__main__':
    rclpy.init()
    node = Node('subscriber_node')
    subscription = node.create_subscription(String, 'my_topic', callback, 10)

    try:
        print('Node started, waiting for messages...')
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"현재 위치: {dxl_present_position}")
        rclpy.spin(node)  # 한 번만 실행
    except KeyboardInterrupt:
        print("?")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
        print("finished")
