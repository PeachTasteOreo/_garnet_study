import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk import *  # Dynamixel SDK 라이브러리 사용
from _settings import *
import _settings

# Dynamixel 설정
dxl_follow = 4

# PortHandler 및 PacketHandler 객체 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

_settings.start(portHandler, BAUDRATE)
_settings.mode(portHandler, packetHandler, dxl_follow, velocity_control)  # 따라가는 셀 모드 설정

# Dynamixel 토크 설정
packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

def callback(msg):
    """ ROS 2에서 키 입력 메시지를 받아 Dynamixel 모터를 제어하는 콜백 함수 """
    key = msg.data
    print(f"받은 키 입력: {key}")

    if key == "ESC":  # ESC 키가 눌리면 종료
        print("프로그램 종료 중...")
        packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 0)
        portHandler.closePort()
        rclpy.shutdown()
    
    elif key.lower() == "a":  # 왼쪽 이동
        packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 100)
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"현재 위치: {dxl_present_position} | 왼쪽 이동")
    
    elif key.lower() == "d":  # 오른쪽 이동
        packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, -100)
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"현재 위치: {dxl_present_position} | 오른쪽 이동")
    
    elif key.lower() == "s":  # 정지
        print("정지")
        packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 0)
        portHandler.closePort()
    
    else:
        print("알 수 없는 입력")

def main():
    rclpy.init()
    node = Node('subscriber_node')
    node.create_subscription(String, 'my_topic', callback, 10)

    try:
        print("Subscriber 시작...")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("프로그램 종료 중...")
        packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()
    finally:
        print("Dynamixel 모터 정리 중...")
        packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        portHandler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    print("*** 프로그램 종료 ***")
