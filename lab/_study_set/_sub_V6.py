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

# follow 셀 토크 온
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
)
if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 설정 오류")
else:
    print("Dynamixel 연결 성공!")

def _settings_v(velocity):
    packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, velocity)
    dxl_present_position, _, _ = packetHandler.read4ByteTxRx(
        portHandler, dxl_follow, ADDR_PRESENT_POSITION
    )
    print(f"현재 위치: {dxl_present_position}")

# 종료 신호를 위한 전역 변수
exit_requested = False

def callback(msg):
    global exit_requested
    key = msg.data.strip()  # 공백 제거
    print(f"입력: {key}")

    if key.lower() == "q":  # 종료 명령
        print("프로그램 종료 명령 수신")
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("토크 비활성화 오류")
        exit_requested = True  # 종료 신호 설정
    elif key.lower() == "a":  # 왼쪽 이동
        _settings_v(100)
    elif key.lower() == "d":  # 오른쪽 이동
        _settings_v(-100)
    elif key.lower() == "s":  # 정지
        print("정지 명령 실행")
        _settings_v(0)
    else:
        print("잘못된 입력")
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(
            portHandler, dxl_follow, ADDR_PRESENT_POSITION
        )
        print(f"현재 위치: {dxl_present_position}")

def main(args=None):
    global exit_requested
    rclpy.init(args=args)
    node = Node('subscriber_node')
    node.create_subscription(String, 'my_topic', callback, 10)

    print('Node started, waiting for messages...')
    dxl_present_position, _, _ = packetHandler.read4ByteTxRx(
        portHandler, dxl_follow, ADDR_PRESENT_POSITION
    )
    print(f"현재 위치: {dxl_present_position}")

    # 종료 명령(q)이 들어올 때까지 spin_once로 메시지 처리
    while rclpy.ok() and not exit_requested:
        rclpy.spin_once(node, timeout_sec=0.1)

    # spin 종료 후 cleanup 진행
    node.destroy_node()
    portHandler.closePort()  # Dynamixel 포트 닫기
    print("프로그램 종료")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
