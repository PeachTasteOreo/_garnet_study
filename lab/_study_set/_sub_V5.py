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
    dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
    print(f"현재 위치: {dxl_present_position}")

class MySubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(String, 'my_topic', self.callback, 10)
        # 종료를 위한 Future (메인 루프에서 spin_until_future_complete에 사용)
        self.exit_future = None

    def callback(self, msg):
        key = msg.data.strip()
        self.get_logger().info(f"입력: {key}")

        if key.lower() == "q":  # 종료 명령
            self.get_logger().info("프로그램 종료 명령 수신")
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
                portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
            )
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                self.get_logger().error("토크 비활성화 오류")
            # Future에 결과를 설정하여 메인 루프에 종료를 알림
            if self.exit_future is not None:
                self.exit_future.set_result(True)
            return

        elif key.lower() == "a":  # 왼쪽 이동
            _settings_v(100)

        elif key.lower() == "d":  # 오른쪽 이동
            _settings_v(-100)

        elif key.lower() == "s":  # 정지
            self.get_logger().info("정지 명령 실행")
            _settings_v(0)

        else:
            self.get_logger().info("잘못된 입력")
            dxl_present_position, _, _ = packetHandler.read4ByteTxRx(
                portHandler, dxl_follow, ADDR_PRESENT_POSITION
            )
            self.get_logger().info(f"현재 위치: {dxl_present_position}")

def main(args=None):
    rclpy.init(args=args)
    node = MySubscriber()

    # 종료 신호를 위한 Future 생성 및 할당
    exit_future = rclpy.Future()
    node.exit_future = exit_future

    print('Node started, waiting for messages...')
    dxl_present_position, _, _ = packetHandler.read4ByteTxRx(
        portHandler, dxl_follow, ADDR_PRESENT_POSITION
    )
    print(f"현재 위치: {dxl_present_position}")

    # Future가 완료될 때까지 spin (q 입력 시 Future가 완료됨)
    rclpy.spin_until_future_complete(node, exit_future)

    # spin 종료 후 cleanup 진행
    node.destroy_node()
    portHandler.closePort()  # Dynamixel 포트 닫기
    print("프로그램 종료")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
