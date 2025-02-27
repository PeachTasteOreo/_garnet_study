import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import sys
from _settings2 import *

# Dynamixel 설정
ids = [3, 4]
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

start()
mode(ids,velocity_control)

# 최종 포트 상태 확인
if not portHandler.is_open:
    print("🚨 Main : 포트가 닫혀 있어 다시 엽니다.")
    portHandler.openPort()

if not portHandler.is_open:
    print("🚨 포트를 다시 열려고 했지만 실패했습니다. 프로그램을 종료합니다.")
    exit(1)

# Dynamixel 토크 활성화
for motor_id in ids:
    result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if result != COMM_SUCCESS:
        print(f"❌ Torque Enable 실패 (ID {motor_id}): {packetHandler.getTxRxResult(result)}")
    elif error != 0:
        print(f"❌ Torque Enable 에러 (ID {motor_id}): {packetHandler.getRxPacketError(error)}")
    else:
        print(f"✅ Torque Enable 성공 (ID {motor_id})")


set_position(ids,500)
set_velocity(ids,40)

class JetsonNode(Node):
    def __init__(self):
        super().__init__('jetson_node')
        self.exit_requested = False
        self.publisher = self.create_publisher(Int32, 'position_topic', 10)
        self.subscription = self.create_subscription(String, 'key_input_topic', self.process_keys, 10)
        self.timer = self.create_timer(0.5, self.publish_position)
    
    def process_keys(self, msg):
        key = msg.data.strip().lower()  # 공백 제거 및 소문자 변환
        # print(f"✅ Main : Received key input: {key}")
        sys.stdout.write(f"\rReceived key input: {key}   ")
        sys.stdout.flush()        
        
        if key == "q":
            print("\nStopping program...")
            for motor_id in ids:
                dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
                if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                    print(f"❌ Main: Torque Disable 실패 (ID {motor_id})")
                else:
                    #print(f"✅ Main: Torque Disable 성공 (ID {motor_id})")
                    sys.stdout.write(f"✅ Main: Torque Disable 성공 (ID {motor_id})")
                    sys.stdout.flush()
            self.exit_requested = True
            
        elif key == "ad":
            #print("좌우 키 동시 입력 - 정지")
            set_velocity(ids, 0)
        elif key == "a":
            set_velocity(ids, 100)
        elif key == "d":
            set_velocity(ids, -100)
        elif key == "s":
            #print("정지 명령 실행")
            set_velocity(ids, 0)
        
    def publish_position(self):
        for dxl_id in ids:
            dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
            msg = Int32()
            msg.data = dxl_present_position
            self.publisher.publish(msg)
            #print(f"Published Position (ID {dxl_id}): {dxl_present_position}")

def main(args=None):
    rclpy.init(args=args)
    node = JetsonNode()
    print('✅ Main : Node started, waiting for key inputs...')

    while rclpy.ok() and not node.exit_requested:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    portHandler.closePort()
    rclpy.shutdown()
    print("\n✅ Main : Program successfully finished")

if __name__ == '__main__':
    main()
