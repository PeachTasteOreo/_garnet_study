import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from pynput import keyboard
from _settings2 import *


# Dynamixel 설정
ids = [3,4] # 한개만 입력해도 단일 변수로 받아들임
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


# Dynamixel 포트 및 모드 설정
if not start():
    print("🚨 포트를 열 수 없어 프로그램을 종료합니다.")
    exit(1)

if not mode(ids, velocity_control):
    print("🚨 모드 설정 실패. 프로그램을 종료합니다.")
    exit(1)

# 최종 포트 상태 확인
if not portHandler.is_open:
    print("🚨 Main : 포트가 닫혀 있어 다시 엽니다.")
    portHandler.openPort()

if not portHandler.is_open:
    print("🚨 포트를 다시 열려고 했지만 실패했습니다. 프로그램을 종료합니다.")
    exit(1)

# # 토크 활성화
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
# if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
#     print("❌ Main : 토크 설정 오류")
# else:
#     print("✅ Main : Dynamixel 연결 성공!")


for motor_id in ids:
    result, error = packetHandler.write1ByteTxRx(portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if result != COMM_SUCCESS:
        print(f"❌ Torque Disable 실패 (ID {motor_id}): {packetHandler.getTxRxResult(result)}")
        continue
    if error != 0:
        print(f"❌ Torque Disable 에러 (ID {motor_id}): {packetHandler.getRxPacketError(error)}")
        continue



#set_velocity(4,100)
#print("================")

# set_position([4],500)
# print("===========")


# time.sleep(10)

class JetsonNode(Node):
    def __init__(self):
        super().__init__('jetson_node')
        self.exit_requested = False
        self.publisher = self.create_publisher(Int32, 'position_topic', 10)
        self.subscription = self.create_subscription(String, 'key_input_topic', self.process_keys, 10)
        self.timer = self.create_timer(0.5, self.publish_position)
        self.pressed_keys = set()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
    
    def on_press(self, key):
        try:
            key = key.char.lower()  # 일반 문자 키
        except AttributeError:
            key = key.name  # 특수 키
            print("on press error : ", key)

        self.pressed_keys.add(key)
        self.process_keys()

    def on_release(self, key):
        try:
            key = key.char.lower()
        except AttributeError:
            key = key.name
            print("on release error : ", key)

        if key in self.pressed_keys:
            self.pressed_keys.remove(key)
        self.process_keys()

    def process_keys(self,msg):
        key = msg.data.strip()  # 공백 제거
        print(f"received key input : {key}")
        
        if key == "q":
            print("\nstart to stop program ... ")
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ids, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print("토크 비활성화 오류")
            self.exit_requested = True
            
        elif key == "ad":
            print("좌우 키 동시 입력 - 정지")
            set_velocity(ids,0)
        elif key == "a":
            set_velocity(ids,100)
        elif key == "d":
            set_velocity(ids,-100)
        elif key == "s":
            print("정지 명령 실행")
            set_velocity(ids,0)
            
        else :
            pass
            # print("\npass ... return : ", key)
            

    def publish_position(self):
        for dxl_id in ids : 
            dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
            msg = Int32()
            msg.data = dxl_present_position
            self.publisher.publish(msg)
            #print(f"발행한 현재 위치: {dxl_present_position}")

def main(args=None):
    rclpy.init(args=args)
    node = JetsonNode()

    print('✅ Main : Node started, waiting for key inputs...')

    while rclpy.ok() and not node.exit_requested:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    portHandler.closePort()
    rclpy.shutdown()
    print("\n✅ Main : program successfully finished")

if __name__ == '__main__':
    main()
