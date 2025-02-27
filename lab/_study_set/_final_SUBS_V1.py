import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

from pynput import keyboard

from _settings import *


import _settings

# Dynamixel 설정
dxl_id = 4
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


start(portHandler, BAUDRATE)
""" mode(portHandler, packetHandler, dxl_id, POSITION_CONTROL)

time.sleep(5)

while True :
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_POSITION, 2000)
    pre, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    
    print(pre)
    
    if not abs(100-pre) > 10 :
        break
    time.sleep(1) """

mode(portHandler, packetHandler, dxl_id, velocity_control)

# follow 셀 토크 온
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 설정 오류")
else:
    print("Dynamixel 연결 성공!")

def _settings_v(velocity):
    packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_GOAL_VELOCITY, velocity)

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
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print("토크 비활성화 오류")
            self.exit_requested = True
            
        elif key == "ad":
            print("좌우 키 동시 입력 - 정지")
            _settings_v(0)
        elif key == "a":
            _settings_v(100)
        elif key == "d":
            _settings_v(-100)
        elif key == "s":
            print("정지 명령 실행")
            _settings_v(0)
            
        else :
            print("\npass ... return : ", key)
            

    def publish_position(self):
        dxl_present_position, _, _ = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
        msg = Int32()
        msg.data = dxl_present_position
        self.publisher.publish(msg)
        print(f"발행한 현재 위치: {dxl_present_position}")

def main(args=None):
    rclpy.init(args=args)
    node = JetsonNode()

    print('Node started, waiting for key inputs...')

    while rclpy.ok() and not node.exit_requested:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    portHandler.closePort()
    rclpy.shutdown()
    print("\nprogram successfully finished")

if __name__ == '__main__':
    main()
