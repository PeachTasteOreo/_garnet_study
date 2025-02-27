import rclpy
from ros_communication import ROSCommunication
from key_listener import KeyListener
from motor_controller import MotorController

class JetsonNode:
    def __init__(self):
        rclpy.init()
        self.motor = MotorController()

        self.ros_node = ROSCommunication(self.process_keys)
        self.key_listener = KeyListener(self.process_keys)

    def process_keys(self, keys):
        key = "".join(sorted(keys))  # 키 목록을 정렬된 문자열로 변환
        print(f"Received key input: {key}")

        if key == "q":
            print("프로그램 종료 준비...")
            self.motor.stop()
            rclpy.shutdown()

        elif key == "ad":
            print("좌우 키 동시 입력 - 정지")
            self.motor.set_velocity(0)

        elif key == "a":
            self.motor.set_velocity(100)

        elif key == "d":
            self.motor.set_velocity(-100)

        elif key == "s":
            print("정지 명령 실행")
            self.motor.set_velocity(0)

        else:
            print("Unknown command: ", key)

    def run(self):
        print("노드 시작, 키 입력 대기 중...")
        rclpy.spin(self.ros_node)

    def shutdown(self):
        del self.key_listener
        del self.motor
        del self.ros_node
        print("프로그램 종료 완료.")

def main():
    node = JetsonNode()
    try:
        node.run()
    except KeyboardInterrupt:
        node.shutdown()

if __name__ == '__main__':
    main()

