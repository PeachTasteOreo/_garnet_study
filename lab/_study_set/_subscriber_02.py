import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String

import time
import sys
import tty
import termios
import select
from dynamixel_sdk import * # Uses Dynamixel SDK library


from _settings import *
import _settings

dxl_lead = 3
dxl_follow = 4


# PortHandler 및 PacketHandler 객체 초기화
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


_settings.start(portHandler, BAUDRATE)

#_settings.mode(portHandler, packetHandler, dxl_lead, position_control) # 돌리는 셀 
_settings.mode(portHandler, packetHandler, dxl_follow, velocity_control) # 따라가는 셀

# lead 셀 토크 온
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_lead, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

# follow 셀 토크 오프
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
    print("토크 설정 오류")
else:
    print(" 1 : Dynamixel has been successfully connected")
    
    

def callback(msg):
    print(msg.data)
    key = msg.data
    print(key)
    print(type(key))
    print("aaa")
    
    if key == chr(27):  # ESC 키가 눌리면 종료
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        print("Exiting program...")
        
    
    elif key == "A" or key == "a":  # 왼쪽 화살표 키
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 100)
        dxl_present_position_7, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"\n7번셀 현재 위치 : {dxl_present_position_7} \n")
        print("Left arrow key pressed!")
    
    elif key == "D" or key == "d":  # 오른쪽 화살표 키
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, -100)
        dxl_present_position_7, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_follow, ADDR_PRESENT_POSITION)
        print(f"\n7번셀 현재 위치 : {dxl_present_position_7} \n")
        print("Right arrow key pressed!")
    
    elif key == "S" or key == "s":  # stop
        print("stop")
        # Close port
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_follow, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_follow, ADDR_GOAL_VELOCITY, 0)
        portHandler.closePort()
    
    else:
        print(" ... ")
        
        

    #time.sleep(0.01)  # 0.01초 대기
    
    
    
    
    
    
    
if __name__ == '__main__':
    rclpy.init()
    node = Node('subscriber_node')
    #subscription = node.create_subscription(Int32, 'my_topic', callback, 10)
    subscription = node.create_subscription(String, 'my_topic', callback, 10)
    
    try:
        while rclpy.ok():
            print('******')
            rclpy.spin(node) 
             
    except KeyboardInterrupt:
        pass
