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

def callback(msg):
    #print("receving : ", msg.data)
    print(msg.data)
    

   
        
        #print(subscription)
        #print("answer : ",msg)
    

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
    
    
    

if __name__ == '__main__':
    rclpy.init()
    node = Node('subscriber_node')
    subscription = node.create_subscription(Int32, 'my_topic', callback, 10)
    #node.create_subscription(String, 'my_topic', callback, 10)
    
    
    try:
        while rclpy.ok():
            
            print('***********')
            rclpy.spin(node) 
             
    except KeyboardInterrupt:
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_lead, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("토크 비활성화 오류")

        # Close port
        portHandler.closePort()

        pass
