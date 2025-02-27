import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String


def callback(msg):
    print(msg.data)
    print("aaa")
    
    
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
