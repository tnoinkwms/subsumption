import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt64MultiArray

import numpy as np

class HandGaze(Node):
    def __init__(self):
        super().__init__('hand_gaze')
        self.subscription = self.create_subscription(UInt64MultiArray,'hand_pos',self.listener_callback,10)
        self.subscription

        self.publisher = self.create_publisher(UInt64MultiArray,'value_now',10)
        self.timer = self.create_timer(0.1, self.publish_axis)
        self.switch = 0
    
    def listener_callback(self,msg):
        self.send_list = [320,msg.data[1],240,msg.data[2]]
        self.switch = 1

    def publish_axis(self):
        try:
            if self.switch == 1:
                self.send_list = np.array(self.send_list,dtype = 'uint8')
                #print(self.send_list)
                send_msg = UInt64MultiArray(data = self.send_list)
                self.publisher.publish(send_msg)
                self.get_logger().info(f"hand_gaze {send_msg.data}")
                self.switch = 0
            else:
                pass
        except AttributeError:
            pass

def main(args =  None):
    rclpy.init(args = args)
    hand_gaze = HandGaze()
    rclpy.spin(hand_gaze)

if __name__ == '__main__':
    main()
