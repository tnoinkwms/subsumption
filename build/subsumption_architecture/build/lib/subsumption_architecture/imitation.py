import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt64MultiArray
from std_msgs.msg import UInt8MultiArray


import numpy as np

class Imitation(Node):
    def __init__(self):
        super().__init__('imitation')
        self.subscription = self.create_subscription(UInt64MultiArray,'hand_pos',self.listener_callback,10)
        self.subscription

        self.publisher = self.create_publisher(UInt8MultiArray,'set_axis',10)
        self.timer = self.create_timer(0.1, self.publish_axis)
        self.switch = 0

    def listener_callback(self,msg):
        if msg.data[2] <200:
            self.switch = 1
            if msg.data[0] == 0: #Left
                 self.send_list = [16,255,17,255,18,255,19,255]
            elif msg.data[0] == 1: #Right
                self.send_list = [29,255,30,255,31,255,32,255]
            else:
                self.switch = 0
        else:
            self.switch = 0
    def publish_axis(self):
        try:
            if self.switch == 1:
                self.send_list = np.array(self.send_list,dtype='uint8')
                #print(self.send_list)
                send_msg = UInt8MultiArray(data = self.send_list)
                self.publisher.publish(send_msg)
                self.get_logger().info(f"imitation {send_msg.data}")
                self.switch = 0
            else:
                pass
        except AttributeError:
            print("err")
            pass

def main(args = None):
    rclpy.init(args = args)
    imitation = Imitation()
    rclpy.spin(imitation)

if __name__ == '__main__':
    main()
