import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import numpy as np


class RandomAxis(Node):
    def __init__(self,time_period=0.1):
        super().__init__('random_axis')
        self.publish = self.create_publisher(UInt8MultiArray,'set_axis',10)
        self.timer = self.create_timer(time_period,self.timer_callback)
    
    def timer_callback(self):
        self.send_list = []
        num_axis = np.random.randint(0,42)
        for i in range(num_axis):
            axis = np.random.randint(1,42)
            value = np.random.randint(10,240)
            if np.any(self.send_list == axis) is not True:
                self.send_list = np.append(self.send_list,axis)
                self.send_list = np.append(self.send_list,value)
        self.send_list = np.array(self.send_list,dtype='uint8')
        #print(self.send_list)
        send_msg = UInt8MultiArray(data = self.send_list)
        self.publish.publish(send_msg)
        self.get_logger().info(f"random_axis {send_msg.data}")

def main(args = None):
    rclpy.init(args = args)
    randomaxis = RandomAxis()
    rclpy.spin(randomaxis)

if __name__ == '__main__':
    main()
