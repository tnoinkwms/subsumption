import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import numpy as np

class PeriodicAxis(Node):
    def __init__(self,time_period = 0.1):
        super().__init__('periodic_acxis')
        self.i = 0
        self.publish = self.create_publisher(UInt8MultiArray,'set_axis',10)
        self.timer = self.create_timer(time_period, self.timer_callback)
        self.num_axis = np.random.randint(0,42)
        self.axis_list = []
        for i in range(self.num_axis):
            axis = np.random.randint(0,42)
            if np.any(self.axis_list == axis) is not True:
                self.axis_list = np.append(self.axis_list,axis)

    def timer_callback(self):
        self.send_list = []
        self.i += 20
        value = set_range(120 - np.sin(self.i*np.pi/360)*120+10)
        for i in range(len(self.axis_list)):
            self.send_list = np.append(self.send_list,self.axis_list[i])
            self.send_list = np.append(self.send_list,value)
        
        self.send_list = np.array(self.send_list,dtype = 'uint8')
        #print(self.send_list)
        send_msg = UInt8MultiArray(data = self.send_list)
        self.get_logger().info(f"periodic_axis {send_msg.data}")
        self.publish.publish(send_msg)
    
def set_range(value):
    if value >=210:
        value = 210
    elif value <=30:
        value= 30
    return value

def main(args =  None):
    rclpy.init(args = args)
    periodic_axis = PeriodicAxis()
    rclpy.spin(periodic_axis)

if __name__ == 'main':
    main()
            

        
