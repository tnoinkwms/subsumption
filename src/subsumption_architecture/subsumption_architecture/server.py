import rclpy
from rclpy.node import Node
from std_msgs import msg

import numpy as np
import pyalter
from pyalter import Alter3
from pyalter.constants import ALTER3_AXIS_NUM

class AlterServer(Node):

    def __init__(self, simulator=False, get_axis_period=0.01, report_period=0.5):
        super().__init__('alterserver')
        
        if simulator:
            #pyalter.safety_mode = False 
            #self.alter = Alter3("simulator", simulator_address="192.168.0.12", simulator_port=11000)
            pyalter.safety_mode = False 
            self.alter = Alter3("simulator", simulator_address="127.0.0.1", simulator_port=11000)
            
        else:
            self.alter = Alter3("serial", serial_port="/dev/ttyUSB0")

        self.__last_set_axis_values = [None] * ALTER3_AXIS_NUM
        self.__set_axis_subscription = self.create_subscription(
            msg.UInt8MultiArray,
            'set_axis',
            self.__set_axis_callback,
            10)
        
        self.__get_axis_publisher = self.create_publisher(
            msg.UInt8MultiArray, 
            'get_axis', 
            10)

        self._get_axis_timer = self.create_timer(get_axis_period, self.__get_axis)
        self._report_timer = self.create_timer(report_period, self.__report)

    def __set_axis_callback(self, msg):
        for i in range(0, len(msg.data), 2):
            axis = msg.data[i]
            val = msg.data[i+1]
            self.alter.set_axes(axis, val)
            self.__last_set_axis_values[axis-1] = val
    
    def __get_axis(self):
        self.__get_axis_values = self.alter.get_axes()
        m = msg.UInt8MultiArray()
        # m.data = self.__get_axis_values
        m.data = [(v if v is not None else 127) for v in self.__get_axis_values]
        self.__get_axis_publisher.publish(m)

    def __report(self):
        assert len(self.__get_axis_values) == ALTER3_AXIS_NUM
        print('-----------------------------------')
        for i, (sa, ga) in enumerate(zip(self.__last_set_axis_values, self.__get_axis_values)):
            sa_str = f'{sa:3}' if sa is not None else 'nan'
            ga_str = f'{ga:3}' if ga is not None else 'nan'
            print(f'#{i+1:02} {sa_str}/{ga_str}  ', end='')
            if (i+1) % 4 == 0 or i == ALTER3_AXIS_NUM-1:
                print()


def main_sim(args=None):
    rclpy.init(args=args)
    server = AlterServer(simulator=True)
    rclpy.spin(server)
    server.destroy_node() # Destroy the node explicitly (it will be done automatically)
    rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    server = AlterServer()
    rclpy.spin(server)
    server.destroy_node() # Destroy the node explicitly (it will be done automatically)
    rclpy.shutdown()



if __name__ == '__main__':
    main()
    # main_sim()
