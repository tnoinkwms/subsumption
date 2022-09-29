import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import time
import numpy as np
from pythonosc import osc_server
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

axis = np.arange(1,42)

class PublishFromOSC(Node):
    def __init__(self,time_period=0.1):
        super().__init__('osc_publisher')
        ip = '10.1.2.10'
        port = 10000
        qos_profile = 10
        self.publisher = self.create_publisher(UInt8MultiArray,'set_axis',qos_profile)
        dispatcher = Dispatcher()
        dispatcher.map('/T7', self.OSC_callback)
        server = BlockingOSCUDPServer((ip,port),dispatcher)
        server.serve_forever()
        #self.timer = self.create_timer(time_period,self.timer_callback)
        
    def OSC_callback(self ,*args):
        self.send_list = []
        osc_get_list =list(args)[1:]
        #print(osc_get_list)
        for i in range(len(osc_get_list)*3):
            if np.any(self.send_list == axis) is not True:
                self.send_list = np.append(self.send_list,axis[i])
                self.send_list = np.append(self.send_list,osc_get_list[i%12])
        self.send_list = np.array(self.send_list,dtype='uint8')
        #print(self.send_list)
        send_msg = UInt8MultiArray(data = self.send_list)
        self.get_logger().info(f"alife_engine {send_msg.data}")
        self.publisher.publish(send_msg)
    

def main(args = None):
    rclpy.init(args=args)
    publisher = PublishFromOSC()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
