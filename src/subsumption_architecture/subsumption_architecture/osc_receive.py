import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import time
import numpy as np
from pythonosc import osc_server
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class PublishFromOSC(Node):

    def __init__(self):
        super().__init__('osc_publisher')
        ip = '10.1.2.10'
        port = 10000
        qos_profile = 10
        self.publisher = self.create_publisher(UInt8MultiArray,'set_axis',qos_profile)
        #self.publish_supression = self.create_publisher(UInt8MultiArray,'spression',10,self.supression_callback,10)
        self.sup_list = [0,0]
        dispatcher = Dispatcher()
        dispatcher.map('/moveaxis',self.OSC_callback)
        server = BlockingOSCUDPServer((ip,port),dispatcher)
        server.serve_forever()

    def OSC_callback(self,*args):
        osc_get_list =list(args)[1:]
        osc_list = [osc_get_list[0],osc_get_list[1]]
        '''
        osc_list = list(args)[1:]
        osc_list.insert(0,0)
        osc_list = np.array([ n for i,n in enumerate(osc_list) if i%3 != 0 ],dtype='uint8')
        
         # supression list
        for axis in self.sup_list:
            if axis in osc_list[::2]:
                del osc_list[osc_list[::2].index(axis):osc_list[::2].index(axis)+2]
        '''
        pub_list = UInt8MultiArray(data=osc_list)
        #self.get_logger().info(f'recieve values {osc_list}')
        #self.get_logger().info(f'{time.time()-start}')
        self.publisher.publish(pub_list)
    
    def supression_callback(self, msg):
        self.sup_list = msg.data[::2]

def main(args = None):
    rclpy.init(args=args)
    publisher = PublishFromOSC()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
