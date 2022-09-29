import rclpy
from rclpy.node import Node
from std_msgs import msg

import numpy as np
import time

class PIDnode(Node):
    
    def __init__(self, time_period = 0.1):
        super().__init__('pid_node')
        #subするのは制御する値、たとえば顔の画面上のピクセル
        #[goal1,now1,goal2,now2]
        self.subscribe = self.create_subscription(msg.UInt64MultiArray,'value_now',self.listener_callback,10)
        self.subscribe_axis = self.create_subscription(msg.UInt8MultiArray,'get_axis',self.axis_callback, 10)
        self.publish = self.create_publisher(msg.UInt8MultiArray,'set_axis',10)

        self.timer = self.create_timer(time_period, self.timer_callback)
        """
        self.declare_parameters(
            namespace = '',
            parameters = [
                ('axis_1',0),
                ('axis_2',0),
                ('Kp_1',0.0),
                ('Kp_2',0.0),
                ('Ki_1',0.0),
                ('Ki_2',0.0),
                ('Kd_1',0.0),
                ('Kd_2',0.0)
            ])
        
        self.timr = self.create_timer(time_period,self.parameter)
        """
        self.axis_1  = 11
        self.Kp_1 = 0.0231
        self.Ki_1 = 3.948e-6
        self.Kd_1 = 1.119e-4

    def axis_callback(self,msg):
        self.value_1 = msg.data[self.axis_1-1]
    
    def listener_callback(self,msg):
        try:
            self.d1_0 = self.d1
            self.dt = time.time() - self.time
        except:
            self.d1_0 = 0
            self.dt = 1

        #self.axis1 = msg.data[0]
        #self.axis2 = msg.data[3]
        self.axis1 = self.axis_1

        if msg.data[1] > msg.data[0]:
            self.d1 = msg.data[1] - 640
        else:
            self.d1 = msg.data[1] - 0

        self.P_1 = self.Kp_1*self.d1

        self.I_1 = self.Ki_1*(self.d1_0 + self.d1)*self.dt

        self.D_1 = self.Kd_1*(self.d1 -self.d1_0)/self.dt

        self.dvalue_1 = self.P_1 + self.I_1 + self.D_1

        self.time = time.time()

        try:
            self.value_1 += self.dvalue_1

            self.value_1 = set_range(self.value_1)

            self.values = [self.axis_1, self.value_1]
            print(self.values)
        except AttributeError:
            pass
    
    def timer_callback(self):
        try:
            send_list = np.array(self.values, dtype='uint8')
            #print(send_list)
            send_msg = msg.UInt8MultiArray(data=send_list)
            self.get_logger().info(f"avoid {send_msg.data}")
            self.publish.publish(send_msg)
        except AttributeError:
            pass
        

def set_range(value):
    if value <= 0:
        value = 10
    elif value >= 255:
        value = 250
    return value

def main(args = None):
    rclpy.init(args = args)
    pid = PIDnode()
    rclpy.spin(pid)
    

if __name__ == '__main__':
    main()
