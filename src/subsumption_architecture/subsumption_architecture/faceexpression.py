import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray

import numpy as np

class FaceExpression(Node):
    def __init__(self):
        super().__init__('face_expression')
        self.publish = self.create_publisher(UInt8MultiArray,'set_axis',10)
        
        self.timer = self.create_timer(1,self.timer_callback)
    
    def timer_callback(self):
        self.mode = np.random.randint(0,4)

        if self.mode == 0:
            print("None :|")
            send_list = [1,10, 4,10, 5,10, 6,10, 8,10]
        elif self.mode == 1:
            print("smile :)")
            send_list = [1,180, 4,180, 5,200, 6,200, 8,10]
        elif self.mode == 2:
            print("anger :(")
            send_list = [1,10, 4,10, 5,10, 6,10, 8, 10]
        elif self.mode == 3:
            print("surprise :o")
            send_list = [1,10, 4,10, 5, 100, 6,100, 7,200, 8,200]
        else:
            self.get_logger().info("err")
            send_list = [42, 125]
        
        send_list = np.array(send_list,dtype = 'uint8')
        send_msg = UInt8MultiArray(data = send_list)
        self.publish.publish(send_msg)

def main(args= None):
    rclpy.init(args=args)
    face_expression = FaceExpression()
    rclpy.spin(face_expression)

if __name__ == '__main__':
    main()
