import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt64MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np
from demo_superglue import *

sp = CreateSuperGlue()

def detect_mortion(mkpts0,mkpts1):
    d =10
    mkpts0x  = np.array([i[0] for i in mkpts0])
    mkpts0y  = np.array([i[1] for i in mkpts0])
    mkpts1x  = np.array([j[0] for j in mkpts1])
    mkpts1y  = np.array([j[1] for j in mkpts1])
    #これ自分の運動を捉えているけど，mkpts0とmkpts1の差の平均をとって，そこから飛んでいるものを見つけてcx,cyにすればいいのでは？
    deltax = mkpts0x - mkpts1x
    deltay = mkpts0y - mkpts1y
    xmean = np.mean(deltax)
    ymean = np.mean(deltay)
    movex = []
    movey = []
    for i in range(len(deltax)):
        if ((ymean-d>= deltay[i]) or (ymean+d <= deltay[i]))and((xmean-d>= deltax[i]) or (xmean+d <= deltax[i])):
            if (abs(deltax[i]<30)) or (abs(deltay[i])<30):
                movex.append(mkpts1x[i])
                movey.append(mkpts1y[i])
            else:
                pass
        else:
            #print("pass")
            pass
    #多分このmeanでうまくいってない。
    cx = np.mean(movex)
    cy = np.mean(movey)
    if np.isnan(cx) or cx == 0.0:
        return 320,240
    else:
        #print(f"movex is {movex}")
        print(len(movex),len(mkpts0))
        print(f"cx cy is {cx},{cy}")
        return cx,cy

class MotionDetector(Node):
    def __init__(self):
        super().__init__('mortin_detector')
        self.publisher = self.create_pubisher(UInt64MultiArray,'value_now',10)
        self.timer = self.create_timer(0.3, self.mortion)
    def mortion(self):
        mkpts0, mkpts1, vs = sp.apply()
        cx,cy = detect_mortion(mkpts0,mkpts1)
        send_list = np.array([320,cx,300,cy],dtype='uint64')
        send_msg = UInt64MultiArray(data = send_list)
        self.get_logger().info(f"moving_obj {send_msg.data}")
        self.publisher.publish(send_msg)

def main(args = None):
    rclpy.init(args=args)
    face_detector = MotionDetector()
    rclpy.spin(face_detector)

if __name__ == '__main__':
    main()

