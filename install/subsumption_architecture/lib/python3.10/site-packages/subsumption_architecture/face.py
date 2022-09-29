import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt64MultiArray
import cv2
from cv_bridge import CvBridge
import numpy as np

class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')
        video_qos = rclpy.qos.QoSProfile(depth=10)
        video_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, video_qos)
        self.publisher = self.create_publisher(UInt64MultiArray,'value_now',10)
        self.subscription
        self.br = CvBridge()
        #change here
        #caspath = '/home/alter3/.pyenv/versions/3.10.6/lib/python3.10/site-packages/cv2/data/haarcascade_frontalface_default.xml'
        caspath = '/home/tnoin/.local/lib/python3.10/site-packages/cv2/data/haarcascade_frontalface_default.xml'
        self.faceCascade = cv2.CascadeClassifier(caspath)
    
    def listener_callback(self,data):
        current_frame = self.br.imgmsg_to_cv2(data)
        gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        self.faces = self.faceCascade.detectMultiScale(
            gray,
            scaleFactor = 1.2,
            minNeighbors = 3,
            minSize = (30,30)
        )

        if len(self.faces) != 0:
            self.face = self.faces[self.faces[:, 2].argmax()]
            self.get_logger().info("detect Face :)")
            cv2.rectangle(current_frame,(self.face[0],self.face[1]),(self.face[2]+self.face[0],self.face[3]+self.face[1]),(255,0,0),2)
            send_list = np.array([320, self.face[0], 240, self.face[1]],dtype='uint64')
            send_msg = UInt64MultiArray(data = send_list)
            self.publisher.publish(send_msg)

        cv2.imshow("video frame",current_frame)
        cv2.waitKey(1)

def main(args = None):
    rclpy.init(args=args)
    face_detector = FaceDetector()
    rclpy.spin(face_detector)

if __name__ == '__main__':
    main()
