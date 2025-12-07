# aruco_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(String, '/aruco/pos', 10)
        self.annotated_pub = self.create_publisher(Image, '/camera/annotated', 10)
        self.bridge = CvBridge()

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        corners, ids, rejected = detector.detectMarkers(gray)

        h, w = img.shape[:2]
        center_y = h // 2

        status = String()
        status.data = "none"

        if ids is not None and len(ids) > 0:
            c = corners[0][0]
            # środek markera
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            # rysowanie
            cv2.polylines(img, [c.astype(int)], True, (0, 255, 0), 2)
            cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)
            cv2.line(img, (0, center_y), (w, center_y), (255, 0, 0), 1)

            if cy < center_y:
                status.data = "above"
            else:
                status.data = "below"
        else:
            status.data = "none"

        self.pub.publish(status)

        annotated_msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        self.annotated_pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
