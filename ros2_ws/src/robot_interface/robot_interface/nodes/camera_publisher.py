import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.cap = cv2.VideoCapture(0)  # indeks 0 - domyślna kamera

        # ustaw większą rozdzielczość (np. Full HD 1920x1080)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  # szerokość
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)  # wysokość

        if not self.cap.isOpened():
            self.get_logger().error('Nie można otworzyć kamery (0). Sprawdź podłączenie.')

        self.bridge = CvBridge()
        timer_period = 0.01  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Brak klatki z kamery')
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()
