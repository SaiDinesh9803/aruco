import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ArucoDetector_node(Node):
    def __init__(self):
        super().__init__('cv_bridge_example')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
    def image_callback(self, msg):
        self.get_logger().info(msg.header)
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg,'passthrough')
        except CvBridgeError as e:
            self.get_logger().info("Cvbridge Error : {0}".format(e))
            
        self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        
        
        cv2.namedWindow("ImageWindow", 1)
        cv2.imshow("Image Window", self.gray)
        cv2.waitKey(1)        
        
        
def main(args = None):
    rclpy.init(args=args)
    
    cv_bridge_example = ArucoDetector_node()
    
    rclpy.spin(cv_bridge_example)
    
    cv_bridge_example.destroy_node()
    
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()