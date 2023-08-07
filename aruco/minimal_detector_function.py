import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError


class ArucoDetector_node(Node):
    def __init__(self):
        super().__init__('cv_bridge_example')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.Detected_ArUco_markers = {}
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def detect_ArUco(self, img):
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids , _ = self.detector.detectMarkers(self.gray)
        i = 0
        
        try:
            for id in self.ids:
                for id_number in id:
                    self.Detected_ArUco_markers[id_number] = self.corners[i][0]
                    
        except TypeError:
            print("No ArUco in front of me")
            
        i += 1
        return self.Detected_ArUco_markers
    
    
    def mark_ArUco(self, img, Detected_Aruco_markers):
        self.Detected_ArUco_markers = Detected_Aruco_markers
        self.img = img
        
        self.ids = self.Detected_ArUco_markers.keys()
        print(self.Detected_ArUco_markers)
        self.centre_aruco = {}
        self.top_centre = {}
        
        try:
            for id in self.ids:
                self.corners = self.Detected_ArUco_markers[id]
                for i in range (0, 4):
                    cv2.circle(self.img, (int(self.corners[i][0]), int(self.corners[i][1])), 5, (0,0,255), -1)
                self.centre_aruco[id] = (self.corners[0]+self.corners[1]+self.corners[2]+self.corners[3])/4
                self.top_centre[id] = (self.corners[0]+self.corners[1])/2
                cv2.line(self.img, (int(self.centre_aruco[id][0]), int(self.centre_aruco[id][1])),
	    		(int(self.top_centre[id][0]), int(self.top_centre[id][1])), (255, 0, 0), 5)
                
        except TypeError:
            print("No aruco in front of me")
        return self.img 
            
        
    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'passthrough')
        except CvBridgeError:
            self.get_logger().info("Cvbridge Error")
            
        self.Detected_ArUco_markers = self.detect_ArUco(self.cv_image)
        
        self.img = self.mark_ArUco(self.cv_image, self.Detected_ArUco_markers)
        
        cv2.namedWindow("ImageWindow", 1)
        cv2.imshow("Image Window", self.img)
        cv2.waitKey(1)        
        
        
def main(args = None):
    rclpy.init(args=args)
    
    cv_bridge_example = ArucoDetector_node()
    
    rclpy.spin(cv_bridge_example)
    
    cv_bridge_example.destroy_node()
    
    rclpy.shutdown()
    
    
if __name__ == '__main__':
    main()