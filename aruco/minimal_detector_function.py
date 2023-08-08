import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np





class ArucoDetector_node(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()
        self.marker_size = 25

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.image_callback,
            10
        )  
        
        
        self.camera_matrix = np.array([[494.30114815 , 0 , 322.46760411] ,[0 , 493.52032388 , 240.6078525 ] , [0 , 0 , 1]])
        self.camera_distortion = np.array([[ 9.71518504e-03 , 1.17503633e+00 , -1.27224644e-03 , 1.90922858e-03 , -4.38978499e+00]])

        
        
        self.Detected_ArUco_markers = {}
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.object_corners = np.array([
            [-self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, self.marker_size / 2, 0],
            [self.marker_size / 2, -self.marker_size / 2, 0],
            [-self.marker_size / 2, -self.marker_size / 2, 0]
            ])
        self.pose_publisher = self.create_publisher(PoseStamped, '/estimated_Pose', 10)
        

    def detect_ArUco(self, img):
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        self.corners, self.ids , _ = self.detector.detectMarkers(self.gray)
        i = 0
        
        try:
            for id in self.ids:
                for id_number in id:
                    self.Detected_ArUco_markers[id_number] = self.corners[i][0]
                # print("Length of Detected Markers : ", len(self.Detected_ArUco_markers))
                    
        except TypeError:
            print("No ArUco in front of me")
        i += 1
        return self.Detected_ArUco_markers
    
    
    def mark_ArUco(self, img, Detected_Aruco_markers):
        self.Detected_ArUco_markers = Detected_Aruco_markers
        self.img = img     
        self.ids = self.Detected_ArUco_markers.keys()
        #print(self.Detected_ArUco_markers)
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
    
    def estimate_pose(self, obj_corners, corners, camera_matrix, dist_coeffs):
            self.corners = corners
            self.camera_matrix = camera_matrix
            self.camera_distortion = dist_coeffs
            self.obj_points = obj_corners
            
            success, rvec, tvec = cv2.solvePnP(self.obj_points, self.corners, self.camera_matrix, self.camera_distortion)
            
            if success:
                self.rvec = rvec
                self.translation_vector = tvec
                
            return (self.rvec, self.translation_vector)
        
    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'passthrough')
        except CvBridgeError:
            self.get_logger().info("Cvbridge Error")
            
        self.Detected_ArUco_markers = self.detect_ArUco(self.cv_image)
        
        self.img = self.mark_ArUco(self.cv_image, self.Detected_ArUco_markers)
        
        if len(self.Detected_ArUco_markers) > 0:
            
            _, detected_corners = next(iter(self.Detected_ArUco_markers.items()))
        
            self.estimated_val = self.estimate_pose(self.object_corners, detected_corners, self.camera_matrix, self.camera_distortion)
            
            if self.estimated_val[0] is not None and self.estimated_val[1] is not None:
                pose_msg = PoseStamped()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = 'estimated_pose'
                # The below estimated_val[1] gives 3X1 vector
                pose_msg.pose.position.x = float(self.estimated_val[1][0])
                pose_msg.pose.position.y = float(self.estimated_val[1][1])
                pose_msg.pose.position.z = float(self.estimated_val[1][2])
                # The estimated_value[0] gives us a 3 * 1 rotation vector
                pose_msg.pose.orientation.x = float(self.estimated_val[0][0])
                pose_msg.pose.orientation.y = float(self.estimated_val[0][1])
                pose_msg.pose.orientation.z = float(self.estimated_val[0][2])
                
                print((self.estimated_val[0][1]))

                
                # print(self.rotation_matrix)
                self.pose_publisher.publish(pose_msg)
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