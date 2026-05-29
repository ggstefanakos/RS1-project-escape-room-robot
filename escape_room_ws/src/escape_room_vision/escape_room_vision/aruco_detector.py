 #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
            
        # Συμβατότητα με παλαιότερες εκδόσεις OpenCV (πριν την 4.7)
        # Ελέγχουμε ποια μέθοδος υπάρχει διαθέσιμη στη βιβλιοθήκη σας
        if hasattr(cv2.aruco, 'Dictionary_get'):
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info("ArUco Detector Node has been started (Legacy OpenCV API).")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Εκτέλεση ανίχνευσης με τον παλιό τρόπο της OpenCV
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            self.get_logger().info(f"Detected ArUco IDs: {ids.flatten().tolist()}")
            
            # Ζωγραφίζουμε το πλαίσιο
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        
        # Εμφάνιση της εικόνας
        cv2.imshow("Escape Room Camera", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()