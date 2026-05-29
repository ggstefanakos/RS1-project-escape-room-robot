#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        # O Bridge μετατρέπει τα μηνύματα του ROS σε εικόνες OpenCV
        self.bridge = CvBridge()
        
        # Κάνουμε subscribe στο topic της κάμερας. 
        # (Όταν πάτε στο ρομπότ, απλά αλλάζετε το '/image_raw' στο topic του MyAGV)
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
            
        # Ρυθμίσεις του ArUco Dictionary (Εξαρτάται τι έχετε εκτυπώσει στο εργαστήριο. Το 6X6 είναι κλασικό)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.get_logger().info("ArUco Detector Node has been started.")

    def image_callback(self, msg):
        try:
            # Μετατροπή ROS Image σε OpenCV (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Εντοπισμός των ArUco Markers
        corners, ids, rejected = self.detector.detectMarkers(cv_image)

        if ids is not None:
            self.get_logger().info(f"Detected ArUco IDs: {ids.flatten().tolist()}")
            
            # (Προαιρετικό) Ζωγραφίζουμε ένα πλαίσιο γύρω από το marker για επιβεβαίωση
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        
        # Δείχνουμε το παράθυρο με την κάμερα (χρήσιμο για debugging στο λάπτοπ)
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
        # Κλείνουμε καθαρά το παράθυρο της OpenCV
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()