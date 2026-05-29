#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.bridge = CvBridge()
        
        # --- Subscribers & Publishers ---
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
            
        self.marker_publisher = self.create_publisher(
            Marker,
            '/marker',
            10)
            
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- Ρυθμίσεις ArUco (Legacy API) ---
        if hasattr(cv2.aruco, 'Dictionary_get'):
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()

        # --- Παράμετροι Κάμερας (DUMMY για την Web Camera) ---
        # ΠΡΟΣΟΧΗ: Αυτά θα πρέπει να αλλάξουν με τα πραγματικά του ρομπότ αργότερα!
        self.camera_matrix = np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # --- Παράμετροι Φίλτρου & Μεγέθους ---
        self.marker_length = 0.05  # Μήκος του ArUco σε μέτρα (5 cm)
        self.use_smoothing = True
        self.alpha = 0.3
        self.filtered_tvec = None
        self.filtered_q = None

        self.get_logger().info("Unified ArUco Detector Node Started (Legacy API)!")

    def filter_pose(self, tvec: np.ndarray, q: np.ndarray):
        """ Εφαρμόζει Exponential Moving Average για να μην τρέμει ο κύβος στο RViz """
        if not self.use_smoothing:
            return tvec, q
        
        if self.filtered_tvec is None or self.filtered_q is None:
            self.filtered_tvec = tvec.copy()
            self.filtered_q = q.copy()
            return tvec, q
        
        self.filtered_tvec = (1.0 - self.alpha) * self.filtered_tvec + self.alpha * tvec
        self.filtered_q = (1.0 - self.alpha) * self.filtered_q + self.alpha * q
        
        # Κανονικοποίηση Quaternion
        self.filtered_q = self.filtered_q / np.linalg.norm(self.filtered_q)

        return self.filtered_tvec, self.filtered_q

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Εκτέλεση ανίχνευσης με τον παλιό τρόπο της OpenCV
        corners, ids, rejected = cv2.aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Υπολογισμός 3D θέσης (Pose) του ArUco
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs
            )

            # Παίρνουμε το πρώτο marker που βρέθηκε
            tvec = tvecs[0][0]
            rvec = rvecs[0][0]

            # Μετατροπή Rotation Vector σε Quaternion μέσω SciPy
            r = R.from_rotvec(rvec)
            q = r.as_quat() # Επιστρέφει [x, y, z, w]

            # Φιλτράρισμα θορύβου
            self.filtered_tvec, self.filtered_q = self.filter_pose(tvec, q)

            # --- Δημιουργία και Εκπομπή TF (Transform) ---
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_link"  # Το όνομα του frame της κάμεράς μας
            t.child_frame_id = f"aruco_marker_{ids[0][0]}"

            # CAST σε float για να μην παραπονεθεί το ROS2 για numpy types
            t.transform.translation.x = float(self.filtered_tvec[0])
            t.transform.translation.y = float(self.filtered_tvec[1])
            t.transform.translation.z = float(self.filtered_tvec[2])

            t.transform.rotation.x = float(self.filtered_q[0])
            t.transform.rotation.y = float(self.filtered_q[1])
            t.transform.rotation.z = float(self.filtered_q[2])
            t.transform.rotation.w = float(self.filtered_q[3])

            self.tf_broadcaster.sendTransform(t)

            # --- Δημιουργία και Εκπομπή Marker για το RViz2 ---
            marker = Marker()
            marker.header.stamp = t.header.stamp
            marker.header.frame_id = "camera_link"
            marker.ns = "aruco_board"
            marker.id = int(ids[0][0])
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = float(self.filtered_tvec[0])
            marker.pose.position.y = float(self.filtered_tvec[1])
            marker.pose.position.z = float(self.filtered_tvec[2])

            marker.pose.rotation.x = float(self.filtered_q[0])
            marker.pose.rotation.y = float(self.filtered_q[1])
            marker.pose.rotation.z = float(self.filtered_q[2])
            marker.pose.rotation.w = float(self.filtered_q[3])
            
            marker.scale.x = float(self.marker_length)
            marker.scale.y = float(self.marker_length)
            marker.scale.z = 0.01  # Κάνουμε τον κύβο "πλακέ" σαν χαρτόνι

            marker.color.r = 0.0
            marker.color.g = 1.0  # Πράσινο χρώμα για να φαίνεται
            marker.color.b = 0.0
            marker.color.a = 0.8

            self.marker_publisher.publish(marker)

            # Ζωγραφίζουμε πάνω στην εικόνα
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            
            if hasattr(cv2, 'drawFrameAxes'):
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length)
            elif hasattr(cv2.aruco, 'drawAxis'):
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length)

        # Δείχνουμε το παράθυρο με την κάμερα
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()