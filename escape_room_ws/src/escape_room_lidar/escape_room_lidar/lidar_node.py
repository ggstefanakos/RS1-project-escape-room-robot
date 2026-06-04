#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import math

class LidarToCartesianNode(Node):
    def __init__(self):
        super().__init__('lidar_to_cartesian_node')
        
        # Κάνουμε subscribe στο topic του Lidar
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        # Publisher για τα σημεία στον χώρο (έτοιμα για RViz ή Mapping)
        self.publisher = self.create_publisher(PointCloud2, '/lidar_points', 10)
        self.get_logger().info('Lidar to Cartesian Node started. Waiting for /scan data...')

    def scan_callback(self, msg):
        points = []
        
        # Η γωνία ξεκινάει από την ελάχιστη τιμή που δηλώνει ο σένσορας (π.χ. -180 μοίρες)
        current_angle = msg.angle_min

        # Διατρέχουμε όλες τις μετρήσεις απόστασης
        for r in msg.ranges:
            # Απορρίπτουμε σφάλματα, μηδενικά και άπειρα (άκυρες μετρήσεις Lidar)
            if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                
                # Τριγωνομετρική Μετατροπή (Πολικές -> Καρτεσιανές)
                x = r * math.cos(current_angle)
                y = r * math.sin(current_angle)
                z = 0.0  # Επειδή θέλουμε 2D χάρτη, το ύψος είναι σταθερό μηδέν
                
                points.append([x, y, z])
            
            # Αυξάνουμε τη γωνία για την επόμενη ακτίνα
            current_angle += msg.angle_increment

        # Δημιουργία Header για το μήνυμα
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.frame_id # Κρατάμε το frame_id του Lidar (π.χ. 'laser_frame')

        # Μετατροπή της λίστας σημείων στο επίσημο μήνυμα PointCloud2 του ROS2
        pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
        
        # Εκπομπή των σημείων
        self.publisher.publish(pc2_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarToCartesianNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()