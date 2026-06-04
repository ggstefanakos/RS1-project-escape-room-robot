 #!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import math

# --- ΝΕΟ IMPORT ΓΙΑ ΤΟ QoS ---
from rclpy.qos import qos_profile_sensor_data 

class LidarToCartesianNode(Node):
    def __init__(self):
        super().__init__('lidar_to_cartesian_node')
        
        # --- ΑΛΛΑΓΗ ΕΔΩ: Χρησιμοποιούμε το qos_profile_sensor_data ---
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data) 
            
        self.publisher = self.create_publisher(PointCloud2, '/lidar_points', 10)
        self.get_logger().info('Lidar to Cartesian Node started. Waiting for /scan data...')

    def scan_callback(self, msg):
        # Βάζουμε ένα print για να επιβεβαιώσουμε ότι παίρνει δεδομένα!
        self.get_logger().info('Received Scan!', once=True) 
        
        points = []
        current_angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                x = r * math.cos(current_angle)
                y = r * math.sin(current_angle)
                z = 0.0
                points.append([x, y, z])
            current_angle += msg.angle_increment

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.frame_id

        pc2_msg = point_cloud2.create_cloud_xyz32(header, points)
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