#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Empty # ΠΡΟΣΘΗΚΗ: Το standard Empty Service του ROS2
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros.transform_broadcaster import TransformBroadcaster
import numpy as np
import cv2
import math
import time

class EkfLidarSlam(Node):
    def __init__(self):
        super().__init__('ekf_lidar_slam_node')

        # --- 1. EKF STATE VECTORS & MATRICES ---
        self.X = np.zeros((3, 1)) # State: [x, y, theta]
        self.P = np.eye(3) * 0.1  # Αρχική αβεβαιότητα
        
        # Πόσο δεν εμπιστευόμαστε την οδομετρία (Process Noise)
        self.Q = np.diag([0.05, 0.05, 0.01]) 
        
        # Πόσο δεν εμπιστευόμαστε το OpenCV Scan Matching (Measurement Noise)
        self.R = np.diag([0.01, 0.01]) 
        
        self.last_time = time.time()
        self.v = 0.0
        self.omega = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # --- 2. ΡΥΘΜΙΣΕΙΣ ΧΑΡΤΗ (Grid) ---
        self.resolution = 0.05  # 5cm ανά pixel
        self.width_m = 6.0
        self.height_m = 6.0
        self.width_px = int(self.width_m / self.resolution)
        self.height_px = int(self.height_m / self.resolution)
        
        # 127 = Άγνωστο, 255 = Εμπόδιο, 0 = Ελεύθερο
        self.grid = np.full((self.height_px, self.width_px), 127, dtype=np.uint8)
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # --- 3. ROS INFRASTRUCTURE ---
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        
        # ΠΡΟΣΘΗΚΗ: Δημιουργία του Service Server για το Reset
        self.reset_service = self.create_service(Empty, 'reset_slam', self.reset_slam_callback)
        
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info("🔥 EKF-Lidar SLAM Started (Odom + OpenCV Scan Matching) with Reset Service Available!")

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def world_to_grid(self, x, y):
        px = int((x - self.origin_x) / self.resolution)
        py = int((y - self.origin_y) / self.resolution)
        return px, py

    def grid_to_world(self, px, py):
        x = (px * self.resolution) + self.origin_x
        y = (py * self.resolution) + self.origin_y
        return x, y

    # ====== ΠΡΟΣΘΗΚΗ: Ο SERVICE SERVER ΚΩΔΙΚΑΣ ======
    def reset_slam_callback(self, request, response):
        """Εκτελεί ακαριαίο reset της θέσης του EKF και καθαρισμό του χάρτη"""
        self.get_logger().warn("🔄 Reset Command Received! Re-initializing SLAM...")
        
        # 1. Μηδενισμός Κατάστασης EKF
        self.X = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1
        self.v = 0.0
        self.omega = 0.0
        self.last_time = time.time()
        
        # 2. Ολικός Καθαρισμός Χάρτη (Όλα ξανά 127 / Άγνωστα)
        self.grid.fill(127)
        
        # 3. Αναγκαστικό άμεσο publish του άδειου χάρτη για να ενημερωθεί το RViz
        self.publish_map()
        
        self.get_logger().info("✅ SLAM successfully initialized to (0,0,0). Map cleared.")
        return response

    # ====== ΒΗΜΑ 1: EKF PREDICT (Από την Οδομετρία) ======
    def odom_callback(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
        
        imu_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        
        # Αποθηκεύουμε την ωμή θέση για το TF
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_yaw = imu_yaw

        theta = self.X[2, 0]

        # EKF Predict
        self.X[0, 0] += self.v * math.cos(theta) * dt
        self.X[1, 0] += self.v * math.sin(theta) * dt
        self.X[2, 0] = imu_yaw  

        F = np.array([
            [1.0, 0.0, -self.v * math.sin(theta) * dt],
            [0.0, 1.0,  self.v * math.cos(theta) * dt],
            [0.0, 0.0, 1.0]
        ])

        self.P = F @ self.P @ F.T + self.Q

        # ΕΚΠΟΜΠΗ ODOM -> BASE_FOOTPRINT
        t_odom = TransformStamped()
        t_odom.header.stamp = self.get_clock().now().to_msg()
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_footprint'
        t_odom.transform.translation.x = self.odom_x
        t_odom.transform.translation.y = self.odom_y
        t_odom.transform.translation.z = msg.pose.pose.position.z
        t_odom.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t_odom)
    # ====== ΒΗΜΑ 2 & 3: MEASUREMENT & UPDATE (Από το Lidar) ======
    def scan_callback(self, msg):
        rx_pred = self.X[0, 0]
        ry_pred = self.X[1, 0]
        ryaw_pred = self.X[2, 0]

        # 1. Δημιουργούμε μια "εικόνα" (Local Template) από το τρέχον Lidar
        local_size = 50 # 100x100 pixels = 5x5 μέτρα
        template = np.full((local_size, local_size), 127, dtype=np.uint8)
        
        current_angle = msg.angle_min
        valid_points = 0

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r):
                # ΔΙΟΡΘΩΣΗ: Προσθήκη math.pi για το hardware offset του Lidar
                global_angle = ryaw_pred + current_angle + math.pi
                
                lx = r * math.cos(global_angle)
                ly = r * math.sin(global_angle)
                
                px = int(lx / self.resolution) + (local_size // 2)
                py = int(ly / self.resolution) + (local_size // 2)
                
                if 0 <= px < local_size and 0 <= py < local_size:
                    template[py, px] = 255 
                    valid_points += 1
            current_angle += msg.angle_increment

        if valid_points > 20:
            rx_px, ry_px = self.world_to_grid(rx_pred, ry_pred)
            
            search_radius = 80
            x_min = max(0, rx_px - search_radius)
            x_max = min(self.width_px, rx_px + search_radius)
            y_min = max(0, ry_px - search_radius)
            y_max = min(self.height_px, ry_px + search_radius)
            
            global_roi = self.grid[y_min:y_max, x_min:x_max]

            if global_roi.shape[0] >= template.shape[0] and global_roi.shape[1] >= template.shape[1]:
                res = cv2.matchTemplate(global_roi, template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                
                if max_val > 0.3:  
                    match_px = x_min + max_loc[0] + (local_size // 2)
                    match_py = y_min + max_loc[1] + (local_size // 2)
                    
                    z_x, z_y = self.grid_to_world(match_px, match_py)
                    Z = np.array([[z_x], [z_y]])
                    
                    H = np.array([
                        [1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0]
                    ])
                    
                    y_res = Z - (H @ self.X)          
                    S = H @ self.P @ H.T + self.R     
                    K = self.P @ H.T @ np.linalg.inv(S) 
                    
                    self.X = self.X + K @ y_res       
                    #self.P = (np.eye(3) - K @ H) @ self.P 

        # ====== ΒΗΜΑ 4: ΧΑΡΤΟΓΡΑΦΗΣΗ ======
        rx_f, ry_f, ryaw_f = self.X[0, 0], self.X[1, 0], self.X[2, 0]
        rx_f_px, ry_f_px = self.world_to_grid(rx_f, ry_f)

        rays_canvas = np.zeros_like(self.grid, dtype=np.uint8)
        hit_points = []
        current_angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r):
                # ΔΙΟΡΘΩΣΗ: Προσθήκη math.pi και εδώ
                global_angle = ryaw_f + current_angle + math.pi
                hx = rx_f + r * math.cos(global_angle)
                hy = ry_f + r * math.sin(global_angle)
                hx_px, hy_px = self.world_to_grid(hx, hy)
                
                if 0 <= hx_px < self.width_px and 0 <= hy_px < self.height_px:
                    cv2.line(rays_canvas, (rx_f_px, ry_f_px), (hx_px, hy_px), 255, 1)
                    hit_points.append((hx_px, hy_px))
            current_angle += msg.angle_increment

        self.grid[rays_canvas == 255] = 0
        for hx, hy in hit_points:
            self.grid[hy, hx] = 255

        self.broadcast_tf()

    def quaternion_from_euler(self, yaw):
        # Βοηθητική συνάρτηση (αν δεν την έχεις ήδη)
        q = np.zeros(4)
        q[0] = 0.0; q[1] = 0.0; q[2] = math.sin(yaw/2.0); q[3] = math.cos(yaw/2.0)
        return q

    def broadcast_tf(self):
        # 1. Βρίσκουμε τη διαφορά γωνίας (Τέλεια - Ωμή)
        yaw_diff = self.X[2, 0] - self.odom_yaw
        
        # 2. Βρίσκουμε τη διαφορά στα X, Y (περιστρέφοντας την οδομετρία για να ευθυγραμμιστεί)
        x_diff = self.X[0, 0] - (self.odom_x * math.cos(yaw_diff) - self.odom_y * math.sin(yaw_diff))
        y_diff = self.X[1, 0] - (self.odom_x * math.sin(yaw_diff) + self.odom_y * math.cos(yaw_diff))

        # 3. Στέλνουμε το σφάλμα ως MAP -> ODOM
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        
        t.transform.translation.x = x_diff
        t.transform.translation.y = y_diff
        t.transform.translation.z = 0.0
        
        q = self.quaternion_from_euler(yaw_diff)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width_px
        msg.info.height = self.height_px
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        
        ros_grid = np.copy(self.grid).astype(np.int8)
        ros_grid[self.grid == 0] = 0       
        ros_grid[self.grid == 127] = -1    
        ros_grid[self.grid == 255] = 100   
        
        msg.data = ros_grid.flatten().tolist()
        self.map_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EkfLidarSlam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()