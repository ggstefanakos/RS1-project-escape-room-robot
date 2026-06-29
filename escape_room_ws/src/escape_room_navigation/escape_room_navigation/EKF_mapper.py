#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped 
from std_srvs.srv import Empty 
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_ros.transform_broadcaster import TransformBroadcaster
import numpy as np
import cv2
import math
import time

class EkfLidarSlam(Node):
    def __init__(self):
        super().__init__('ekf_lidar_slam_node')

        self.X = np.zeros((3, 1)) 
        self.P = np.eye(3) * 0.1  
        
        self.Q = np.diag([1e-3, 1e-3, 1e-4]) 
        self.R = np.diag([0.01, 0.01]) 
        
        self.last_time = time.time()
        self.v = 0.0
        self.vy = 0.0
        self.omega = 0.0

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.resolution = 0.05  
        self.width_m = 10.0
        self.height_m = 10.0
        self.width_px = int(self.width_m / self.resolution)
        self.height_px = int(self.height_m / self.resolution)
        
        self.origin_x = -self.width_m / 2.0
        self.origin_y = -self.height_m / 2.0

        # === ΠΙΝΑΚΑΣ ΠΙΘΑΝΟΤΗΤΩΝ (0.0 έως 100.0) ===
        # 50.0 σημαίνει "Άγνωστο"
        self.prob_map = np.full((self.height_px, self.width_px), 50.0, dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # ===Publisher για την εκτιμώμενη θέση ===
        self.pos_pub = self.create_publisher(PoseStamped, '/est_pos', 10)

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.reset_service = self.create_service(Empty, 'reset_slam', self.reset_slam_callback)
        self.timer = self.create_timer(1.0, self.publish_map)

        self.get_logger().info("🔥 Probabilistic EKF-Lidar SLAM Started!")

    def euler_from_quaternion(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    def quaternion_from_euler(self, yaw):
        q = np.zeros(4)
        q[0] = 0.0; q[1] = 0.0; q[2] = math.sin(yaw/2.0); q[3] = math.cos(yaw/2.0)
        return q

    # === Χρήση του round() για αποφυγή του 1-pixel jitter ===
    def world_to_grid(self, x, y):
        px = int(round((x - self.origin_x) / self.resolution))
        py = int(round((y - self.origin_y) / self.resolution))
        return px, py

    def grid_to_world(self, px, py):
        x = (px * self.resolution) + self.origin_x
        y = (py * self.resolution) + self.origin_y
        return x, y

    def odom_callback(self, msg):
        curr_odom_x = msg.pose.pose.position.x
        curr_odom_y = msg.pose.pose.position.y
        curr_imu_yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        if not hasattr(self, 'first_odom_received'):
            self.odom_x = curr_odom_x
            self.odom_y = curr_odom_y
            self.odom_yaw = curr_imu_yaw
            self.first_odom_received = True
            self.last_time = time.time()
            return

        dx_global = curr_odom_x - self.odom_x
        dy_global = curr_odom_y - self.odom_y
        
        dx_local = dx_global * math.cos(-self.odom_yaw) - dy_global * math.sin(-self.odom_yaw)
        dy_local = dx_global * math.sin(-self.odom_yaw) + dy_global * math.cos(-self.odom_yaw)

        theta = self.X[2, 0]
        self.X[0, 0] += dx_local * math.cos(theta) - dy_local * math.sin(theta)
        self.X[1, 0] += dx_local * math.sin(theta) + dy_local * math.cos(theta)
        self.X[2, 0] = curr_imu_yaw 

        # Εξαγωγή του ακριβούς χρόνου του μηνύματος σε δευτερόλεπτα
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        current_time = sec + nanosec * 1e-9
        dt = current_time - self.last_time
        self.last_time = current_time
        
        v = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        
        dx_dtheta = (-v * math.sin(theta) - vy * math.cos(theta)) * dt
        dy_dtheta = ( v * math.cos(theta) - vy * math.sin(theta)) * dt

        F = np.array([
            [1.0, 0.0, dx_dtheta],
            [0.0, 1.0, dy_dtheta],
            [0.0, 0.0, 1.0]
        ])
        
        self.P = F @ self.P @ F.T + self.Q

        self.odom_x = curr_odom_x
        self.odom_y = curr_odom_y
        self.odom_yaw = curr_imu_yaw


    def reset_slam_callback(self, request, response):
        """Εκτελεί ακαριαίο reset της θέσης του EKF και καθαρισμό του χάρτη"""
        self.get_logger().warn("🔄 Reset Command Received! Re-initializing SLAM...")
        
        # 1. Μηδενισμός Κατάστασης EKF
        self.X = np.zeros((3, 1))
        self.P = np.eye(3) * 0.1
        self.v = 0.0
        self.omega = 0.0
        self.last_time = time.time()
        
        self.prob_map.fill(50.0)
        self.publish_map()
        return response

    def scan_callback(self, msg):
        rx_pred = self.X[0, 0]
        ry_pred = self.X[1, 0]
        ryaw_pred = self.X[2, 0]

        local_size = 100 
        template = np.full((local_size, local_size), 127, dtype=np.uint8)
        
        current_angle = msg.angle_min
        hit_points_temp = []

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r):
                global_angle = ryaw_pred + current_angle + math.pi
                
                lx = r * math.cos(global_angle)
                ly = r * math.sin(global_angle)
                
                px = int(round(lx / self.resolution)) + (local_size // 2)
                py = int(round(ly / self.resolution)) + (local_size // 2)
                
                if 0 <= px < local_size and 0 <= py < local_size:
                    cv2.line(template, (local_size // 2, local_size // 2), (px, py), 0, 1)
                    hit_points_temp.append((px, py))
            current_angle += msg.angle_increment

        valid_points = len(hit_points_temp)
        for px, py in hit_points_temp:
            template[py, px] = 255

        if valid_points > 20:
            rx_px, ry_px = self.world_to_grid(rx_pred, ry_pred)
            
            search_radius = 80
            x_min = max(0, rx_px - search_radius)
            x_max = min(self.width_px, rx_px + search_radius)
            y_min = max(0, ry_px - search_radius)
            y_max = min(self.height_px, ry_px + search_radius)
            
            # === Εξαγωγή του ROI από τον πίνακα πιθανοτήτων ===
            global_roi_prob = self.prob_map[y_min:y_max, x_min:x_max]
            global_roi = np.full(global_roi_prob.shape, 127, dtype=np.uint8)
            global_roi[global_roi_prob < 40.0] = 0
            global_roi[global_roi_prob > 60.0] = 255

            if global_roi.shape[0] >= template.shape[0] and global_roi.shape[1] >= template.shape[1]:
                res = cv2.matchTemplate(global_roi, template, cv2.TM_CCOEFF_NORMED)
                min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
                
                if max_val > 0.5:  
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
                    self.P = (np.eye(3) - K @ H) @ self.P 

        # === PROBABILISTIC MAPPING ===
        rx_f, ry_f, ryaw_f = self.X[0, 0], self.X[1, 0], self.X[2, 0]
        rx_f_px, ry_f_px = self.world_to_grid(rx_f, ry_f)

        rays_canvas = np.zeros(self.prob_map.shape, dtype=np.uint8)
        hit_canvas = np.zeros(self.prob_map.shape, dtype=np.uint8)
        current_angle = msg.angle_min

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not math.isinf(r):
                global_angle = ryaw_f + current_angle + math.pi
                hx = rx_f + r * math.cos(global_angle)
                hy = ry_f + r * math.sin(global_angle)
                hx_px, hy_px = self.world_to_grid(hx, hy)
                
                if 0 <= hx_px < self.width_px and 0 <= hy_px < self.height_px:
                    cv2.line(rays_canvas, (rx_f_px, ry_f_px), (hx_px, hy_px), 255, 1)
                    hit_canvas[hy_px, hx_px] = 255
            current_angle += msg.angle_increment

        # Η Μαγεία του Log-Odds: Τα ελεύθερα πεδία χάνουν 2%, τα εμπόδια κερδίζουν 10%
        self.prob_map[rays_canvas == 255] -= 3.0
        self.prob_map[hit_canvas == 255] += 9.0
        np.clip(self.prob_map, 0.0, 100.0, out=self.prob_map)

        self.broadcast_tf()

    def broadcast_tf(self):
        now = self.get_clock().now().to_msg()
        
        # === Εκπομπή Direct TF από 'map' σε 'base_footprint' ===
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.X[0, 0]
        t.transform.translation.y = self.X[1, 0]
        t.transform.translation.z = 0.0
        
        q = self.quaternion_from_euler(self.X[2, 0])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

        # === Δημοσίευση της θέσης στο topic /est_pos ===
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = self.X[0, 0]
        pose_msg.pose.position.y = self.X[1, 0]
        pose_msg.pose.position.z = 0.0
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        
        self.pos_pub.publish(pose_msg)

    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.width_px
        msg.info.height = self.height_px
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        
        # Μετατροπή του Πίνακα Πιθανοτήτων σε format του ROS 2
        ros_grid = np.full(self.prob_map.shape, -1, dtype=np.int8)
        ros_grid[self.prob_map < 40.0] = 0    # Πιθανότητα κάτω από 40% = Ελεύθερο
        ros_grid[self.prob_map > 60.0] = 100  # Πιθανότητα πάνω από 60% = Τοίχος
        
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