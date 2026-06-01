#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

def euler_from_quaternion(x, y, z, w):
    """Βοηθητική συνάρτηση: Μετατρέπει τα quaternions του ROS σε γωνία (rad)"""
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z

class LocalPlannerPID(Node):
    def __init__(self):
        super().__init__('local_planner_node')
        
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_path = []
        self.target_idx = 0

        # --- ΡΥΘΜΙΣΕΙΣ ΠΛΗΡΟΥΣ PID ΚΑΙ ΟΔΗΓΗΣΗΣ ---
        self.Kp_ang = 1.5   # P: Δύναμη στροφής προς τον στόχο
        self.Ki_ang = 0.05  # I: Διόρθωση μικρής μόνιμης απόκλισης (π.χ. αν το ρομπότ "τραβάει" μονόπατα)
        self.Kd_ang = 0.3   # D: Φρένο για να μην ταλαντώνεται (κάνει ζιγκ-ζαγκ)
        
        # Μεταβλητές μνήμης για το PID
        self.prev_error_ang = 0.0
        self.integral_error_ang = 0.0
        self.max_integral = 2.0 # ANTI-WINDUP: Δεν αφήνουμε το ολοκλήρωμα να ξεφύγει!

        self.max_linear_speed = 0.20  # m/s
        self.lookahead_distance = 0.3 # Πόσο μπροστά στο μονοπάτι κοιτάει

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Full PID Local Controller initialized! Waiting for /plan...")

    def path_callback(self, msg):
        self.current_path = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        self.target_idx = 0
        # Κάθε φορά που έχουμε νέο μονοπάτι, μηδενίζουμε τη μνήμη του PID
        self.integral_error_ang = 0.0
        self.prev_error_ang = 0.0
        self.get_logger().info(f"Received new path with {len(self.current_path)} waypoints.")

    def control_loop(self):
        if not self.current_path:
            return

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except TransformException:
            return

        rx = t.transform.translation.x
        ry = t.transform.translation.y
        rz = euler_from_quaternion(
            t.transform.rotation.x, t.transform.rotation.y,
            t.transform.rotation.z, t.transform.rotation.w)

        # 1. Βρίσκουμε τον στόχο (Lookahead)
        target_x, target_y = self.current_path[-1]
        for i in range(self.target_idx, len(self.current_path)):
            px, py = self.current_path[i]
            dist = math.hypot(px - rx, py - ry)
            if dist > self.lookahead_distance:
                target_x, target_y = px, py
                self.target_idx = i
                break

        # 2. Υπολογισμός Σφάλματος (Error)
        angle_to_target = math.atan2(target_y - ry, target_x - rx)
        error_ang = angle_to_target - rz
        
        # Κανονικοποίηση [-π, π]
        error_ang = math.atan2(math.sin(error_ang), math.cos(error_ang))

        # --- ΥΠΟΛΟΓΙΣΜΟΣ ΠΛΗΡΟΥΣ PID ---
        
        # Proportional (P)
        p_term = self.Kp_ang * error_ang
        
        # Integral (I) με Anti-Windup προστασία
        self.integral_error_ang += error_ang
        # Κόβουμε το άθροισμα αν ξεπεράσει το όριο που θέσαμε
        self.integral_error_ang = max(min(self.integral_error_ang, self.max_integral), -self.max_integral)
        i_term = self.Ki_ang * self.integral_error_ang
        
        # Derivative (D)
        d_term = self.Kd_ang * (error_ang - self.prev_error_ang)
        self.prev_error_ang = error_ang

        # Τελική εντολή στροφής (Sum)
        cmd_w = p_term + i_term + d_term

        # --- ΕΛΕΓΧΟΣ ΤΑΧΥΤΗΤΑΣ ΚΑΙ ΤΕΡΜΑΤΙΣΜΟΥ ---
        cmd_v = self.max_linear_speed * max(0.0, (1.0 - abs(error_ang)/(math.pi/2)))

        final_x, final_y = self.current_path[-1]
        if math.hypot(final_x - rx, final_y - ry) < 0.15:
            self.get_logger().info("Target Reached!")
            self.current_path = []
            cmd_v = 0.0
            cmd_w = 0.0

        twist = Twist()
        twist.linear.x = float(cmd_v)
        twist.angular.z = float(cmd_w)
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerPID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()