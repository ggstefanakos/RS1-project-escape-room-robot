#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import math

class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        
        # Τρέχουσα εικονική θέση του ρομπότ (ξεκινάμε από το 0,0)
        self.x = 0.0
        self.y = 0.0
        
        # Στόχος (αρχικά None)
        self.target_x = None
        self.target_y = None
        
        # Παράμετροι κίνησης
        self.speed = 0.1 # m/s (ταχύτητα προσομοίωσης)
        self.dt = 0.1    # 10Hz (συχνότητα ανανέωσης)
        
        # Subscriber στο goal_pose που εκπέμπει το mission_control
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
            
        # TF Broadcaster για να εκπέμπουμε τη θέση στο tf tree
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer για να ανανεώνουμε τη θέση και να στέλνουμε TF συνεχώς
        self.timer = self.create_timer(self.dt, self.update_position)
        self.get_logger().info("Το Mock Robot ξεκίνησε! Περιμένει στόχο στο /goal_pose...")

    def goal_callback(self, msg):
        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.get_logger().info(f"Νέος στόχος ελήφθη: ({self.target_x}, {self.target_y})")

    def update_position(self):
        # Αν έχουμε στόχο, κινήσου προς τα εκεί με βάση την ταχύτητα
        if self.target_x is not None and self.target_y is not None:
            dx = self.target_x - self.x
            dy = self.target_y - self.y
            dist = math.hypot(dx, dy)
            
            if dist > 0.05: # Αν απέχουμε πάνω από 5cm, συνέχισε να κινείσαι
                angle = math.atan2(dy, dx)
                # Υπολογισμός νέας θέσης
                self.x += self.speed * math.cos(angle) * self.dt
                self.y += self.speed * math.sin(angle) * self.dt
        
        # Φτιάξε το μήνυμα TransformStamped
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Απλοποίηση: Μηδενικό rotation (quaternion) για τις ανάγκες του τεστ
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        # Εκπομπή του TF
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = MockRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()