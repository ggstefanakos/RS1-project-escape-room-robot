#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import py_trees
import time
import random
import math
from geometry_msgs.msg import PoseStamped, Point

# --- ΝΕΑ IMPORTS ΓΙΑ ΤΗΝ ΟΠΤΙΚΟΠΟΙΗΣΗ (RVIZ & NAV2) ---
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

# ==========================================
# CONFIGURATION: ΕΔΩ ΘΑ ΒΑΛΕΤΕ ΤΑ DATA ΤΗΣ ΠΑΡΟΥΣΙΑΣΗΣ
# ==========================================
# Το format είναι: { Door_ArUco_ID : Required_Key_ArUco_ID }
KEY_DOOR_MATCHES = {
    10: 1,  # Η πόρτα 10 ανοίγει με το κλειδί 1
    11: 2,  # Η πόρτα 11 ανοίγει με το κλειδί 2
    12: 3   # Η πόρτα 12 ανοίγει με το κλειδί 3
}

# ==========================================
# 1. ΤΑ BEHAVIORS (Τουβλάκια)
# ==========================================

class CheckForUnlockableDoor(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckForUnlockableDoor, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="keys_inventory", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="discovered_doors", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_door", access=py_trees.common.Access.WRITE)

    def update(self):
        # Ελέγχουμε αν υπάρχει ταίριασμα ανάμεσα στα κλειδιά μας και τις πόρτες που ξέρουμε
        for door_id, door_coords in self.blackboard.discovered_doors.items():
            required_key = KEY_DOOR_MATCHES.get(door_id)
            
            if required_key in self.blackboard.keys_inventory:
                # Έχουμε το σωστό κλειδί για αυτή την πόρτα!
                self.blackboard.target_door = {
                    "door_id": door_id, 
                    "key_id": required_key, 
                    "x": door_coords[0], 
                    "y": door_coords[1]
                }
                return py_trees.common.Status.SUCCESS
                
        return py_trees.common.Status.FAILURE

class UnlockDoorAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(UnlockDoorAction, self).__init__(name)
        self.node = node
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="target_door", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="keys_inventory", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="discovered_doors", access=py_trees.common.Access.WRITE)

    def initialise(self):
        self.goal_sent = False
        self.target = self.blackboard.target_door
        self.node.get_logger().info(f" 🚀 [MISSION] Πάω να ανοίξω την ΠΟΡΤΑ {self.target['door_id']} με το ΚΛΕΙΔΙ {self.target['key_id']}!")

    def update(self):
        if not self.goal_sent:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = float(self.target["x"])
            msg.pose.position.y = float(self.target["y"])
            msg.pose.orientation.w = 1.0
            self.goal_pub.publish(msg)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            dist = math.hypot(self.target["x"] - rx, self.target["y"] - ry)

            if dist < 0.20:
                self.node.get_logger().info(f">>> ✨ ΞΕΚΛΕΙΔΩΣΑ ΤΗΝ ΠΟΡΤΑ {self.target['door_id']}! 🔓 <<<")
                
                # Πετάμε το κλειδί και διαγράφουμε την πόρτα από τη μνήμη
                self.blackboard.keys_inventory.remove(self.target['key_id'])
                del self.blackboard.discovered_doors[self.target['door_id']]
                
                self.blackboard.target_door = None
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info(f"Πλησιάζω Πόρτα {self.target['door_id']}... Απόσταση: {dist:.2f}m")
                return py_trees.common.Status.RUNNING
                
        except TransformException:
            return py_trees.common.Status.RUNNING

# ΑΛΛΑΓΗ 2: Αντικατέστησε ολόκληρη την ExploreMazeAction
class ExploreMazeAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(ExploreMazeAction, self).__init__(name)
        self.node = node

    def update(self):
        self.node.get_logger().info("Εξερεύνηση... (Αναμονή για δεδομένα στο /vision/detected_aruco)")
        return py_trees.common.Status.RUNNING


# ==========================================
# 2. ΧΤΙΣΙΜΟ ΔΕΝΤΡΟΥ & ΚΟΜΒΟΣ
# ==========================================

def create_root(node):
    root = py_trees.composites.Selector(name="Mission_Priorities", memory=False)
    unlock_sequence = py_trees.composites.Sequence(name="Unlock_Door_Priority", memory=False)
    
    check_match = CheckForUnlockableDoor(name="Condition: Έχουμε κλειδί για γνωστή πόρτα;")
    unlock_door = UnlockDoorAction(name="Action: Άνοιξε Πόρτα", node=node)
    
    unlock_sequence.add_children([check_match, unlock_door])
    explore = ExploreMazeAction(name="Action: Εξερεύνηση", node=node)
    
    root.add_children([unlock_sequence, explore])
    return root

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        # ΑΛΛΑΓΗ 3: Μέσα στην def __init__(self):
        self.aruco_sub = self.create_subscription(
            Point, 
            '/vision/detected_aruco', 
            self.aruco_callback, 
            10
        )
        # --- PUBLISHERS ΓΙΑ RVIZ ΚΑΙ NAV2 ---
        self.marker_pub = self.create_publisher(MarkerArray, '/door_markers', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/dynamic_doors_cloud', 10)
        
        # Timer που ζωγραφίζει συνεχώς τα εμπόδια (2 φορές το δευτερόλεπτο)
        self.vis_timer = self.create_timer(0.5, self.publish_dynamic_obstacles)
        
        # Αρχικοποίηση Blackboard (Μνήμη)
        self.blackboard = py_trees.blackboard.Client(name="Master")
        self.blackboard.register_key(key="keys_inventory", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="discovered_doors", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_door", access=py_trees.common.Access.WRITE)
        
        self.blackboard.keys_inventory = []
        self.blackboard.discovered_doors = {}
        self.blackboard.target_door = None
        
        self.tree = py_trees.trees.BehaviourTree(create_root(self))
        self.tree.setup(timeout=15)
        
        self.timer = self.create_timer(1.0, self.tick_tree)

    # ΑΛΛΑΓΗ 4: Νέα συνάρτηση μέσα στο MissionControlNode
    def aruco_callback(self, msg):
        """ Διαβάζει το topic της κάμερας και ενημερώνει τη μνήμη του δέντρου """
        detected_id = int(msg.z) # Χρησιμοποιούμε το Z για το ArUco ID!
        x = float(msg.x)
        y = float(msg.y)
        
        # Αν το ID είναι ΚΛΕΙΔΙ
        if detected_id in KEY_DOOR_MATCHES.values():
            if detected_id not in self.blackboard.keys_inventory:
                self.get_logger().info(f"📥 [VISION] Βρήκα ΚΛΕΙΔΙ: {detected_id}")
                self.blackboard.keys_inventory.append(detected_id)
                
        # Αν το ID είναι ΠΟΡΤΑ
        elif detected_id in KEY_DOOR_MATCHES.keys():
            if detected_id not in self.blackboard.discovered_doors:
                self.get_logger().info(f"📥 [VISION] Βρήκα ΠΟΡΤΑ: {detected_id} στα ({x}, {y})")
                self.blackboard.discovered_doors[detected_id] = (x, y)

    def tick_tree(self):
        self.tree.tick()

    def publish_dynamic_obstacles(self):
        """ Ζωγραφίζει τις κλειδωμένες πόρτες στο RViz και στον χάρτη του A* """
        marker_array = MarkerArray()
        points = []
        marker_id = 0
        
        # Διαβάζει απευθείας από τη μνήμη ποιες πόρτες ξέρουμε
        for door_id, (x, y) in self.blackboard.discovered_doors.items():
            
            # --- 1. Marker για το RViz (Κόκκινος Κύβος) ---
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "locked_doors"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = 0.5 
            
            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 1.0
            
            marker.color.a = 0.8 # Διαφάνεια
            marker.color.r = 1.0 # Κόκκινο
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # --- 2. PointCloud για να μπλοκάρει τον A* Planner ---
            points.append([float(x), float(y), 0.0])
            points.append([float(x) + 0.1, float(y), 0.0])
            points.append([float(x) - 0.1, float(y), 0.0])
            points.append([float(x), float(y) + 0.1, 0.0])
            points.append([float(x), float(y) - 0.1, 0.0])

        self.marker_pub.publish(marker_array)
        
        # Εκπέμπουμε το PointCloud μόνο αν υπάρχουν σημεία, αλλιώς στέλνουμε άδειο για να καθαρίσει ο χάρτης
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()