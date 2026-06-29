#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import py_trees
import time
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

# ==========================================
# CONFIGURATION
# ==========================================
KEY_DOOR_MATCHES = {
    0: 1,  # Η πόρτα 0 ανοίγει με το κλειδί 1
    2: 4,  # Η πόρτα 2 ανοίγει με το κλειδί 4
    5: 6   # Η πόρτα 5 ανοίγει με το κλειδί 6
}

# ==========================================
# 1. ΤΑ BEHAVIORS (Τουβλάκια)
# ==========================================


class InitialSpinAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(InitialSpinAction, self).__init__(name)
        self.node = node
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.spin_duration = 13.0 # Δευτερόλεπτα (Αργή περιστροφή)
        self.start_time = None

    def initialise(self):
        self.start_time = self.node.get_clock().now().nanoseconds / 1e9
        self.node.get_logger().info(f"🌀 Ξεκινάω 360 Spin αναγνώρισης χώρου ({int(self.spin_duration)} sec)...")

    def update(self):
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        if current_time - self.start_time < self.spin_duration:
            # Δημιουργία καθαρού μηνύματος
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.01
            
            self.cmd_pub.publish(msg)
            return py_trees.common.Status.RUNNING
        else:
            # Φρένο - επίσης καθαρό
            stop_msg = Twist()
            # Τα float είναι από default 0.0, αλλά για σιγουριά:
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            
            self.cmd_pub.publish(stop_msg)
            self.node.get_logger().info("✅ Το 360 Spin ολοκληρώθηκε!")
            return py_trees.common.Status.SUCCESS
class CheckForUnlockableDoor(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckForUnlockableDoor, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="keys_inventory", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="discovered_doors", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="target_door", access=py_trees.common.Access.WRITE)

    def update(self):
        for door_id, door_coords in self.blackboard.discovered_doors.items():
            required_key = KEY_DOOR_MATCHES.get(door_id)
            
            if required_key in self.blackboard.keys_inventory:
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
        # ΠΡΟΣΘΗΚΗ: Μνήμη για τις πόρτες που ανοίξαμε
        self.blackboard.register_key(key="unlocked_doors", access=py_trees.common.Access.WRITE)

    def initialise(self):
        self.goal_sent = False
        self.target = self.blackboard.target_door
        self.node.get_logger().info(f" 🚀 [MISSION] Πάω στο ΚΕΝΤΡΟ της ΠΟΡΤΑΣ {self.target['door_id']} με το ΚΛΕΙΔΙ {self.target['key_id']}!")

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
                
                self.blackboard.keys_inventory.remove(self.target['key_id'])
                del self.blackboard.discovered_doors[self.target['door_id']]
                
                # Καταγράφουμε ότι άνοιξε για να την αγνοεί η κάμερα στο μέλλον
                self.blackboard.unlocked_doors.append(self.target['door_id'])
                self.blackboard.target_door = None
                
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info(f"Πλησιάζω Κέντρο Πόρτας {self.target['door_id']}... Απόσταση: {dist:.2f}m")
                return py_trees.common.Status.RUNNING
                
        except TransformException:
            return py_trees.common.Status.RUNNING

class ExploreMazeAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.candidates = []
        self.sub_candidate_idx = 0 
        self.goal_sent = False
        
        # ΝΕΟ: Μνήμη για τις γειτονιές που αποδείχθηκαν απροσπέλαστες
        self.blacklisted_tiles = set()

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except TransformException as e:
            self.node.get_logger().warn(f"Δεν βρέθηκε TF: {e}")
            return None
        
    def update(self):
        # 1. ΔΥΝΑΜΙΚΗ ΑΝΑΝΕΩΣΗ: Αν η λίστα είναι άδεια, σκανάρουμε τον ΝΕΟ χάρτη
        if not self.candidates:
            self.candidates = self.get_frontier_candidates()
            self.sub_candidate_idx = 0
        
        if not self.candidates:
            self.node.get_logger().warn("Δεν υπάρχουν άλλα frontiers. Εξερεύνηση ολοκληρώθηκε!")
            return py_trees.common.Status.FAILURE

        # Παίρνουμε ΠΑΝΤΑ το καλύτερο tile (index 0) της ανανεωμένης λίστας
        current_tile_info = self.candidates[0] 
        current_target_list = current_tile_info['points']
        current_tile_id = current_tile_info['tile_id']
        current_target = current_target_list[self.sub_candidate_idx]

        # 2. ΕΛΕΓΧΟΣ ΑΝ ΦΤΑΣΑΜΕ ΣΤΟ GOAL
        robot_pose = self.get_robot_pose()
        if robot_pose and self.goal_sent:
            dist = math.hypot(robot_pose[0] - current_target[0], robot_pose[1] - current_target[1])
            
            if dist < 0.2:
                self.node.get_logger().info(f"📍 Περιοχή εξερευνήθηκε! Αναγκαστική ανανέωση χάρτη...")
                self.candidates = [] # Αδειάζουμε τη λίστα για δυναμικό recalculation
                self.blacklisted_tiles.clear() # Καθαρίζουμε το ιστορικό λαθών γιατί αλλάξαμε οπτική!
                # Φρένο - επίσης καθαρό
                stop_msg = Twist()
                # Τα float είναι από default 0.0, αλλά για σιγουριά:
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
            
                self.cmd_pub.publish(stop_msg)
                self.goal_sent = False
                return py_trees.common.Status.RUNNING

        # 3. ΕΛΕΓΧΟΣ ΑΠΟΤΥΧΙΑΣ PLANNER
        if self.goal_sent:
            # Περιμένουμε τον Planner να υπολογίσει! 
            # Αν δεν έχει έρθει ακόμα απάντηση, δεν κάνουμε τίποτα.
            if not self.node.blackboard.path_received:
                return py_trees.common.Status.RUNNING

            # Αν ΗΡΘΕ απάντηση και είναι άδεια, σημαίνει πραγματική αποτυχία
            if len(self.node.blackboard.current_path) == 0:
                self.node.get_logger().warn(f"Planner απέτυχε στο σημείο {self.sub_candidate_idx+1}/{len(current_target_list)}.")
                
                self.sub_candidate_idx += 1 
                
                # Αν δοκιμάσαμε όλα τα σημεία της γειτονιάς
                if self.sub_candidate_idx >= len(current_target_list):
                    self.node.get_logger().warn(f"🚫 Η γειτονιά είναι εντελώς απροσπέλαστη. Blacklisted!")
                    self.blacklisted_tiles.add(current_tile_id) 
                    self.candidates = [] 
                
                self.goal_sent = False
                return py_trees.common.Status.RUNNING

        # 4. ΑΠΟΣΤΟΛΗ ΣΤΟΧΟΥ
        if not self.goal_sent:
            # ΠΡΙΝ στείλουμε νέο στόχο, σβήνουμε το παλιό path και κατεβάζουμε το σημαιάκι
            self.node.blackboard.path_received = False
            self.node.blackboard.current_path = []
            
            self.publish_goal(current_target)
            self.goal_sent = True
            self.node.get_logger().info(f"🚀 Στόχος σε νέα γειτονιά (Σημείο δοκιμής: {self.sub_candidate_idx+1}/{len(current_target_list)})")
            
        return py_trees.common.Status.RUNNING

    def get_frontier_candidates(self):
        grid = self.node.blackboard.grid_map
        if grid is None: return []
        res = self.node.blackboard.map_info.resolution
        orig_x = self.node.blackboard.map_info.origin.position.x
        orig_y = self.node.blackboard.map_info.origin.position.y
        h, w = grid.shape
        tile_size = 20
        
        candidates = []
        for y in range(0, h - tile_size, tile_size):
            for x in range(0, w - tile_size, tile_size):
                tile_id = (x, y)
                
                # ΝΕΟ: Αν το tile έχει αποτύχει στο παρελθόν, το προσπερνάμε ακαριαία
                if tile_id in self.blacklisted_tiles:
                    continue
                    
                tile = grid[y:y+tile_size, x:x+tile_size]
                tile_arr = np.array(tile, dtype=np.int8)
                
                unknown_count = np.sum(tile_arr == -1)
                free_count = np.sum(tile_arr == 0)
                
                if unknown_count > 0 and free_count > 0:
                    free_indices = np.argwhere(tile_arr == 0)
                    
                    # Μέχρι 3 σημεία διάσπαρτα στον ελεύθερο χώρο του tile
                    num_points = min(5, len(free_indices))
                    step = max(1, len(free_indices) // num_points)
                    
                    sub_targets = []
                    for i in range(num_points):
                        fy, fx = free_indices[i * step]
                        real_x = (x + fx) * res + orig_x
                        real_y = (y + fy) * res + orig_y
                        sub_targets.append((real_x, real_y))
                    
                    # Αποθηκεύουμε τα σημεία, το ID για το Blacklist και το Score
                    candidates.append({'points': sub_targets, 'tile_id': tile_id, 'score': unknown_count})
                    
        # Ταξινόμηση βάσει άγνωστων pixels
        candidates.sort(key=lambda k: k['score'], reverse=True)
        return candidates

    def publish_goal(self, pose_tuple):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.position.x = pose_tuple[0]
        goal_msg.pose.position.y = pose_tuple[1]
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)

    def publish_goal(self, pose_tuple):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.position.x = pose_tuple[0]
        goal_msg.pose.position.y = pose_tuple[1]
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)

def create_root(node):
    # Κεντρική Ακολουθία: Πρώτα Spin (μια φορά) -> Μετά Αποστολές
    root = py_trees.composites.Sequence(name="Main_Mission", memory=True)
    
    spin_action = InitialSpinAction(name="Action: 360 Spin", node=node)
    
    # ΠΡΟΣΘΗΚΗ: wrapping του spin σε OneShot Decorator
    spin_oneshot = py_trees.decorators.OneShot(
        child=spin_action,
        name="OneShot_Protection",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )
    
    mission_selector = py_trees.composites.Selector(name="Mission_Priorities", memory=False)
    
    unlock_sequence = py_trees.composites.Sequence(name="Unlock_Door_Priority", memory=False)
    check_match = CheckForUnlockableDoor(name="Condition: Έχουμε κλειδί για γνωστή πόρτα;")
    unlock_door = UnlockDoorAction(name="Action: Άνοιξε Πόρτα", node=node)
    
    unlock_sequence.add_children([check_match, unlock_door])
    explore = ExploreMazeAction(name="Action: Εξερεύνηση", node=node)
    
    mission_selector.add_children([unlock_sequence, explore])
    
    # ΠΡΟΣΘΗΚΗ: Αντί για το spin_action, βάζουμε το spin_oneshot
    root.add_children([spin_oneshot, mission_selector])
    
    return root

# Πρόσθεσε το import για το Path αν λείπει
from nav_msgs.msg import Path

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        # ... (υπάρχοντα subscriptions)

        self.blackboard = py_trees.blackboard.Client(name="Master")
        
        # 2. ΜΕΤΑ ΚΑΝΕΙΣ REGISTER ΤΑ KEYS
        self.blackboard.register_key(key="keys_inventory", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="discovered_doors", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_door", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="unlocked_doors", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="grid_map", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="map_info", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="path_received", access=py_trees.common.Access.WRITE)
        self.blackboard.path_received = False
        
        # 3. ΠΡΟΣΘΗΚΗ ΤΟΥ ΝΕΟΥ KEY (Εδώ ήταν το λάθος αν το είχες βάλει παραπάνω)
        self.blackboard.register_key(key="current_path", access=py_trees.common.Access.WRITE)
        
        # ΠΡΟΣΘΗΚΗ: Subscription στο /plan
        self.path_sub = self.create_subscription(Path, '/plan', self.path_callback, 10)
        
        # ΠΡΟΣΘΗΚΗ στο blackboard
        self.blackboard.current_path = [] # Αρχικοποίηση με άδειο μονοπάτι
        
        # ... (υπόλοιπα)
        self.aruco_sub = self.create_subscription(
            Point, 
            '/vision/detected_aruco', 
            self.aruco_callback, 
            10
        )        

        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        
        self.marker_pub = self.create_publisher(MarkerArray, '/door_markers', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/dynamic_doors_cloud', 10)
        self.vis_timer = self.create_timer(0.5, self.publish_dynamic_obstacles)
        
        # ΠΡΟΣΘΗΚΗ: Δομές δεδομένων για τον υπολογισμό των πορτών
        self.raw_door_posts = {} # Format: {door_id: [(x, y)]}
        self.DOOR_WIDTH_THRESHOLD = 0.2 # Η ελάχιστη απόσταση σε μέτρα ανάμεσα στα 2 ίδια ArUco για να θεωρηθούν ξεχωριστές κολώνες (όχι θόρυβος)

        self.blackboard.grid_map = None
        self.blackboard.map_info = None
        
        self.blackboard.keys_inventory = []
        self.blackboard.discovered_doors = {}
        self.blackboard.target_door = None
        self.blackboard.unlocked_doors = []
        
        self.tree = py_trees.trees.BehaviourTree(create_root(self))
        self.tree.setup(timeout=15)
        
        self.timer = self.create_timer(1.0, self.tick_tree)

    def path_callback(self, msg):
        # Αποθήκευση του μονοπατιού στο Blackboard
        self.blackboard.current_path = msg.poses
        # Ανάβουμε το "πράσινο φως" ότι ο planner απάντησε!
        self.blackboard.path_received = True

    def aruco_callback(self, msg):
        detected_id = int(msg.z) 
        x = float(msg.x)
        y = float(msg.y)
        
        # Αν το ID είναι ΚΛΕΙΔΙ
        if detected_id in KEY_DOOR_MATCHES.values():
            if detected_id not in self.blackboard.keys_inventory:
                self.get_logger().info(f"📥 [VISION] Βρήκα ΚΛΕΙΔΙ: {detected_id}")
                self.blackboard.keys_inventory.append(detected_id)
                
        # Αν το ID είναι ΠΟΡΤΑ
        elif detected_id in KEY_DOOR_MATCHES.keys():
            # Αγνόησε την αν την έχουμε ήδη ανοίξει ή αν έχει ήδη βρεθεί πλήρως
            if detected_id in self.blackboard.unlocked_doors or detected_id in self.blackboard.discovered_doors:
                return

            if detected_id not in self.raw_door_posts:
                self.raw_door_posts[detected_id] = [(x, y)]
                self.get_logger().info(f"🔍 [VISION] Εντοπίστηκε η 1η κολώνα της ΠΟΡΤΑΣ {detected_id} στα ({x:.2f}, {y:.2f}). Ψάχνω την 2η...")
            else:
                # Έχουμε δει ξανά αυτό το ID. Είναι η 2η κολώνα ή απλά διαβάσαμε την 1η από άλλη γωνία;
                first_post = self.raw_door_posts[detected_id][0]
                dist = math.hypot(x - first_post[0], y - first_post[1])
                
                # Αν απέχει ικανοποιητικά, τότε είναι το 2ο ArUco της πόρτας!
                if dist > self.DOOR_WIDTH_THRESHOLD and len(self.raw_door_posts[detected_id]) == 1:
                    # Υπολογισμός του μέσου (Κέντρο του Ανοίγματος)
                    mid_x = (first_post[0] + x) / 2.0
                    mid_y = (first_post[1] + y) / 2.0
                    
                    self.blackboard.discovered_doors[detected_id] = (mid_x, mid_y)
                    self.get_logger().info(f"🎯 [VISION] Η ΠΟΡΤΑ {detected_id} ΚΛΕΙΔΩΣΕ! Κέντρο στα ({mid_x:.2f}, {mid_y:.2f}). (Άνοιγμα {dist:.2f}m)")

    def tick_tree(self):
        self.tree.tick()

    def publish_dynamic_obstacles(self):
        marker_array = MarkerArray()
        delete_all_marker = Marker()
        delete_all_marker.action = Marker.DELETEALL 
        marker_array.markers.append(delete_all_marker)
        points = []
        marker_id = 0
        
        for door_id, (mid_x, mid_y) in self.blackboard.discovered_doors.items():
            
            # --- 1. Marker για το RViz (Κόκκινος Κύβος στο κέντρο) ---
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "locked_doors"
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(mid_x)
            marker.pose.position.y = float(mid_y)
            marker.pose.position.z = 0.5 
            
            marker.scale.x = 0.6 # Το κάνουμε λίγο πιο πλατύ για να μπλοκάρει το πέρασμα
            marker.scale.y = 0.6
            marker.scale.z = 1.0
            
            marker.color.a = 0.8 
            marker.color.r = 1.0 
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            marker_array.markers.append(marker)
            marker_id += 1
            
            # --- 2. PointCloud για τον A* Planner ---
            points.append([float(mid_x), float(mid_y), 0.0])
            points.append([float(mid_x) + 0.15, float(mid_y), 0.0])
            points.append([float(mid_x) - 0.15, float(mid_y), 0.0])
            points.append([float(mid_x), float(mid_y) + 0.15, 0.0])
            points.append([float(mid_x), float(mid_y) - 0.15, 0.0])

        self.marker_pub.publish(marker_array)
        
        header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(cloud_msg)
    def map_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        self.blackboard.grid_map = np.array(msg.data, dtype=np.int8).reshape((height, width))
        self.blackboard.map_info = msg.info
        
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