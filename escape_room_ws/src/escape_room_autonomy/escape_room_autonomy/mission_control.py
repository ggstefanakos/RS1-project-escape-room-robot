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
from nav_msgs.msg import OccupancyGrid, Path   # <-- Προσθήκη: Path
import numpy as np
import cv2
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

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
            msg = Twist()
            msg.angular.z = 0.09
            self.cmd_pub.publish(msg)
            return py_trees.common.Status.RUNNING
        else:
            self.cmd_pub.publish(Twist())
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
                self.blackboard.unlocked_doors.append(self.target['door_id'])
                self.blackboard.target_door = None
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info(f"Πλησιάζω Κέντρο Πόρτας {self.target['door_id']}... Απόσταση: {dist:.2f}m")
                return py_trees.common.Status.RUNNING

        except TransformException:
            return py_trees.common.Status.RUNNING


# ==========================================
# 2. FRONTIER EXPLORATION BEHAVIOUR
# ==========================================

class ExploreMazeAction(py_trees.behaviour.Behaviour):
    """
    Frontier-based maze exploration ως py_trees Behaviour.

    Αλγόριθμος:
    1. Ανάγνωση χάρτη από το Blackboard (grid_map, map_info).
    2. Ανίχνευση frontier cells: ελεύθερα κελιά (==0) που γειτνιάζουν με άγνωστα (==-1).
    3. Ομαδοποίηση frontiers σε clusters (cv2.connectedComponents).
    4. Επιλογή καλύτερου cluster: score = distance / sqrt(size).
    5. Αποστολή goal στον A* Planner (/goal_pose).
    6. Αναμονή — αν επιτευχθεί ο στόχος → επανάληψη.
       Αν timeout → blacklist του frontier και δοκιμή επόμενου.
    7. Επιστρέφει SUCCESS όταν δεν βρεθούν άλλα frontiers.

    Topics που χρησιμοποιεί:
        SUB: /est_pos       (PoseStamped) – εκτιμώμενη θέση από EKF SLAM
        SUB: /plan          (Path)        – άδειο path = goal reached (από A* Planner)
        PUB: /goal_pose     (PoseStamped) – εντολή στόχου για τον A* Planner

    Blackboard keys (READ):
        grid_map  – np.ndarray (height × width), τιμές: 0=free, 100=wall, -1=unknown
        map_info  – nav_msgs/MapMetaData (resolution, origin, κλπ.)
    """

    # ── Παράμετροι εξερεύνησης ──────────────────────────────────────────────
    MIN_FRONTIER_SIZE  = 5    # ελάχιστο μέγεθος cluster σε pixels για να ληφθεί υπόψη
    NAV_TIMEOUT_SEC    = 35.0 # δευτερόλεπτα πλοήγησης πριν blacklist
    GOAL_TOLERANCE_M   = 0.40 # μέτρα — απόσταση "goal reached"
    BLACKLIST_RADIUS_M = 0.60 # μέτρα — ζώνη γύρω από μη-προσβάσιμο frontier

    def __init__(self, name, node):
        super(ExploreMazeAction, self).__init__(name)
        self.node = node

        # --- Publishers / Subscribers ---
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)

        # Θέση ρομπότ από EKF SLAM (αντί για TF lookup)
        self.current_pose = None
        self.pos_sub = self.node.create_subscription(
            PoseStamped, '/est_pos', self._pos_callback, 10
        )

        # Ακούμε τον A* Planner: στέλνει κενό path όταν ο στόχος επιτευχθεί
        self.path_empty_received = False
        self.path_sub = self.node.create_subscription(
            Path, '/plan', self._path_callback, 10
        )

        # --- Blackboard ---
        self.blackboard = py_trees.blackboard.Client(name=name + "_explore")
        self.blackboard.register_key(key="grid_map",  access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="map_info",  access=py_trees.common.Access.READ)

        # --- Εσωτερική Κατάσταση ---
        # 'FIND'     : αναζήτηση επόμενου frontier
        # 'NAVIGATE' : εν κινήσει προς frontier
        self._nav_state  = 'FIND'
        self._current_goal = None       # (world_x, world_y) του τρέχοντος στόχου
        self._blacklist    = []         # [(world_x, world_y)] – μη-προσβάσιμα frontiers
        self._nav_start_t  = None       # χρόνος έναρξης πλοήγησης (seconds)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _pos_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose.position

    def _path_callback(self, msg: Path):
        """
        Ο A* Planner στέλνει κενό Path όταν φτάσουμε στον στόχο.
        Το χρησιμοποιούμε σαν αξιόπιστο σήμα "goal reached".
        """
        if len(msg.poses) == 0 and self._nav_state == 'NAVIGATE':
            self.path_empty_received = True

    # ── py_trees lifecycle ─────────────────────────────────────────────────────

    def initialise(self):
        """
        Καλείται κάθε φορά που το behaviour ξεκινά από μη-RUNNING κατάσταση.
        Διατηρούμε το blacklist ώστε να μην ξαναπάμε σε μη-προσβάσιμα frontiers
        αν το behaviour διακόπηκε προσωρινά από το UnlockDoor.
        """
        self.node.get_logger().info("🔍 [EXPLORE] Ξεκινώ εξερεύνηση λαβυρίνθου...")
        self._nav_state         = 'FIND'
        self._current_goal      = None
        self._nav_start_t       = None
        self.path_empty_received = False

    def update(self) -> py_trees.common.Status:
        """
        Καλείται κάθε tick (1 sec). Κεντρική λογική state machine:
          NAVIGATE → ελέγχουμε αν φτάσαμε ή αν κολλήσαμε
          FIND     → βρίσκουμε και στέλνουμε νέο frontier
        """
        grid_map = self.blackboard.grid_map
        map_info = self.blackboard.map_info

        if grid_map is None or map_info is None:
            self.node.get_logger().warn("[EXPLORE] Αναμονή χάρτη...")
            return py_trees.common.Status.RUNNING

        if self.current_pose is None:
            self.node.get_logger().warn("[EXPLORE] Αναμονή θέσης ρομπότ (/est_pos)...")
            return py_trees.common.Status.RUNNING

        # ── Κατάσταση NAVIGATE ───────────────────────────────────────────────
        if self._nav_state == 'NAVIGATE':
            if self._is_goal_reached() or self.path_empty_received:
                # ✅ Φτάσαμε!
                self.node.get_logger().info("✅ [EXPLORE] Frontier επιτεύχθηκε! Ψάχνω επόμενο...")
                self._nav_state         = 'FIND'
                self._current_goal      = None
                self._nav_start_t       = None
                self.path_empty_received = False
                # Δεν κάνουμε return — αμέσως ψάχνουμε για επόμενο frontier

            elif self._is_navigation_timed_out():
                # ⚠️ Κολλήσαμε — προσθέτουμε στο blacklist
                self.node.get_logger().warn(
                    f"⚠️ [EXPLORE] Timeout ({self.NAV_TIMEOUT_SEC}s) για frontier "
                    f"({self._current_goal[0]:.2f}, {self._current_goal[1]:.2f}). "
                    f"Blacklisting και δοκιμή επόμενου..."
                )
                if self._current_goal:
                    self._blacklist.append(self._current_goal)
                self._nav_state         = 'FIND'
                self._current_goal      = None
                self._nav_start_t       = None
                self.path_empty_received = False
                # Δεν κάνουμε return — αμέσως ψάχνουμε για επόμενο frontier

            else:
                # 🚀 Εν κινήσει — δεν χρειάζεται να κάνουμε τίποτα
                return py_trees.common.Status.RUNNING

        # ── Κατάσταση FIND ────────────────────────────────────────────────────
        result = self._find_and_send_frontier(grid_map, map_info)

        if result == 'DONE':
            self.node.get_logger().info(
                "🎉 [EXPLORE] Εξερεύνηση ολοκληρώθηκε! Δεν βρέθηκαν άλλα frontiers."
            )
            return py_trees.common.Status.SUCCESS

        # result == 'SENT' — στόχος εστάλη, μεταβαίνουμε σε NAVIGATE
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        """
        Καλείται όταν το behaviour εξέρχεται από RUNNING (preempt ή completion).
        Αφήνουμε τον τρέχοντα στόχο ενεργό — ο planner θα σταματήσει μόνος του.
        """
        self.node.get_logger().info(
            f"[EXPLORE] Τερματισμός με status: {new_status.name} | "
            f"Blacklisted frontiers: {len(self._blacklist)}"
        )

    # ── Frontier Detection ─────────────────────────────────────────────────────

    def _detect_frontiers(self, grid_map: np.ndarray) -> np.ndarray:
        """
        Εντοπίζει frontier cells: ελεύθερα κελιά (==0) που γειτνιάζουν
        με τουλάχιστον ένα άγνωστο κελί (==-1).

        Επιστρέφει binary mask: uint8, 255 = frontier cell.
        """
        # Χρησιμοποιούμε int16 για ασφαλή σύγκριση με -1 (αποφυγή overflow σε uint8)
        grid = grid_map.astype(np.int16)

        free_mask    = (grid == 0  ).astype(np.uint8) * 255
        unknown_mask = (grid == -1 ).astype(np.uint8) * 255

        # Διαστολή του unknown χώρου → βρίσκουμε τα γειτονικά ελεύθερα κελιά
        kernel          = np.ones((3, 3), np.uint8)
        dilated_unknown = cv2.dilate(unknown_mask, kernel, iterations=1)

        frontier_mask = cv2.bitwise_and(free_mask, dilated_unknown)
        return frontier_mask

    def _cluster_frontiers(self, frontier_mask: np.ndarray, map_info) -> list:
        """
        Ομαδοποιεί τα frontier pixels σε clusters (connected components).
        Φιλτράρει μικρά clusters και blacklisted frontiers.

        Επιστρέφει λίστα από dicts: {cx, cy, wx, wy, size}
        """
        num_labels, _, stats, centroids = cv2.connectedComponentsWithStats(
            frontier_mask, connectivity=8
        )

        clusters = []
        for i in range(1, num_labels):  # 0 = background
            size = int(stats[i, cv2.CC_STAT_AREA])
            if size < self.MIN_FRONTIER_SIZE:
                continue

            cx, cy = float(centroids[i][0]), float(centroids[i][1])
            wx, wy = self._grid_to_world(cx, cy, map_info)

            if self._is_blacklisted(wx, wy):
                continue

            clusters.append({'cx': cx, 'cy': cy, 'wx': wx, 'wy': wy, 'size': size})

        return clusters

    def _select_best_frontier(self, clusters: list, map_info):
        """
        Βαθμολόγηση frontiers:
            score = distance_in_pixels / sqrt(cluster_size)
        Προτιμώνται μεγάλα clusters που βρίσκονται κοντά στο ρομπότ.
        Μικρότερο score = καλύτερη επιλογή.
        """
        if not clusters or self.current_pose is None:
            return None

        robot_gx, robot_gy = self._world_to_grid(
            self.current_pose.x, self.current_pose.y, map_info
        )

        best, best_score = None, float('inf')
        for c in clusters:
            dist  = math.hypot(c['cx'] - robot_gx, c['cy'] - robot_gy)
            score = dist / math.sqrt(c['size'])
            if score < best_score:
                best_score = score
                best = c

        return best

    # ── Core Logic ─────────────────────────────────────────────────────────────

    def _find_and_send_frontier(self, grid_map: np.ndarray, map_info) -> str:
        """
        Εντοπίζει, επιλέγει και στέλνει το καλύτερο frontier goal.
        Επιστρέφει:
            'DONE' – κανένα frontier δεν βρέθηκε (εξερεύνηση τελείωσε)
            'SENT' – goal εστάλη επιτυχώς
        """
        frontier_mask = self._detect_frontiers(grid_map)
        clusters      = self._cluster_frontiers(frontier_mask, map_info)

        if not clusters:
            return 'DONE'

        best = self._select_best_frontier(clusters, map_info)
        if best is None:
            return 'DONE'

        # Ορισμός νέου στόχου
        self._current_goal = (best['wx'], best['wy'])
        self._nav_start_t  = self.node.get_clock().now().nanoseconds / 1e9
        self.path_empty_received = False
        self._nav_state = 'NAVIGATE'

        # Δημοσίευση goal στον A* Planner
        goal_msg = PoseStamped()
        goal_msg.header.stamp    = self.node.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = best['wx']
        goal_msg.pose.position.y = best['wy']
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)

        self.node.get_logger().info(
            f"🎯 [EXPLORE] Νέο frontier: ({best['wx']:.2f}, {best['wy']:.2f}) | "
            f"Cluster: {best['size']} px | Blacklist: {len(self._blacklist)} | "
            f"Υπόλοιπα clusters: {len(clusters)}"
        )
        return 'SENT'

    # ── Helper Methods ─────────────────────────────────────────────────────────

    def _is_goal_reached(self) -> bool:
        if self._current_goal is None or self.current_pose is None:
            return False
        dx = self._current_goal[0] - self.current_pose.x
        dy = self._current_goal[1] - self.current_pose.y
        return math.hypot(dx, dy) < self.GOAL_TOLERANCE_M

    def _is_navigation_timed_out(self) -> bool:
        if self._nav_start_t is None:
            return False
        elapsed = self.node.get_clock().now().nanoseconds / 1e9 - self._nav_start_t
        return elapsed > self.NAV_TIMEOUT_SEC

    def _is_blacklisted(self, wx: float, wy: float) -> bool:
        for bx, by in self._blacklist:
            if math.hypot(wx - bx, wy - by) < self.BLACKLIST_RADIUS_M:
                return True
        return False

    def _world_to_grid(self, x: float, y: float, map_info) -> tuple[float, float]:
        gx = (x - map_info.origin.position.x) / map_info.resolution
        gy = (y - map_info.origin.position.y) / map_info.resolution
        return gx, gy

    def _grid_to_world(self, gx: float, gy: float, map_info) -> tuple[float, float]:
        wx = gx * map_info.resolution + map_info.origin.position.x
        wy = gy * map_info.resolution + map_info.origin.position.y
        return wx, wy


# ==========================================
# 3. ΔΕΝΤΡΟ ΣΥΜΠΕΡΙΦΟΡΩΝ
# ==========================================

def create_root(node):
    # Κεντρική Ακολουθία: Πρώτα Spin (μια φορά) -> Μετά Αποστολές
    root = py_trees.composites.Sequence(name="Main_Mission", memory=True)

    spin_action  = InitialSpinAction(name="Action: 360 Spin", node=node)
    spin_oneshot = py_trees.decorators.OneShot(
        child=spin_action,
        name="OneShot_Protection",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    mission_selector = py_trees.composites.Selector(name="Mission_Priorities", memory=False)

    unlock_sequence = py_trees.composites.Sequence(name="Unlock_Door_Priority", memory=False)
    check_match     = CheckForUnlockableDoor(name="Condition: Έχουμε κλειδί για γνωστή πόρτα;")
    unlock_door     = UnlockDoorAction(name="Action: Άνοιξε Πόρτα", node=node)

    unlock_sequence.add_children([check_match, unlock_door])

    explore = ExploreMazeAction(name="Action: Εξερεύνηση", node=node)

    mission_selector.add_children([unlock_sequence, explore])

    root.add_children([spin_oneshot, mission_selector])
    return root


# ==========================================
# 4. ΚΟΜΒΟΣ ΕΛΕΓΧΟΥ ΑΠΟΣΤΟΛΗΣ
# ==========================================

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')

        self.aruco_sub = self.create_subscription(
            Point, '/vision/detected_aruco', self.aruco_callback, 10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        self.marker_pub = self.create_publisher(MarkerArray, '/door_markers', 10)
        self.cloud_pub  = self.create_publisher(PointCloud2, '/dynamic_doors_cloud', 10)
        self.vis_timer  = self.create_timer(0.5, self.publish_dynamic_obstacles)

        self.raw_door_posts       = {}
        self.DOOR_WIDTH_THRESHOLD = 0.2

        self.blackboard = py_trees.blackboard.Client(name="Master")
        self.blackboard.register_key(key="keys_inventory",  access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="discovered_doors",access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="target_door",     access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="unlocked_doors",  access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="grid_map",        access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="map_info",        access=py_trees.common.Access.WRITE)

        self.blackboard.grid_map        = None
        self.blackboard.map_info        = None
        self.blackboard.keys_inventory  = []
        self.blackboard.discovered_doors= {}
        self.blackboard.target_door     = None
        self.blackboard.unlocked_doors  = []

        self.tree = py_trees.trees.BehaviourTree(create_root(self))
        self.tree.setup(timeout=15)

        self.timer = self.create_timer(1.0, self.tick_tree)

    def aruco_callback(self, msg):
        detected_id = int(msg.z)
        x = float(msg.x)
        y = float(msg.y)

        if detected_id in KEY_DOOR_MATCHES.values():
            if detected_id not in self.blackboard.keys_inventory:
                self.get_logger().info(f"📥 [VISION] Βρήκα ΚΛΕΙΔΙ: {detected_id}")
                self.blackboard.keys_inventory.append(detected_id)

        elif detected_id in KEY_DOOR_MATCHES.keys():
            if detected_id in self.blackboard.unlocked_doors or detected_id in self.blackboard.discovered_doors:
                return

            if detected_id not in self.raw_door_posts:
                self.raw_door_posts[detected_id] = [(x, y)]
                self.get_logger().info(f"🔍 [VISION] Εντοπίστηκε η 1η κολώνα της ΠΟΡΤΑΣ {detected_id} στα ({x:.2f}, {y:.2f}). Ψάχνω την 2η...")
            else:
                first_post = self.raw_door_posts[detected_id][0]
                dist = math.hypot(x - first_post[0], y - first_post[1])

                if dist > self.DOOR_WIDTH_THRESHOLD and len(self.raw_door_posts[detected_id]) == 1:
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
        points   = []
        marker_id = 0

        for door_id, (mid_x, mid_y) in self.blackboard.discovered_doors.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp    = self.get_clock().now().to_msg()
            marker.ns      = "locked_doors"
            marker.id      = marker_id
            marker.type    = Marker.CUBE
            marker.action  = Marker.ADD

            marker.pose.position.x = float(mid_x)
            marker.pose.position.y = float(mid_y)
            marker.pose.position.z = 0.5

            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 1.0

            marker.color.a = 0.8
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker_array.markers.append(marker)
            marker_id += 1

            points.extend([
                [float(mid_x),        float(mid_y),        0.0],
                [float(mid_x) + 0.15, float(mid_y),        0.0],
                [float(mid_x) - 0.15, float(mid_y),        0.0],
                [float(mid_x),        float(mid_y) + 0.15, 0.0],
                [float(mid_x),        float(mid_y) - 0.15, 0.0],
            ])

        self.marker_pub.publish(marker_array)

        header    = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.cloud_pub.publish(cloud_msg)

    def map_callback(self, msg):
        width  = msg.info.width
        height = msg.info.height
        # Αποθηκεύουμε ως int16 για ασφαλή χειρισμό του -1 (unknown)
        self.blackboard.grid_map = np.array(msg.data, dtype=np.int16).reshape((height, width))
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