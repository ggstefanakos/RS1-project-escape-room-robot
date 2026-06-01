#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import cv2
import heapq
import math

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        
        # 1. Subscribers: Τι ακούει ο αλγόριθμος
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.start_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.start_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # 2. Publisher: Τι εκπέμπει ο αλγόριθμος (Το μονοπάτι)
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        
        # Εσωτερικές μεταβλητές
        self.grid_map = None
        self.map_data = None
        self.start_pose = None
        self.goal_pose = None
        
        # Παράμετρος: Πόσα pixels να "φουσκώσουμε" τους τοίχους (Ακτίνα ρομπότ)
        self.inflation_radius_pixels = 4 

        self.get_logger().info("A* Global Planner initialized. Waiting for Map, Start, and Goal...")

    def map_callback(self, msg):
        """Λαμβάνει τον χάρτη από το SLAM ή τον Map Server και φουσκώνει τους τοίχους"""
        self.map_data = msg
        width = msg.info.width
        height = msg.info.height
        
        # Μετατροπή του 1D array του ROS σε 2D Numpy Array
        grid = np.array(msg.data).reshape((height, width))
        
        # Δημιουργία καθαρού χάρτη εμποδίων (0=Ελεύθερο, 100=Τοίχος)
        obstacle_map = np.zeros_like(grid, dtype=np.uint8)
        obstacle_map[grid > 50] = 255  # Οι τοίχοι γίνονται λευκοί
        
        # INFLATION: Φουσκώνουμε τους τοίχους με OpenCV (Dilation) για να μην βρίσκει το ρομπότ
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.inflation_radius_pixels*2, self.inflation_radius_pixels*2))
        inflated_map = cv2.dilate(obstacle_map, kernel, iterations=1)
        
        self.grid_map = inflated_map
        self.get_logger().info("Map received and inflated!")
        self.try_plan()

    def start_callback(self, msg):
        self.start_pose = msg.pose.pose.position
        self.get_logger().info("Start pose received.")
        self.try_plan()

    def goal_callback(self, msg):
        self.goal_pose = msg.pose.position
        self.get_logger().info("Goal pose received. Calculating path...")
        self.try_plan()

    # --- ΒΟΗΘΗΤΙΚΕΣ ΣΥΝΑΡΤΗΣΕΙΣ ΜΕΤΑΤΡΟΠΗΣ ΣΥΝΤΕΤΑΓΜΕΝΩΝ ---
    def world_to_grid(self, x, y):
        """Μετατρέπει συντεταγμένες μέτρων (m) σε pixels του χάρτη"""
        res = self.map_data.info.resolution
        orig_x = self.map_data.info.origin.position.x
        orig_y = self.map_data.info.origin.position.y
        grid_x = int((x - orig_x) / res)
        grid_y = int((y - orig_y) / res)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        """Μετατρέπει pixels του χάρτη σε συντεταγμένες μέτρων (m)"""
        res = self.map_data.info.resolution
        orig_x = self.map_data.info.origin.position.x
        orig_y = self.map_data.info.origin.position.y
        x = (grid_x * res) + orig_x + (res / 2.0)
        y = (grid_y * res) + orig_y + (res / 2.0)
        return (x, y)

    # --- Ο ΚΥΡΙΟΣ ΑΛΓΟΡΙΘΜΟΣ A* ---
    def heuristic(self, a, b):
        """Ευκλείδεια απόσταση (H-cost)"""
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def try_plan(self):
        """Ελέγχει αν έχουμε όλα τα δεδομένα και τρέχει τον A*"""
        if self.grid_map is None or self.start_pose is None or self.goal_pose is None:
            return

        start_idx = self.world_to_grid(self.start_pose.x, self.start_pose.y)
        goal_idx = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        # Έλεγχος αν το start ή το goal είναι μέσα σε τοίχο
        if self.grid_map[start_idx[1], start_idx[0]] == 255:
            self.get_logger().error("Start position is inside an obstacle!")
            return
        if self.grid_map[goal_idx[1], goal_idx[0]] == 255:
            self.get_logger().error("Goal position is inside an obstacle!")
            return

        # Εκτέλεση A*
        path_indices = self.a_star(start_idx, goal_idx)
        
        if path_indices:
            self.publish_path(path_indices)
            self.get_logger().info(f"Path found! Length: {len(path_indices)} waypoints.")
        else:
            self.get_logger().error("A* could not find a path.")
            
        # Καθαρισμός του goal για να περιμένει νέα εντολή
        self.goal_pose = None

    def a_star(self, start, goal):
        """Ο πυρήνας του A* Algorithm με χρήση Priority Queue (heapq)"""
        neighbors = [(0,1),(0,-1),(1,0),(-1,0), (1,1),(-1,1),(1,-1),(-1,-1)] # 8-way movement
        
        close_set = set()
        came_from = {}
        gscore = {start: 0}
        fscore = {start: self.heuristic(start, goal)}
        oheap = []
        
        heapq.heappush(oheap, (fscore[start], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(start)
                return data[::-1] # Επιστροφή της λίστας από την αρχή στο τέλος
                
            close_set.add(current)
            
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                
                # Έλεγχος αν βγαίνουμε εκτός χάρτη
                if 0 <= neighbor[1] < self.grid_map.shape[0]:
                    if 0 <= neighbor[0] < self.grid_map.shape[1]:
                        # Έλεγχος για εμπόδιο (Τοίχος)
                        if self.grid_map[neighbor[1]][neighbor[0]] == 255:
                            continue
                    else:
                        continue
                else:
                    continue
                
                # Αν η κίνηση είναι διαγώνια, το κόστος είναι 1.414, αλλιώς 1.0
                move_cost = 1.414 if i != 0 and j != 0 else 1.0
                tentative_g_score = gscore[current] + move_cost
                
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                    
                if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
                    
        return False

    def publish_path(self, path_indices):
        """Μετατρέπει τα pixels ξανά σε μέτρα και τα κάνει publish ως Path message"""
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for idx in path_indices:
            pose = PoseStamped()
            pose.header = msg.header
            world_x, world_y = self.grid_to_world(idx[0], idx[1])
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            msg.poses.append(pose)
            
        self.path_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()