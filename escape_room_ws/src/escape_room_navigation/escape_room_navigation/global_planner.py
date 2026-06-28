#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
import heapq
import math
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        
        # 1. Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.inf_map_pub = self.create_publisher(OccupancyGrid, '/infmap', 10)
        
        # ΑΛΛΑΓΗ: Αντικατάσταση του TF Listener με Subscriber στο /est_pos
        self.pos_sub = self.create_subscription(PoseStamped, '/est_pos', self.pos_callback, 10)
        # ΑΛΛΑΓΗ: Ακούμε το Mission Control για τις κλειδωμένες πόρτες (Εικονικά εμπόδια)
        self.door_sub = self.create_subscription(PointCloud2, '/dynamic_doors_cloud', self.door_callback, 10)
        
        # 2. Publisher (Το μονοπάτι)
        self.path_pub = self.create_publisher(Path, '/plan', 10)


        # Προσθήκη Timer που τρέχει τον Planner συνεχώς κάθε 1 δευτερόλεπτο
        self.replanning_timer = self.create_timer(2.0, self.try_plan)
        
        # Εσωτερικές μεταβλητές
        self.grid_map = None
        self.map_data = None
        self.goal_pose = None
        self.current_pose = None # Αποθήκευση της τρέχουσας θέσης του ρομπότ
        
        self.inflation_radius_pixels = 2

        self.get_logger().info("🚀 A* Global Planner initialized. Waiting for Map, Robot Pose (/est_pos) and Goal...")

    def map_callback(self, msg):
        self.map_data = msg
        width = msg.info.width
        height = msg.info.height
        
        grid = np.array(msg.data).reshape((height, width))
       # Δημιουργία χάρτη εμποδίων (Εμπόδιο γίνεται ο τοίχος (>60) Ή το άγνωστο (-1))
        obstacle_map = np.zeros_like(grid, dtype=np.uint8)
        obstacle_map[(grid > 60) | (grid == -1)] = 255
        
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.inflation_radius_pixels*2, self.inflation_radius_pixels*2))
        inflated_map = cv2.dilate(obstacle_map, kernel, iterations=1)
        
        self.grid_map = inflated_map
        self.publish_inflated_map(self.grid_map, msg.info)
        self.get_logger().info("Map received and inflated!")
        self.try_plan()

    def goal_callback(self, msg):
        self.goal_pose = msg.pose.position
        self.get_logger().info("🎯 Goal pose received. Planning path from current estimated position...")
        self.try_plan()
    
    def door_callback(self, msg):
        """ Ζωγραφίζει τις κλειδωμένες πόρτες από το Mission Control πάνω στον χάρτη του A* """
        if self.grid_map is None:
            return
            
        # Διαβάζουμε τα σημεία (X, Y) από το PointCloud
        for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            # Μετατρέπουμε τα πραγματικά μέτρα σε pixels του χάρτη
            idx = self.world_to_grid(p[0], p[1])
            
            # Αν τα pixels είναι εντός χάρτη, τα κάνουμε απόλυτο εμπόδιο (255)
            if 0 <= idx[0] < self.grid_map.shape[1] and 0 <= idx[1] < self.grid_map.shape[0]:
                self.grid_map[idx[1], idx[0]] = 255

    # ΑΛΛΑΓΗ: Callback για την ανανέωση της θέσης του ρομπότ από το EKF SLAM
    def pos_callback(self, msg):
        self.current_pose = msg.pose.position

    def world_to_grid(self, x, y):
        res = self.map_data.info.resolution
        orig_x = self.map_data.info.origin.position.x
        orig_y = self.map_data.info.origin.position.y
        grid_x = int((x - orig_x) / res)
        grid_y = int((y - orig_y) / res)
        return (grid_x, grid_y)

    def grid_to_world(self, grid_x, grid_y):
        res = self.map_data.info.resolution
        orig_x = self.map_data.info.origin.position.x
        orig_y = self.map_data.info.origin.position.y
        x = (grid_x * res) + orig_x + (res / 2.0)
        y = (grid_y * res) + orig_y + (res / 2.0)
        return (x, y)

    def heuristic(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def try_plan(self):
        if self.grid_map is None or self.goal_pose is None:
            return

        # ΑΛΛΑΓΗ: Έλεγχος αν έχουμε λάβει έστω και μία φορά τη θέση από το /est_pos
        if self.current_pose is None:
            self.get_logger().warn("Waiting for a valid robot position from /est_pos...")
            return

        # Άντληση των συντεταγμένων απευθείας από την εσωτερική μεταβλητή
        robot_x = self.current_pose.x
        robot_y = self.current_pose.y

        start_idx = self.world_to_grid(robot_x, robot_y)
        goal_idx = self.world_to_grid(self.goal_pose.x, self.goal_pose.y)

        # Έλεγχος: Φτάσαμε στον στόχο; (Απόσταση < 0.20m)
        dist_to_goal = self.heuristic(start_idx, goal_idx) * self.map_data.info.resolution
        if dist_to_goal < 0.15:
            self.get_logger().info("🏁 Στόχος επετεύχθη!")
            self.goal_pose = None
            self.publish_path([]) # Στέλνουμε άδειο μονοπάτι για να σταματήσει ο local controller
            return

        # Έλεγχος αν τα όρια του grid map είναι σωστά για αποφυγή IndexError
        if not (0 <= start_idx[0] < self.grid_map.shape[1] and 0 <= start_idx[1] < self.grid_map.shape[0]):
            self.get_logger().error("Robot is out of map bounds! Cannot plan.")
            self.goal_pose = None
            return

        if not (0 <= goal_idx[0] < self.grid_map.shape[1] and 0 <= goal_idx[1] < self.grid_map.shape[0]):
            self.get_logger().error("Goal position is out of map bounds!")
            self.goal_pose = None
            return

        # Έλεγχος αν το start ή το goal είναι μέσα σε τοίχο
        # if self.grid_map[start_idx[1], start_idx[0]] == 255:
        #     self.get_logger().error("Robot is currently inside an obstacle! Cannot plan.")
        #     self.goal_pose = None
        #     return
        # if self.grid_map[goal_idx[1], goal_idx[0]] == 255:
        #     self.get_logger().error("Goal position is inside an obstacle!")
        #     self.goal_pose = None
        #     return

        self.get_logger().info(f"Planning from ({robot_x:.2f}, {robot_y:.2f}) to ({self.goal_pose.x:.2f}, {self.goal_pose.y:.2f})...")
        
        # Εκτέλεση A*
        path_indices = self.a_star(start_idx, goal_idx)
        
        if path_indices:
            self.publish_path(path_indices)
            self.get_logger().info(f"✅ Path found! Length: {len(path_indices)} waypoints.")
        else:
            self.get_logger().error("❌ A* could not find a valid path.")
            # self.publish_path([])
            
        # Καθαρισμός του goal για να περιμένει νέα εντολή
        ##self.goal_pose = None

    def a_star(self, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0), (1,1),(-1,1),(1,-1),(-1,-1)] 
        
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
                return data[::-1] 
                
            close_set.add(current)
            
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                
                if 0 <= neighbor[1] < self.grid_map.shape[0]:
                    if 0 <= neighbor[0] < self.grid_map.shape[1]:
                        if self.grid_map[neighbor[1]][neighbor[0]] == 255:
                            continue
                    else:
                        continue
                else:
                    continue
                
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

    def publish_inflated_map(self, inflated_grid, original_map_info):
            """
            Παίρνει το 2D numpy array (inflated_grid) και τις πληροφορίες 
            του αρχικού χάρτη (resolution, width, height, origin) και τα στέλνει.
            """
            inf_msg = OccupancyGrid()
            inf_msg.header.stamp = self.get_clock().now().to_msg()
            inf_msg.header.frame_id = 'map'
            inf_msg.info = original_map_info
            
            # Το ROS 2 OccupancyGrid απαιτεί μια μονοδιάστατη λίστα (1D) από int8.
            # Οπότε κάνουμε flatten() τον 2D πίνακα και τον μετατρέπουμε.
            inf_msg.data = inflated_grid.flatten().astype(np.int8).tolist()
            
            self.inf_map_pub.publish(inf_msg)
            self.get_logger().info("Δημοσιεύτηκε ο Inflated Map στο /infmap")

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()