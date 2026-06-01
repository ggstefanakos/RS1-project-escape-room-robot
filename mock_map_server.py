#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import os

class MockMapServer(Node):
    def __init__(self):
        super().__init__('mock_map_server')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        
        # Βάλε το σωστό μονοπάτι για τον χάρτη σου
        self.map_path = os.path.join(os.path.dirname(__file__),'GIA_ANAFORA/myagv_navigation2/map/map.pgm')
        self.grid_msg = self.load_map()
        
        # Εκπέμπουμε τον χάρτη κάθε 2 δευτερόλεπτα για να τον δει σίγουρα ο A* και το RViz
        if self.grid_msg:
            self.timer = self.create_timer(2.0, self.publish_map)
            self.get_logger().info("Ο ψεύτικος Map Server ξεκίνησε! Εκπέμπει στο /map...")

    def load_map(self):
        img = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            self.get_logger().error(f"Δεν βρέθηκε ο χάρτης στο: {self.map_path}")
            return None
            
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.05  # 5 εκατοστά ανά pixel (κλασικό στο ROS)
        grid.info.width = img.shape[1]
        grid.info.height = img.shape[0]
        # Κεντράρισμα του χάρτη
        grid.info.origin.position.x = -(img.shape[1] * 0.05) / 2.0
        grid.info.origin.position.y = -(img.shape[0] * 0.05) / 2.0
        
        # Μετατροπή pixel σε Occupancy (0 = Ελεύθερο, 100 = Τοίχος)
        ros_data = np.zeros_like(img, dtype=np.int8)
        ros_data[img < 200] = 100  # Μαύρα pixels = Τοίχοι
        ros_data[img >= 200] = 0   # Λευκά pixels = Ελεύθερα
        
        grid.data = ros_data.flatten().tolist()
        return grid

    def publish_map(self):
        self.grid_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.grid_msg)

def main():
    rclpy.init()
    node = MockMapServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()