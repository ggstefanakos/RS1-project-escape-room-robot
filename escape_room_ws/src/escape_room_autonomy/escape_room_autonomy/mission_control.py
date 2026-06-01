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

# ==========================================
# 1. ΤΑ BEHAVIORS (Τουβλάκια)
# ==========================================

class CheckForKey(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckForKey, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="has_key", access=py_trees.common.Access.READ)

    def update(self):
        if self.blackboard.has_key:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE

class UnlockDoorAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(UnlockDoorAction, self).__init__(name)
        self.node = node
        
        # Publisher για να στέλνουμε τον στόχο στον A*
        self.goal_pub = self.node.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Listener για να διαβάζουμε τη θέση του ρομπότ
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        self.goal_sent = False
        
        # Η τοποθεσία της Πόρτας στον χάρτη σας (Βάλτε εδώ τις πραγματικές συντεταγμένες)
        self.target_x = 1.0 
        self.target_y = -1.0

    def initialise(self):
        """Εκτελείται ΜΟΝΟ την πρώτη φορά που το δέντρο μπαίνει σε αυτό το Node"""
        self.goal_sent = False

    def update(self):
        """Εκτελείται συνεχώς όσο το Action είναι RUNNING"""
        
        # Αν δεν έχουμε στείλει τον στόχο, τον στέλνουμε τώρα
        if not self.goal_sent:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.pose.position.x = self.target_x
            msg.pose.position.y = self.target_y
            msg.pose.orientation.w = 1.0 # Μηδενική περιστροφή
            self.goal_pub.publish(msg)
            
            self.node.get_logger().info(f"[ACTION] Ο στόχος ({self.target_x}, {self.target_y}) στάλθηκε στον A* Planner!")
            self.goal_sent = True
            return py_trees.common.Status.RUNNING

        # Αν τον έχουμε στείλει, ελέγχουμε το TF για να δούμε αν φτάσαμε
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = t.transform.translation.x
            ry = t.transform.translation.y
            dist = math.hypot(self.target_x - rx, self.target_y - ry)

            if dist < 0.20: # Αν πλησιάσαμε στα 20 εκατοστά
                self.node.get_logger().info(">>> ΠΟΡΤΑ ΞΕΚΛΕΙΔΩΘΗΚΕ! ΑΠΟΣΤΟΛΗ ΕΞΕΤΕΛΕΣΘΗ! <<<")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().info(f"Πλησιάζω στην πόρτα... Απόσταση: {dist:.2f}m")
                return py_trees.common.Status.RUNNING
                
        except TransformException:
            # Αν δεν υπάρχει TF ακόμα, απλά περιμένουμε
            return py_trees.common.Status.RUNNING

class ExploreMazeAction(py_trees.behaviour.Behaviour):
    def __init__(self, name, node):
        super(ExploreMazeAction, self).__init__(name)
        self.node = node
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="has_key", access=py_trees.common.Access.WRITE)

    def update(self):
        self.node.get_logger().info("Εξερεύνηση... Ψάχνω για ArUco...")
        
        # 10% πιθανότητα να βρούμε το κλειδί σε κάθε κύκλο (Για προσομοίωση)
        if random.random() < 0.1:
            self.node.get_logger().info("!!! [VISION] ΒΡΗΚΑ ΤΟ ΚΛΕΙΔΙ (ArUco ID: 5) !!!")
            self.blackboard.has_key = True
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING

# ==========================================
# 2. ΧΤΙΣΙΜΟ ΔΕΝΤΡΟΥ & ΚΟΜΒΟΣ
# ==========================================

def create_root(node):
    root = py_trees.composites.Selector(name="Mission_Priorities", memory=False)
    
    # Priority 1: Έχουμε κλειδί -> Πάμε στην πόρτα
    unlock_sequence = py_trees.composites.Sequence(name="Unlock_Door_Priority", memory=False)
    check_key = CheckForKey(name="Condition: Έχουμε κλειδί;")
    unlock_door = UnlockDoorAction(name="Action: Άνοιξε Πόρτα", node=node)
    unlock_sequence.add_children([check_key, unlock_door])
    
    # Priority 2: Αλλιώς -> Εξερεύνηση
    explore = ExploreMazeAction(name="Action: Εξερεύνηση", node=node)
    
    root.add_children([unlock_sequence, explore])
    return root

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        
        # Αρχικοποίηση Blackboard (Μνήμη)
        self.blackboard = py_trees.blackboard.Client(name="Master")
        self.blackboard.register_key(key="has_key", access=py_trees.common.Access.WRITE)
        self.blackboard.has_key = False
        
        # Περνάμε το ROS2 node (self) μέσα στο δέντρο για να μπορεί να εκπέμπει μηνύματα
        self.tree = py_trees.trees.BehaviourTree(create_root(self))
        self.tree.setup(timeout=15)
        
        self.timer = self.create_timer(1.0, self.tick_tree)

    def tick_tree(self):
        self.tree.tick()

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