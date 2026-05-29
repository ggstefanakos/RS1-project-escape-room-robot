#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import py_trees
import time
import random

# ==========================================
# 1. ΦΤΙΑΧΝΟΥΜΕ ΤΑ "ΤΟΥΒΛΑΚΙΑ" (BEHAVIORS)
# ==========================================

class CheckForKey(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(CheckForKey, self).__init__(name)
        # Το Blackboard είναι η "Μνήμη" του δέντρου
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="has_key", access=py_trees.common.Access.READ)

    def update(self):
        """Ελέγχει αν έχουμε βρει το κλειδί. Επιστρέφει SUCCESS ή FAILURE ακαριαία."""
        if self.blackboard.has_key:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

class UnlockDoorAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(UnlockDoorAction, self).__init__(name)

    def update(self):
        """Εξομοιώνει την πλοήγηση προς την πόρτα."""
        print("[ACTION] Πλοήγηση προς την πόρτα... (Nav2 Mock)")
        time.sleep(2) # Προσομοίωση χρόνου κίνησης
        print("[ACTION] Η πόρτα ξεκλείδωσε! Αποστολή εξετελέσθη.")
        return py_trees.common.Status.SUCCESS

class ExploreMazeAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ExploreMazeAction, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name=name)
        self.blackboard.register_key(key="has_key", access=py_trees.common.Access.WRITE)

    def update(self):
        """Εξομοιώνει την εξερεύνηση. Κάποια στιγμή 'βρίσκει' τυχαία ένα κλειδί."""
        print("[ACTION] Εξερεύνηση Λαβύρινθου... Ψάχνω για ArUco Markers...")
        time.sleep(1)
        
        # Προσομοίωση: Υπάρχει 20% πιθανότητα να δει το Vision Node ένα κλειδί
        if random.random() < 0.2:
            print(">>> [VISION EVENT] Εντοπίστηκε Κλειδί (ArUco ID: 5)! <<<")
            self.blackboard.has_key = True
            return py_trees.common.Status.SUCCESS
            
        return py_trees.common.Status.RUNNING


# ==========================================
# 2. ΧΤΙΖΟΥΜΕ ΤΟ ΔΕΝΤΡΟ
# ==========================================

def create_root():
    # Ο Selector δοκιμάζει τα παιδιά του από πάνω προς τα κάτω. 
    # Σταματάει στο πρώτο που θα πετύχει (SUCCESS) ή που τρέχει (RUNNING).
    root = py_trees.composites.Selector(name="Mission_Priorities", memory=False)
    
    # Priority 1: Αν έχουμε κλειδί, πήγαινε άνοιξε την πόρτα
    # Το Sequence απαιτεί ΟΛΑ τα παιδιά του να πετύχουν για να προχωρήσει.
    unlock_sequence = py_trees.composites.Sequence(name="Unlock_Door_Priority", memory=False)
    check_key = CheckForKey(name="Condition: Έχουμε κλειδί;")
    unlock_door = UnlockDoorAction(name="Action: Άνοιξε Πόρτα")
    unlock_sequence.add_children([check_key, unlock_door])
    
    # Priority 2: Αλλιώς, απλά εξερεύνησε
    explore = ExploreMazeAction(name="Action: Εξερεύνηση")
    
    # Ενώνουμε τα κλαδιά στη ρίζα
    root.add_children([unlock_sequence, explore])
    
    return root

# ==========================================
# 3. Ο ROS2 ΚΟΜΒΟΣ
# ==========================================

class MissionControlNode(Node):
    def __init__(self):
        super().__init__('mission_control_node')
        self.get_logger().info("Mission Control Started.")
        
        # Αρχικοποίηση Μνήμης
        self.blackboard = py_trees.blackboard.Client(name="Master")
        self.blackboard.register_key(key="has_key", access=py_trees.common.Access.WRITE)
        self.blackboard.has_key = False
        
        self.tree = py_trees.trees.BehaviourTree(create_root())
        self.tree.setup(timeout=15)
        
        # Το δέντρο αξιολογεί την κατάσταση 2 φορές το δευτερόλεπτο
        self.timer = self.create_timer(0.5, self.tick_tree)

    def tick_tree(self):
        print("\n--- Νέος Κύκλος (Tick) ---")
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