#!/bin/bash
echo "🚀 Εκκίνηση Συστήματος Escape Room..."

# Tab 1: Ο Κόσμος του ROS1 (Οδηγός Lidar)
# Σημείωση: Αντικατέστησε το "roslaunch ydlidar..." με την ακριβή εντολή που τρέχετε για το Lidar στο ROS1
gnome-terminal --tab --title="1. Lidar (ROS1)" -- bash -c "
echo 'Φόρτωση ROS1...';
source /opt/ros/noetic/setup.bash;
roscore &
sleep 5;
echo 'Εκκίνηση Lidar...';
source /home/er/Desktop/Projects/myagv_ros/devel/setup.bash
roslaunch ydlidar_ros_driver X2.launch;
exec bash"

gnome-terminal --tab --title="1.1 Odometry (ROS1)" -- bash -c "
echo 'Φόρτωση ROS1...';
source /opt/ros/noetic/setup.bash;
roscore &
sleep 15;
echo 'Εκκίνηση Odom...';
source /home/er/Desktop/Projects/myagv_ros/devel/setup.bash
roslaunch myagv_odometry myagv_active.launch
exec bash"

# Tab 2: Η Γέφυρα (ros1_bridge)
gnome-terminal --tab --title="2. Bridge (ROS1+ROS2)" -- bash -c "
echo 'Περιμένω το Lidar να ξεκινήσει...';
sleep 20;
echo 'Φόρτωση ROS1 & ROS2...';
source /opt/ros/noetic/setup.bash;
source /opt/ros/galactic/setup.bash;
echo 'Εκκίνηση Γέφυρας...';
rosparam load /home/er/Desktop/Projects/TeamA06/RS1-project-escape-room-robot/escape_room_ws/bridge.yaml;
ros2 run ros1_bridge parameter_bridge;
exec bash"

# Tab 3: Ο Εγκέφαλος ROS2 (TF, Converter, RViz)
gnome-terminal --tab --title="3. Brain (ROS2)" -- bash -c "
echo 'Περιμένω τη Γέφυρα...';
sleep 25;
echo 'Καθαρισμός περιβάλλοντος και φόρτωση ROS2...';
# unset PYTHONPATH;
source /opt/ros/galactic/setup.bash;
source ~/Desktop/Projects/TeamA06/RS1-project-escape-room-robot/escape_room_ws/install/setup.bash;
echo 'Εκκίνηση Launch File...';
ros2 launch escape_room_bringup robot_brain.launch.py;
exec bash"

echo "✅ Τα τερματικά άνοιξαν στο παρασκήνιο!"
