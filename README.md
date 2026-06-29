# Οδηγίες εγκατάστασης

```bash
cd /myPath
git clone <repo_url>

cd ~/Desktop/Projects/myagv_ros/src/myagv_odometry/scripts
./start_ydlidar.sh
```

## 1ος τρόπος
Απαιτείεται φυσική σύνδεση με το ρομπότ (οθόνη και keyboard/mouse)
```bash
cd /myPath/RS1-project-escape-room-robot/escape_room_robot_ws
colcon build
source install/setup.bash
./robot_start.sh
```
Τρέχει όλους τους προαπαιτούμενους αισθητήρες του ρομπότ (lidar, odometry, camera, bridge).

## 2ος τρόπος
Μπορεί να γίνει και απομακρυσμένα (με ssh). Απαιτείται να τρέξουμε σε διαφορετικά terminal, όλα όσα περιέχονται στο `robot_start.sh`.

1. Lidar
```bash
source /opt/ros/noetic/setup.bash
export ROS_DOMAIN_ID=43
roscore &
source /home/er/Desktop/Projects/myagv_ros/devel/setup.bash
roslaunch ydlidar_ros_driver X2.launch
```

2. Οδομετρία
```bash
source /opt/ros/noetic/setup.bash
export ROS_DOMAIN_ID=43
roscore &
source /home/er/Desktop/Projects/myagv_ros/devel/setup.bash
roslaunch myagv_odometry myagv_active.launch
```

3. Γέφυρα
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/galactic/setup.bash
export ROS_DOMAIN_ID=43
rosparam load /myPath/RS1-project-escape-room-robot/escape_room_ws/bridge.yaml;
ros2 run ros1_bridge parameter_bridge
```

4. Κάμερα
```bash
source /opt/ros/galactic/setup.bash
export ROS_DOMAIN_ID=43
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0
```

5. Launch file
```bash
source /opt/ros/galactic/setup.bash
source ~/Desktop/Projects/TeamA06/RS1-project-escape-room-robot/escape_room_ws/install/setup.bash
export ROS_DOMAIN_ID=43
ros2 launch escape_room_bringup robot_brain.launch.py
```