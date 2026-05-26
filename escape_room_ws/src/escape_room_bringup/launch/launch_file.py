import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # 1. Βρίσκουμε πού είναι εγκατεστημένα τα εργοστασιακά πακέτα
    myagv_odometry_dir = get_package_share_directory('myagv_odometry')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')

    # 2. Φορτώνουμε το Launch file του ρομπότ (Οδομετρία + Μοτέρ + TF Tree)
    # Προσοχή: Ελέγξτε αν το αρχείο λέγεται 'myagv_active.launch.py' στον φάκελο myagv_odometry/launch
    myagv_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(myagv_odometry_dir, 'launch', 'myagv_active.launch.py')
        )
    )

    # 3. Φορτώνουμε το Launch file του YDLidar
    # Προσοχή: Ελέγξτε αν το αρχείο λέγεται 'ydlidar_launch.py' στον φάκελο ydlidar_ros2_driver/launch
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')
        )
    )

    # 4. Τα πακετάρουμε όλα μαζί και τα επιστρέφουμε στο ROS2 για να τα τρέξει
    return LaunchDescription([
        myagv_base_launch,
        ydlidar_launch
    ])