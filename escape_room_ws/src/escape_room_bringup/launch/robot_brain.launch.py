import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # --- 1. ΦΟΡΤΩΣΗ ΤΟΥ URDF ---
    # Βρίσκουμε πού έχει εγκατασταθεί το πακέτο myagv_description
    myagv_desc_dir = get_package_share_directory('myagv_description')
    urdf_path = os.path.join(myagv_desc_dir, 'urdf', 'myAGV.urdf')

    # Διαβάζουμε το περιεχόμενο του αρχείου URDF ως κείμενο
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # Ο κεντρικός κόμβος που μετατρέπει το URDF σε TF Tree και 3D Μοντέλο
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # --- 2. ΤΑ ΥΠΟΛΟΙΠΑ NODES ΜΑΣ ---
    lidar_converter_node = Node(
        package='escape_room_lidar',
        executable='lidar_converter',
        name='lidar_to_cartesian_node',
        output='screen'
    )

    vision_node = Node(
        package='escape_room_vision',
        executable='aruco_node',
        name='aruco_detector',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        lidar_converter_node,
        vision_node,
        rviz_node
    ])