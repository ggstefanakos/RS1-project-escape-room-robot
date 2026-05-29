from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Node για την αναγνώριση ArUco
    vision_node = Node(
        package='escape_room_vision',
        executable='aruco_node',
        name='aruco_detector',
        output='screen'
    )

    # (Εδώ στο μέλλον θα μπούνε το SLAM, το Nav2 και το Autonomy Tree)

    return LaunchDescription([
        vision_node
    ])