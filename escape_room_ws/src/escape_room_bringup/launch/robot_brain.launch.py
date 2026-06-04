from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Το TF Tree (Σπονδυλική Στήλη): Ενώνει τον χάρτη με το Lidar
    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='link_map_to_laser',
        arguments=['0', '0', '0.15', '0', '0', '0', 'map', 'laser_frame'],
        output='screen'
    )

    # 2. Ο Μετατροπέας (Διαβάζει /scan από το bridge, βγάζει PointCloud στο /lidar_points)
    lidar_converter_node = Node(
        package='escape_room_lidar',
        executable='lidar_converter',
        name='lidar_to_cartesian_node',
        output='screen'
    )

    # # 3. Το Vision Node (Το αφήνουμε έτοιμο για όταν βάλετε κάμερα)
    # vision_node = Node(
    #     package='escape_room_vision',
    #     executable='aruco_node',
    #     name='aruco_detector',
    #     output='screen'
    # )

    # 4. RViz2 (Για να μην το ανοίγεις με το χέρι)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
        # Αν αποθηκεύσεις τις ρυθμίσεις του rviz σε ένα αρχείο .rviz, μπορείς να το φορτώνεις αυτόματα εδώ:
        # arguments=['-d', '/διαδρομή/προς/το/αρχείο/config.rviz']
    )

    return LaunchDescription([
        tf_node,
        lidar_converter_node,
        # vision_node,
        rviz_node
    ])