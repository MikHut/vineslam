import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('vineslam_ros'),
        'config',
        'setup_slam.yaml'
    )

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return LaunchDescription([
        Node(
            package='vineslam_ros',
            executable='slam_node',
            name='slam_node',
            parameters=[config],
            remappings=[
                ('/odom_topic', '/husky_velocity_controller/odom'),
                ('/gps_topic', '/fix'),
                ('/features_topic', '/image_feature_array'),
                ('/detections_topic', '/tpu/detections'),
                ('/scan_topic', '/velodyne_points'),
            ]
        ),
    ])