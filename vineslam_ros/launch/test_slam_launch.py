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

    rviz_path = os.path.join(
        get_package_share_directory('vineslam_ros'),
        'config',
        'test.rviz')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    return LaunchDescription([
        # Tf transformations
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cam2base',
            arguments=['0.343', '0.079', '0.820', '-0.002', '0.100', '-0.004', '0.995', 'base_link',
                       'zed_camera_left_optical_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne2base',
            arguments=['0.000', '0.000', '0.942', '-0.000', '0.000', '-0.000', '1.000', 'base_link', 'velodyne']
        ),

        # VineSLAM node
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

        # Rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
        ),

    ])
