import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ------------------------------------------------------------
    # ---- Setup
    # ------------------------------------------------------------

    # Simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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

    ld = LaunchDescription()

    # ------------------------------------------------------------
    # ---- Declare ros nodes
    # ------------------------------------------------------------

    # Tf transformations
    tf1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cam2base',
        arguments=['0.000', '0.350', '0.90', '0.000', '0.000', '0.707', '0.707', 'base_link',
                   config['slam_node']['camera_sensor_frame']]
    )
    ld.add_action(tf1)
    tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne2base',
        arguments=['0.000', '0.000', '0.650', '-0.000', '0.000', '-0.000', '1.000', 'base_link',
                   config['slam_node']['lidar_sensor_frame']]
    )
    ld.add_action(tf2)

    # VineSLAM node
    vineslam = Node(
        package='vineslam_ros',
        executable='slam_node',
        name='slam_node',
        parameters=[config],
        remappings=[
            ('/odom_topic', '/agrobv18/husky_velocity_controller/odom'),
            ('/gps_topic', '/agrobv18/gps2/fix'),
            ('/imu_topic', '/agrobv18/imu_um6/rpy'),
            ('/imu_data_topic', '/agrobv18/imu_um6/data'),
            ('/features_topic', '/image_feature_array'),
            ('/detections_topic', '/tpu/detections'),
            ('/scan_topic', '/agrobv18/velodyne_points'),
            ('/occupancy_map', '/map')
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )
    ld.add_action(vineslam)

    if config['slam_node']['use_semantic_features'] == True or config['slam_node']['use_image_features']:
        image_topic = '/rgb_left_depth_publisher/color/image'

        # Image republish
        republish = Node(
            package='image_transport',
            executable='republish',
            name='republish',
            arguments=['compressed', 'in/compressed:=' + image_topic + '/compressed', 'raw', 'out:=' + image_topic]
        )
        ld.add_action(republish)

    if config['slam_node']['use_semantic_features']:
        # Detector node

        detector = Node(
            package='object_detection',
            executable='run_detection_model',
            name='run_detection_model',
            parameters=[
                {'model_file': '/home/andresaguiar/ROS/ros2_ws/src/tpu-object-detection/object_detection/models/mv1_grape/frozen_model_edgetpu.tflite'},
                {'labels_file': '/home/andresaguiar/ROS/ros2_ws/src/tpu-object-detection/object_detection/models/mv1_grape/edgetpu_cpp_model_labels.txt'},
                {'score_threshold': 0.5},
                {'tile_sizes': '300x300'},
                {'tile_overlap': 15},
                {'iou_threshold': 0.1}
            ],
            remappings=[
                ('/input_rgb_image', image_topic),
            ]
        )
        ld.add_action(detector)

    if config['slam_node']['use_image_features']:
        vfe_config_path = os.path.join(
            get_package_share_directory('vfe'),
            'config',
            'setup.yaml'
        )

        with open(vfe_config_path, 'r') as f:
            vfe_config = yaml.safe_load(f)

        # vfe node
        extractor = Node(
            package='vfe',
            executable='vfe',
            name='vfe',
            parameters=[vfe_config],
            remappings=[
                ('/input_rgb_image', image_topic),
                ('/input_depth_image', depth_topic),
            ]
        )
        ld.add_action(extractor)

    # Map server
    map_server_config_path = os.path.join(
        get_package_share_directory('vineslam_ros'),
        'config',
        'map_server_params.yaml'
    )
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[map_server_config_path])
    ld.add_action(map_server_cmd)

    # Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path, '--ros-args', '--log-level', 'INFO'],
    )
    ld.add_action(rviz)

    return ld
