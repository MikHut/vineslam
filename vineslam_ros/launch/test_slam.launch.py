import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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
        arguments=['0.343', '0.079', '0.820', '-0.002', '0.100', '-0.004', '0.995', 'base_link',
                   'zed_camera_left_optical_frame']
    )
    ld.add_action(tf1)
    tf2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='velodyne2base',
        arguments=['0.000', '0.000', '0.942', '-0.000', '0.000', '-0.000', '1.000', 'base_link', 'velodyne']
    )
    ld.add_action(tf2)

    # VineSLAM node
    vineslam = Node(
        package='vineslam_ros',
        executable='slam_node',
        name='slam_node',
        parameters=[config],
        remappings=[
            ('/odom_topic', '/white/husky_velocity_controller/odom'),
            ('/gps_topic', '/white/piksi/enu_pose_best_fix'),
            ('/imu_topic', '/white/imu_7/rpy'),
            ('/features_topic', '/image_feature_array'),
            ('/detections_topic', '/tpu/detections'),
            ('/scan_topic', '/white/velodyne_points'),
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )
    ld.add_action(vineslam)

    if config['slam_node']['use_semantic_features'] == True or config['slam_node']['use_image_features']:

        depth_topic = '/zed/zed_node/depth/depth_registered'
        image_topic = '/zed/zed_node/left/image'

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
                {'model_file': '/home/andresaguiar/ROS/catkin_ws_ros2/src/tpu-object-detection/object_detection/models/mv1/edgetpu_cpp_model_output_tflite_graph_edgetpu.tflite'},
                {'labels_file': '/home/andresaguiar/ROS/catkin_ws_ros2/src/tpu-object-detection/object_detection/models/mv1/edgetpu_cpp_model_labels.txt'}
            ],
            remappings=[
                ('/input_rgb_image', image_topic),
                ('/input_depth_image', depth_topic),
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

    # Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path, '--ros-args', '--log-level', 'INFO'],
    )
    ld.add_action(rviz)


    return ld