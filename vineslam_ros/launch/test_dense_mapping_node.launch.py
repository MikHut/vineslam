import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # ------------------------------------------------------------
    # ---- Setup
    # ------------------------------------------------------------

    # Simulated time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    config_path = os.path.join(
        get_package_share_directory('vineslam_ros'),
        'config/offline',
        'setup_dense_mapping_node.yaml'
    )

    rviz_path = os.path.join(
        get_package_share_directory('vineslam_ros'),
        'config/offline',
        'test_dense_mapping_node.rviz')

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    ld = LaunchDescription()

    log_level = DeclareLaunchArgument("log_level", default_value=["info"], description="Logging level")
    ld.add_action(log_level)

    # ------------------------------------------------------------
    # ---- Declare ros nodes
    # ------------------------------------------------------------

    # VineSLAM node
    logger = LaunchConfiguration("log_level")
    vineslam = Node(
        package='vineslam_ros',
        executable='dense_mapping_node',
        name='dense_mapping_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', logger]
    )
    ld.add_action(vineslam)

    # Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path, '--ros-args', '--log-level', 'INFO'],
    )
    ld.add_action(rviz)

    return ld
