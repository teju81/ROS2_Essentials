from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    base_config_path = os.path.join(get_package_share_directory('multi_robot_cpp_topics_pkg'), 'config')
    config_path = os.path.join(base_config_path, "config.yaml")
    return LaunchDescription([
        Node(
            package='multi_robot_cpp_topics_pkg',
            executable='talker',
            name='multi_robot_cpp_publisher_node',
            parameters=[config_path],
        ),
        Node(
            package='multi_robot_cpp_topics_pkg',
            executable='listener',
            name='multi_robot_subscriber_node',
            parameters=[config_path],
        )
    ])