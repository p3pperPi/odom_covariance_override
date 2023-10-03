from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="gnss_odom_publisher",
            executable="gnss_odom_publisher",
            name="gnss_odom_publisher",
            output="screen",
            parameters=[os.path.join(get_package_share_directory("gnss_odom_publisher"), 'params', 'config.yaml')]
        ),
    ])