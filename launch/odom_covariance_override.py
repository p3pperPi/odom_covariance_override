from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="odom_covariance_override",
            executable="odom_covariance_override",
            name="odom_covariance_override",
            output="screen",
            parameters=[os.path.join(get_package_share_directory("odom_covariance_override"), 'params', 'config.yaml')]
        ),
    ])