from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="gnss_odom_publisher",
            executable="gnss_odom_publisher",
            name="gnss_odom_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"publish_topic_name": "hoge"}
            ]
        )
    ])