from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package="gnss_odom_publisher",
            executable="gnss_odom_publisher",
            name="gnss_odom_publisher",
            output="screen",
            parameters=[
                {"publish_topic_name": "hoge"}
            ]
        ),
    ])