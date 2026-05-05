from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    cloud_world = Node(
        package="franka_moveit",
        executable="cloud_transform",
        output="screen",
        parameters=[{
            "input_topic": "/points_filtered",
            "output_topic": "/points_world",
        }],
        emulate_tty=True
    )

    return LaunchDescription([cloud_world])