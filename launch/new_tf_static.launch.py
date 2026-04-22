import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder

def generate_launch_description():
    tag_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.5", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "tag36h11:15"],
    )

    camera_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["-1.0", "0.0", "0.4", "1.57", "1.57", "0.0", "tag36h11:15", "camera_link"],
    )

    depth_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "camera_link", "camera_depth_frame"],
    )

    depth_optical_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "-1.57", "1.57", "0.0", "camera_depth_frame", "camera_depth_optical_frame"],
    )

    color_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "-0.059", "-0.215", "0.0", "0.0", "0.0", "camera_link", "camera_color_frame"],
    )

    optical_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "-1.57", "1.57", "0.0", "camera_color_frame", "camera_color_optical_frame"],
    )

    return LaunchDescription([
        tag_frame,
        camera_frame,
        depth_frame,
        depth_optical_frame,
        color_frame,
        optical_frame,
    ])