#!/usr/bin/env python3
"""
Launch file for the RoboPoint Stretch Goal Follower
Launches the stretch bridge node that converts RoboPoint affordance results into goal points for Stretch robot
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for camera parameters
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Depth image topic name'
    )
    
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_frame',
        description='Camera frame ID'
    )
    
    fx_arg = DeclareLaunchArgument(
        'fx',
        default_value='525.0',
        description='Camera focal length in x direction'
    )
    
    fy_arg = DeclareLaunchArgument(
        'fy',
        default_value='525.0',
        description='Camera focal length in y direction'
    )
    
    cx_arg = DeclareLaunchArgument(
        'cx',
        default_value='319.5',
        description='Camera principal point x coordinate'
    )
    
    cy_arg = DeclareLaunchArgument(
        'cy',
        default_value='239.5',
        description='Camera principal point y coordinate'
    )
    
    # Stretch bridge node
    stretch_bridge_node = Node(
        package='robopoint_stretch_bridge',
        executable='stretch_bridge',
        name='robopoint_stretch_bridge',
        output='screen',
        parameters=[{
            'depth_topic': LaunchConfiguration('depth_topic'),
            'camera_frame': LaunchConfiguration('camera_frame'),
            'fx': LaunchConfiguration('fx'),
            'fy': LaunchConfiguration('fy'),
            'cx': LaunchConfiguration('cx'),
            'cy': LaunchConfiguration('cy'),
        }],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        depth_topic_arg,
        camera_frame_arg,
        fx_arg,
        fy_arg,
        cx_arg,
        cy_arg,
        stretch_bridge_node
    ])