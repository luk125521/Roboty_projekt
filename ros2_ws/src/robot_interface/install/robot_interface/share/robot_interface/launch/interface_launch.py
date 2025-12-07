# interface_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_interface',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{'camera_index': 0}]
        ),
        Node(
            package='robot_interface',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen'
        ),
        Node(
            package='robot_interface',
            executable='turtle_controller',
            name='turtle_controller',
            output='screen'
        ),
        Node(
            package='robot_interface',
            executable='image_viewer',
            name='image_viewer',
            output='screen'
        ),
    ])
