from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='corejp_sample_commander',
            executable='commander',
            name='commander',
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('corejp_sample_commander'),
                    'launch',
                    'joy.launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('corejp_sample_controller'),
                    'launch',
                    'controller.launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('corejp_sample_detector'),
                    'launch',
                    'detector.launch.py'
                ])
            ]),
        ),
    ])