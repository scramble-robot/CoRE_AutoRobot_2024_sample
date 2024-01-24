from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='corejp_sample_detector',
            executable='stop_sign_detector',
            name='stop_sign_detector',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
        ),
    ])