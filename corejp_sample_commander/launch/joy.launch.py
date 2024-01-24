from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            emulate_tty=True,
            output='screen'
        ),
        Node(
            package='corejp_sample_commander',
            executable='joy',
            name='ui',
            emulate_tty=True,
            output='screen'
        ),
    ])