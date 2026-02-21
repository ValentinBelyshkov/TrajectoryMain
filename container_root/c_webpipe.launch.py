from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_transport',
            executable='republish',
            name='image_republish',
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/camera/image_raw'),
                ('out', '/camera/image_raw_comp')
            ],
            output='screen'
        )
    ])
