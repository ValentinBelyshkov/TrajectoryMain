from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='image_republish',
            arguments=['raw', 'compressed', '--ros-args'],
            remappings=[
                ('in', '/camera/image_raw'),
                ('out', '/camera/image_compressed')
            ],
            parameters=[
                {'queue_size': 10},
                {'buff_size': 2**24}  # большой буфер для больших изображений
            ],
            output='screen'
        )
    ])
