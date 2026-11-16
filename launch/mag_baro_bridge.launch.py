from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value='14571'),
        DeclareLaunchArgument('frame_id', default_value='base_link'),
        DeclareLaunchArgument('namespace', default_value='px4'),
        Node(
            package='mag_baro_bridge',
            executable='mag_baro_node',
            name='mag_baro_bridge',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'udp_port': LaunchConfiguration('udp_port'),
                'frame_id': LaunchConfiguration('frame_id'),
                'frd_to_flu': True,
            }],
        ),
    ])
