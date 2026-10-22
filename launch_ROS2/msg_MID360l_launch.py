import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.normpath(
        os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', 'config', 'MID360l_config.json')
    )
    return LaunchDescription([
        DeclareLaunchArgument('user_config_path', default_value=config_path),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            parameters=[
                {'xfer_format': 1},
                {'multi_topic': 0},
                {'data_src': 0},
                {'publish_freq': 10.0},
                {'output_data_type': 0},
                {'frame_id': 'livox_frame'},
                {'lvx_file_path': '/home/livox/livox_test.lvx'},
                {'user_config_path': LaunchConfiguration('user_config_path')},
                {'cmdline_input_bd_code': 'livox0000000001'},
            ],
        ),
    ])
