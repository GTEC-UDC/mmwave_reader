from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Default radar config path
    default_radar_config = os.path.join(
        get_package_share_directory('gtec_mmwave_reader'),
        'example_radar_configs',
        'tracking_people_6m_6843ISK.cfg'
    )

    return LaunchDescription([
        SetEnvironmentVariable('PYTHONUNBUFFERED', '1'),

        DeclareLaunchArgument(
            'uart_port',
            default_value='/dev/ttyUSB0',
            description='UART port of the radar'
        ),
        DeclareLaunchArgument(
            'data_port',
            default_value='/dev/ttyUSB1',
            description='DATA port of the radar'
        ),
        DeclareLaunchArgument(
            'config_file_path',
            default_value=default_radar_config,
            description='Path to the radar configuration file'
        ),

        DeclareLaunchArgument('radar_id', default_value='isk'),
        DeclareLaunchArgument('radar_pos_x', default_value='0.0'),
        DeclareLaunchArgument('radar_pos_y', default_value='0.0'),
        DeclareLaunchArgument('radar_pos_z', default_value='1.0'),
        DeclareLaunchArgument('radar_yaw', default_value='0.0'),
        DeclareLaunchArgument('radar_pitch', default_value='0.0'),
        DeclareLaunchArgument('radar_roll', default_value='0.0'),
        DeclareLaunchArgument('elev_tilt', default_value='5.0'),

        Node(
            package='gtec_mmwave_reader',
            executable='people_counting_reader',
            name='isk_people_counting_reader',
            output='screen',
            parameters=[
                {'uart_port': LaunchConfiguration('uart_port')},
                {'data_port': LaunchConfiguration('data_port')},
                {'config_file_path': LaunchConfiguration('config_file_path')},
                {'publish_radar_topic': '/radar_scan'},
                {'publish_target_topic': '/target'},
                {'publish_cloud_topic': '/cloud'},
                {'publish_all_target_topic': '/all_targets'},
                {'sensor_height': LaunchConfiguration('radar_pos_z')},
                {'sensor_x': LaunchConfiguration('radar_pos_x')},
                {'sensor_y': LaunchConfiguration('radar_pos_y')},
                {'elev_tilt': LaunchConfiguration('elev_tilt')},
                {'radar_pitch': LaunchConfiguration('radar_pitch')},
                {'radar_yaw': LaunchConfiguration('radar_yaw')},
                {'radar_roll': LaunchConfiguration('radar_roll')},
                {'radar_id': LaunchConfiguration('radar_id')},
                {'boundary_box_x_min': -10.0},
                {'boundary_box_y_min': -10.0},
                {'boundary_box_z_min': -10.0},
                {'boundary_box_x_max': 10.0},
                {'boundary_box_y_max': 10.0},
                {'boundary_box_z_max': 10.0}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),

        # odom > radar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_radar',
            arguments=[
                '--x', LaunchConfiguration('radar_pos_x'),
                '--y', LaunchConfiguration('radar_pos_y'),
                '--z', LaunchConfiguration('radar_pos_z'),
                '--yaw', LaunchConfiguration('radar_yaw'),
                '--pitch', LaunchConfiguration('radar_pitch'),
                '--roll', LaunchConfiguration('radar_roll'),
                '--frame-id', 'odom',
                '--child-frame-id', LaunchConfiguration('radar_id')
            ]
        ),

        # map > odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--yaw', '0',
                '--pitch', '0',
                '--roll', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'odom'
            ]
        )
    ]) 