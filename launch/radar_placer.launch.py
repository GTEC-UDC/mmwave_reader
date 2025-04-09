from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radar_id', default_value='isk_0'),
        DeclareLaunchArgument('radar_pos_x', default_value='0'),
        DeclareLaunchArgument('radar_pos_y', default_value='0'),
        DeclareLaunchArgument('radar_pos_z', default_value='1.741'),
        DeclareLaunchArgument('radar_yaw', default_value='0'),
        DeclareLaunchArgument('radar_pitch', default_value='0'),
        DeclareLaunchArgument('radar_roll', default_value='-0.22'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=['odom_to_radar_', LaunchConfiguration('radar_id')],
            arguments=[
                LaunchConfiguration('radar_pos_x'),
                LaunchConfiguration('radar_pos_y'),
                LaunchConfiguration('radar_pos_z'),
                LaunchConfiguration('radar_yaw'),
                LaunchConfiguration('radar_pitch'),
                LaunchConfiguration('radar_roll'),
                'odom',
                LaunchConfiguration('radar_id'),
                '30'
            ]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=['radar_to_rviz_', LaunchConfiguration('radar_id')],
            arguments=[
                '0', '0', '0', '1.57', '0', '0',
                LaunchConfiguration('radar_id'),
                ['rviz_', LaunchConfiguration('radar_id')],
                '30'
            ]
        ),

        Node(
            package='pose_publisher',
            executable='pose_publisher',
            name=['radar_pose_', LaunchConfiguration('radar_id')],
            parameters=[
                {'map_frame': 'odom'},
                {'base_frame': ['rviz_', LaunchConfiguration('radar_id')]}
            ],
            remappings=[('/pose', ['/pose_', LaunchConfiguration('radar_id')])]
        )
    ]) 