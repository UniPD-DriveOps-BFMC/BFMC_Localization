from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution([
        FindPackageShare('bfmc_odometry_fusion'),
        'config',
        'local_ekf.yaml',
    ])

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    odom_stats_node = Node(
        package='bfmc_odometry_fusion',
        executable='odom_stats_node',
        name='odom_stats_node',
        output='screen',
        parameters=[{
            'local_odom_topic': '/odometry/local',
            'distance_topic': '/odom_distance',
            'velocity_topic': '/odom_velocity',
        }],
    )

    return LaunchDescription([
        ekf_node,
        odom_stats_node,
    ])
