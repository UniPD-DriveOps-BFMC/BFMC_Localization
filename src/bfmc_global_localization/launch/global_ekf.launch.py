from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def launch_setup(context, *args, **kwargs):
    use_gps = LaunchConfiguration('use_gps').perform(context).lower() == 'true'

    config_file = 'global_ekf.yaml' if use_gps else 'global_ekf_no_gps.yaml'

    ekf_config = PathJoinSubstitution([
        FindPackageShare('bfmc_global_localization'),
        'config',
        config_file,
    ])

    sign_map_file = PathJoinSubstitution([
        FindPackageShare('bfmc_global_localization'),
        'maps',
        'sign_with_position.txt',
    ])

    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_config],
        remappings=[
            ('odometry/filtered', '/odometry/global'),
        ],
    )

    sign_localization_node = Node(
        package='bfmc_global_localization',
        executable='sign_localization_node',
        name='sign_localization_node',
        output='screen',
        parameters=[{
            'sign_map_file': sign_map_file,
            'detection_topic': '/traffic/detection',
            'local_odom_topic': '/odometry/global',
            'output_pose_topic': '/automobile/sign/base_pose',
            'map_frame': 'map',
            # Stddev of distance sensor reading (metres).
            'distance_stddev_m': 0.30,
            # Stddev of bearing to sign (radians).
            # Lateral position error = distance * bearing_stddev.
            'bearing_stddev_rad': 0.10,
            # Only associate a detected sign if the nearest map candidate
            # is within this radius of the current odometry estimate.
            'max_association_m': 5.0,
        }],
    )

    graph_file = PathJoinSubstitution([
        FindPackageShare('bfmc_global_localization'),
        'maps',
        'Competition_track_graph.graphml',
    ])

    position_publisher_node = Node(
        package='bfmc_global_localization',
        executable='position_publisher_node',
        name='position_publisher_node',
        output='screen',
        parameters=[{
            'use_gps': use_gps,
            'odom_topic': '/odometry/global',
            'local_odom_topic': '/odometry/local',
            'coordinate_topic': '/automobile/current_coordinate',
            'node_topic': '/automobile/current_node',
            'distance_topic': '/automobile/total_distance',
            'speed_topic': '/automobile/current_speed',
            'graph_file': graph_file,
        }],
    )

    return [global_ekf_node, sign_localization_node, position_publisher_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gps',
            default_value='true',
            description='Enable GPS fusion. Set false when GPS hardware is unavailable.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
