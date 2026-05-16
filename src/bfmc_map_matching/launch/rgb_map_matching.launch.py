from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('bfmc_map_matching'),
        'config',
        'map_matching.yaml',
    ])

    maps_dir = PathJoinSubstitution([
        FindPackageShare('bfmc_map_matching'),
        'maps',
    ])

    rgb_map_matching_node = Node(
        package='bfmc_map_matching',
        executable='rgb_map_matching_node',
        name='rgb_map_matching_node',
        output='screen',
        parameters=[
            config_file,
            {'maps_dir': maps_dir},
        ],
    )

    return LaunchDescription([
        rgb_map_matching_node,
    ])
