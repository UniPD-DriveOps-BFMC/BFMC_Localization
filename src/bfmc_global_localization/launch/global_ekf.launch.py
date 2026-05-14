from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution([
        FindPackageShare('bfmc_global_localization'),
        'config',
        'global_ekf.yaml',
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

    return LaunchDescription([
        global_ekf_node,
    ])
