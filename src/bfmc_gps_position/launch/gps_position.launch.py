from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gps_tag_to_base_node = Node(
        package='bfmc_gps_position',
        executable='gps_tag_to_base_node',
        name='gps_tag_to_base_node',
        output='screen',
        parameters=[{
            'tag_pose_topic': '/automobile/gps/tag_pose',
            'local_odom_topic': '/odometry/local',
            'base_pose_topic': '/automobile/gps/base_pose',

            'map_frame': 'map',
            'base_frame': 'base_link',
            'tag_frame': 'gps_tag_link',

            # Use local EKF yaw to rotate the URDF tag offset.
            'use_yaw_from_local_odom': True,

            # Force competition GPS/UWB covariance.
            'force_xy_variance': True,
            'position_stddev_m': 0.15,
        }],
    )

    return LaunchDescription([
        gps_tag_to_base_node,
    ])
