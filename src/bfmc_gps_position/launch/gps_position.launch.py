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

            # Stddev in metres at quality=100 (max error = 15 cm radius).
            # Scales linearly: stddev = base_stddev * (100 / quality).
            'base_stddev_m': 0.15,
            'max_stddev_m': 2.0,
            'min_quality': 5,

            # GPS messages arrive ~1 s after the measurement was taken.
            # Subtracted from the header stamp for correct EKF time association.
            'measurement_delay_s': 1.0,
        }],
    )

    return LaunchDescription([
        gps_tag_to_base_node,
    ])
