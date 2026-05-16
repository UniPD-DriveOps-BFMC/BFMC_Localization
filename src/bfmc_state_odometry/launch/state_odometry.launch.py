from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    encoder_odometry_node = Node(
        package='bfmc_state_odometry',
        executable='encoder_odometry_node',
        name='encoder_odometry_node',
        output='screen',
        parameters=[{
            'speed_topic': '/automobile/encoder/speed',
            'distance_topic': '/automobile/encoder/distance',
            'odom_topic': '/encoder_odom',

            'odom_frame': 'odom',
            'base_frame': 'base_link',

            # false = integrate speed topic
            # true  = use distance increments
            'use_distance': False,

            'publish_rate_hz': 50.0,
            'velocity_variance': 0.02,
            'large_variance': 999999.0,
        }],
    )

    car_imu_republisher_node = Node(
        package='bfmc_state_odometry',
        executable='car_imu_republisher_node',
        name='car_imu_republisher_node',
        output='screen',
        parameters=[{
            'input_topic': '/automobile/imu/data',
            'output_topic': '/car/imu/data',
            'output_frame_id': 'imu_link',
        }],
    )

    return LaunchDescription([
        encoder_odometry_node,
        car_imu_republisher_node,
    ])
