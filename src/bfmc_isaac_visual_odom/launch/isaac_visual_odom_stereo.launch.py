from launch import LaunchDescription

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    left_image_topic = '/oak/left/image_mono'
    right_image_topic = '/oak/right/image_mono'
    left_info_topic = '/oak/left/camera_info'
    right_info_topic = '/oak/right/camera_info'
    imu_topic = '/oak/imu/data'

    base_frame = 'base_link'
    odom_frame = 'odom'
    map_frame = 'visual_map'
    imu_frame = 'oak_imu_frame'

    left_camera_frame =  'oak_left_camera_optical_frame'
    right_camera_frame = 'oak_right_camera_optical_frame'

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'num_cameras': 2,
            'min_num_images': 2,

            'rectified_images': True,

            'sync_matching_threshold_ms': 20.0,
            'image_jitter_threshold_ms': 34.0,

            'enable_imu_fusion': True,
            'imu_frame': imu_frame,

            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'imu_jitter_threshold_ms': 10.0,
            'imu_buffer_size': 200,

            'enable_localization_n_mapping': False,

            'enable_ground_constraint_in_odometry': True,
            'enable_ground_constraint_in_slam': False,

            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'map_frame': map_frame,

            'camera_optical_frames': [
                left_camera_frame,
                right_camera_frame,
            ],

            'publish_odom_to_base_tf': False,
            'publish_map_to_odom_tf': False,

            'enable_slam_visualization': True,
            'enable_observations_view': True,
            'enable_landmarks_view': False,
            'verbosity': 1,
        }],
        remappings=[
            ('visual_slam/image_0', left_image_topic),
            ('visual_slam/image_1', right_image_topic),
            ('visual_slam/camera_info_0', left_info_topic),
            ('visual_slam/camera_info_1', right_info_topic),
            ('visual_slam/imu', imu_topic),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    visual_odom_republisher = Node(
        package='bfmc_isaac_visual_odom',
        executable='visual_odom_republisher',
        name='visual_odom_republisher',
        output='screen',
        parameters=[{
            'input_topic': '/visual_slam/tracking/odometry',
            'output_topic': '/visual_odom',
            'planar_output_topic': '/visual_odom_planar',

            'publish_planar': True,
            'flatten_planar': True,

            'output_frame_id': odom_frame,
            'output_child_frame_id': base_frame,

            'position_variance_xy': 0.04,
            'position_variance_z': 999.0,
            'roll_pitch_variance': 999.0,
            'yaw_variance': 0.08,
            'linear_velocity_variance_xy': 0.10,
            'angular_velocity_variance_yaw': 0.10,
        }],
    )

    return LaunchDescription([
        visual_slam_container,
        visual_odom_republisher,
    ])