"""
BFMC Localization — Master Launch File
=======================================
Starts all localization packages in dependency order.

IMPORTANT: isaac_ros_visual_slam (used by bfmc_isaac_visual_odom) requires
the Isaac ROS Docker container. Run this launch file from inside:

    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
    ./scripts/run_dev.sh -d ~/workspaces/isaac_ros-dev --docker_arg "--privileged"

Then inside the container:
    source /opt/ros/humble/setup.bash
    source ~/workspaces/isaac_ros-dev/install/setup.bash
    ros2 launch bfmc_global_localization bfmc_localization.launch.py

Arguments
---------
use_gps      : true | false   Enable GPS fusion (default: true)
camera_mode  : stereo | rgbd  Visual odometry camera mode (default: stereo)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_gps = LaunchConfiguration('use_gps').perform(context)
    camera_mode = LaunchConfiguration('camera_mode').perform(context).lower()

    # --- 1. State Odometry (encoder + IMU) ---
    state_odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bfmc_state_odometry'),
                'launch', 'state_odometry.launch.py',
            ])
        ])
    )

    # --- 2. Isaac Visual Odometry (stereo or RGBD) ---
    visual_odom_file = (
        'isaac_visual_odom_stereo.launch.py'
        if camera_mode == 'stereo'
        else 'isaac_visual_odom_rgbd.launch.py'
    )
    isaac_visual_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bfmc_isaac_visual_odom'),
                'launch', visual_odom_file,
            ])
        ])
    )

    # --- 3. Odometry Fusion (local EKF: encoder + IMU + visual odom) ---
    odometry_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bfmc_odometry_fusion'),
                'launch', 'local_ekf.launch.py',
            ])
        ])
    )

    # --- 4. GPS Position (tag pose → base pose with quality covariance) ---
    gps_position = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bfmc_gps_position'),
                'launch', 'gps_position.launch.py',
            ])
        ])
    )

    # --- 5. Map Matching (RGB image → map-matched pose) ---
    map_matching = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bfmc_map_matching'),
                'launch', 'rgb_map_matching.launch.py',
            ])
        ])
    )

    # --- 6. Global Localization (global EKF + sign localization + position publisher) ---
    global_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bfmc_global_localization'),
                'launch', 'global_ekf.launch.py',
            ])
        ]),
        launch_arguments={'use_gps': use_gps}.items(),
    )

    return [
        state_odometry,
        isaac_visual_odom,
        odometry_fusion,
        gps_position,
        map_matching,
        global_localization,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gps',
            default_value='true',
            description='Enable GPS fusion. Set false when GPS hardware is unavailable.',
        ),
        DeclareLaunchArgument(
            'camera_mode',
            default_value='stereo',
            description='Visual odometry camera mode: stereo or rgbd.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
