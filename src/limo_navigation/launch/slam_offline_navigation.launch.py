import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Directories
    bringup_dir = get_package_share_directory('limo_navigation')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation clock')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true', description='Autostart Nav2 stack')

    # Include SLAM toolbox offline launch
    slam_offline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'slam_offline_map_launch.py')
        )
    )

    # Include Limo Navigation launch
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'limo_navigation.launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart
        }.items()
    )

    # Build LaunchDescription
    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(slam_offline)
    ld.add_action(navigation)

    return ld
