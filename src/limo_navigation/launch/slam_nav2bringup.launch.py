import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare use_sim_time argument
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include SLAM Toolbox online async launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Include Nav2 bringup launch
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch', 'navigation_launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Assemble launch description
    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_bringup_launch)
    return ld
