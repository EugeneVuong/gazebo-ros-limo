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

    # # Declare SLAM params_file argument
    # declare_slam_params_file = DeclareLaunchArgument(
    #     'slam_params_file',  # Renamed from 'params_file'
    #     default_value=os.path.join(
    #         get_package_share_directory('limo_navigation'),
    #         'params', 'mapper_params_online.yaml'),
    #     description='Path to SLAM Toolbox online parameters')
    # slam_params_file = LaunchConfiguration('slam_params_file')  # Renamed from 'params_file'

    # Declare Nav2 params file argument
    # declare_nav2_params_file = DeclareLaunchArgument(
    #     'nav2_params_file',
    #     default_value=os.path.join(
    #         get_package_share_directory('limo_navigation'),
    #         'params', 'nav2_params.yaml'),
    #     description='Path to Nav2 parameters')
    # nav2_params_file = LaunchConfiguration('nav2_params_file')

    # Include SLAM Toolbox online async launch
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        # Use the correct argument name 'slam_params_file'
        # launch_arguments={'use_sim_time': use_sim_time, 'slam_params_file': slam_params_file}.items()
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
        # Use the correct argument name 'params_file' for Nav2
        # launch_arguments={'use_sim_time': use_sim_time, 'params_file': nav2_params_file}.items()
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Assemble launch description
    ld = LaunchDescription()
    ld.add_action(declare_sim_time)
    # ld.add_action(declare_slam_params_file)  # Use the renamed declaration
    # ld.add_action(declare_nav2_params_file)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(nav2_bringup_launch)
    return ld
