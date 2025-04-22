from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    rviz_config_dir = os.path.join(get_package_share_directory('limo_navigation'),
                                   'rviz', 'slam_toolbox_default.rviz')
    return LaunchDescription([
        launch_ros.actions.Node(
          parameters=[
            get_package_share_directory('limo_navigation') + '/params/mapper_params_offline.yaml'
          ],
          package='slam_toolbox',
          executable='sync_slam_toolbox_node',
          name='slam_toolbox',
          output='screen'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            # parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])