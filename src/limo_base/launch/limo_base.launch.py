import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory
    model_path = os.path.join(get_package_share_directory('limo_description'),
                            'urdf', 'limo_four_diff.xacro')

    declare_port_name_arg = DeclareLaunchArgument('port_name', default_value='ttyTHS1',
                                          description='usb bus name, e.g. ttyTHS1')
    declare_odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value='odom',
                                           description='Odometry frame id')
    declare_base_link_frame_arg = DeclareLaunchArgument('base_frame', default_value='base_link',
                                                description='Base link frame id')
    declare_odom_topic_arg = DeclareLaunchArgument('odom_topic_name', default_value='odom',
                                           description='Odometry topic name')
    declare_sim_control_rate_arg = DeclareLaunchArgument('control_rate', default_value='50',
                                                 description='Simulation control loop update rate')
    declare_use_mcnamu_arg = DeclareLaunchArgument('use_mcnamu', default_value='false',
                                           description='Use mecanum motion mode')
    declare_pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true',
                                           description='Parameter to publish odom')

    start_limo_base = Node(
        package='limo_base',
        executable='limo_base',
        output='screen',
        emulate_tty=True,
        parameters=[{
                'port_name': LaunchConfiguration('port_name'),
                'odom_frame': LaunchConfiguration('odom_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_topic_name': LaunchConfiguration('odom_topic_name'),
                'control_rate': LaunchConfiguration('control_rate'),
                'use_mcnamu': LaunchConfiguration('use_mcnamu'),
                'pub_odom_tf': LaunchConfiguration("pub_odom_tf")
        }])

    start_limo_joint_state_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher'
    )
    start_limo_robot_state_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', model_path])}]
    )

    ld = LaunchDescription()

    ld.add_action(declare_port_name_arg)
    ld.add_action(declare_odom_frame_arg)
    ld.add_action(declare_base_link_frame_arg)
    ld.add_action(declare_odom_topic_arg)
    ld.add_action(declare_sim_control_rate_arg)
    ld.add_action(declare_use_mcnamu_arg)
    ld.add_action(declare_pub_odom_tf_arg)
    ld.add_action(start_limo_base)
    ld.add_action(start_limo_joint_state_node)
    ld.add_action(start_limo_robot_state_node)

    return ld
