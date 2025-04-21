import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # --- Constants ---
    sdf_model_name = 'limo_four_diff_low.sdf' # Using SDF directly
    world_file_name = 'test_zone.world'   # Your world file
    bridge_config_file_name = 'limo_bridge.yaml' # Bridge config is essential

    robot_name_in_model = 'limo_low'

    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.1'

    # --- Package Paths ---
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_limo_gazebosim = get_package_share_directory('limo_gazebosim') # Assumed package for world/config

    # --- File Paths ---
    default_sdf_model_path = os.path.join(
        pkg_limo_gazebosim,
        'models', # Assuming SDF is in description/urdf
        'limo_low',
        sdf_model_name
    )
    world_path = os.path.join(
        pkg_limo_gazebosim,
        'worlds',
        world_file_name
    )
    default_bridge_config_path = os.path.join(
        pkg_limo_gazebosim,
        'config', # Store bridge config in a 'config' directory
        bridge_config_file_name
    )
    print("test", default_bridge_config_path)

    # --- Launch Configurations ---
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gz_gui = LaunchConfiguration('gui', default='true') # Gazebo GUI client ('gui' is a common name)
    gz_headless = LaunchConfiguration('headless', default='false') # Server only
    namespace = LaunchConfiguration('namespace', default='')
    sdf_model = LaunchConfiguration('sdf_model', default=default_sdf_model_path)
    bridge_config_file = LaunchConfiguration('bridge_config_file', default=default_bridge_config_path)
    use_simulator = LaunchConfiguration('use_simulator', default='true')
    world = LaunchConfiguration('world', default=world_path)

    # --- Declare Launch Arguments ---
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
    declare_gz_gui_cmd = DeclareLaunchArgument( # Changed name for clarity
        name='gui', default_value='true',
        description='Whether to launch the Gazebo GUI client')
    declare_gz_headless_cmd = DeclareLaunchArgument( # Changed name for clarity
        name='headless', default_value='false',
        description='Whether to execute Gazebo server only (no GUI)')
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace', default_value='',
        description='Top-level namespace for ROS nodes (e.g., bridge)')
    declare_sdf_model_path_cmd = DeclareLaunchArgument(
        name='sdf_model', default_value=default_sdf_model_path,
        description='Absolute path to robot SDF file (for Gazebo spawning)')
    declare_bridge_config_file_cmd = DeclareLaunchArgument(
        name='bridge_config_file', default_value=default_bridge_config_path,
        description='Full path to the ROS-Gazebo Bridge config file')
    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator', default_value='true',
        description='Whether to start the Gazebo simulator')
    declare_world_cmd = DeclareLaunchArgument(
        name='world', default_value=world_path,
        description='Full path to the world model file to load')

    # --- Environment Variables ---
    # MUST include paths for Gazebo to find models, meshes, etc.
    set_env_vars_resources = AppendEnvironmentVariable(
      'GZ_SIM_RESOURCE_PATH',
      [
          os.path.join(pkg_limo_gazebosim, 'worlds'),
          os.path.join(pkg_limo_gazebosim, 'models'), # CORRECT: Path containing 'limo' and 'test_zone' folders
          os.path.join(pkg_limo_gazebosim, 'config')
      ]
    )

    # --- Gazebo Simulation ---
    # Start Server
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    # # --- Robot Spawning ---
    # Spawn robot in Gazebo using the SDF file
    spawn_entity_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-file', sdf_model, # Load from SDF file directly
                   '-name', robot_name_in_model,
                   '-allow_renaming', 'true',
                   '-x', spawn_x_val,
                   '-y', spawn_y_val,
                   '-z', spawn_z_val,
                  ],
        condition=IfCondition(use_simulator)
    )

    # # --- ROS <-> Gazebo Communication ---
    # # Start ROS-Gazebo Bridge (Parameter Bridge)
    start_ros_gz_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        namespace=namespace,
        arguments=['--ros-args', '-p', f'config_file:={bridge_config_file}'],
        condition=IfCondition(use_simulator),
        output='screen'
    )
    # Start ROS-Gazebo Image Bridge (Uncomment if parameter bridge doesn't handle images)
    # start_ros_gz_image_bridge_cmd = Node(
    #      package='ros_gz_image',
    #      executable='image_bridge',
    #      namespace=namespace,
    #      # List specific camera topics to bridge if needed
    #      arguments=['/limo/depth/image_raw'],
    #      condition=IfCondition(use_simulator),
    #      output='screen'
    # )

    # --- Launch Description Assembly ---
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_gz_gui_cmd)
    ld.add_action(declare_gz_headless_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_sdf_model_path_cmd)
    ld.add_action(declare_bridge_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add actions
    ld.add_action(set_env_vars_resources) # Set environment variables first
    ld.add_action(gzserver_cmd)           # Start Gazebo Server
    ld.add_action(gzclient_cmd)           # Start Gazebo Client (GUI)
    ld.add_action(spawn_entity_cmd)       # Spawn Robot from SDF
    ld.add_action(start_ros_gz_bridge_cmd) # Start Bridge
    # ld.add_action(start_ros_gz_image_bridge_cmd) # Add if needed

    return ld