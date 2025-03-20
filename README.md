# LIMO ROS2

This repository contains code for **ROS2 development with the LIMO robot**. Please note that:

* If you're new to the LIMO robot platform, it's highly recommended that you read [LIMO Usage and Development Manual](https://github.com/agilexrobotics/limo-doc/blob/master/Limo%20user%20manual(EN).md) and get yourself familiar with basic operations of the robot first. 
* If you're not familiar with ROS2, you can refer to the [ROS2 Documentation](https://docs.ros.org/en/humble/index.html) to learn more about ROS2 concepts. 

Since the onboard computer equipped on the LIMO robot is Nvidia Jetson Nano which only works with Jetpack v4.x (Ubuntu 18.04), there is no easy way to do native development with an active ROS2 LTS release on the robot ([REP2000](https://www.ros.org/reps/rep-2000.html)). Thus the packages within this repository are developed and only tested in a ROS Humble Docker container (docker image: westonrobot/limo_ros:humble).

## Changelog

### 2023-08-22

- Moved obstacle layer to local costmap
- Cleaned up old ros1 costmap params
- **Set environment flag to use cyclone dds to fix costmap obstacle not working**

## Setup development environment

You can set up the development environment either on the Jetson Nano or your desktop/laptop PC. The steps for the setup are the same:

* **Clone the repo to the computer**

```bash
$ git clone https://github.com/westonrobot/limo_ros2.git
$ cd limo_ros2
```

* **Install VS Code with [Remote Development Plugins](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack)**

* **Start development inside the container**
    If you're developing on the LIMO directly:
    1. Launch VS Code on LIMO
    2. In the workspace root folder, "Crtl + Shift + P" to open up the command palette, and select "Remote-Containers: Rebuild and Reopen in Container"

    If you're developing remotely from your PC:
    1. Launch VS Code on your PC
    2. In the workspace root folder, "Crtl + Shift + P" to open up the command palette, and select "Remote-SSH: Connect to Host", finish the connection configuration
    3. Type "Crtl + Shift + P" to open up the command palette again, and select "Remote-Containers: Rebuild and Reopen in Container"

Latest image: westonrobot/limo_ros:humble_22082023

You can find more information about development inside containers [here](./docs/README.md)

## Use limo_ros2 packages

Once you're inside the container, the commands to build or launch ROS packages are the same as those in a native environment:

* Build the colcon workspace
  
    ```bash
    # in workspace root
    ./build.sh

    # Remember to always source ros workspace on new terminal
    . install/setup.bash
    ```

* Bringup robot

    ```bash
    # Bringup robot
    ros2 launch limo_bringup limo_start.launch.py

    # OR

    # Bringup robot in simulator
    # Add use_sim_time:=true argument for subsequent launches if using sim
    ros2 launch limo_gazebosim limo_gazebo_diff.launch.py
    ```

* Bringup camera
  
    ```bash
    ros2 launch astra_camera dabai.launch.py
    ```

* Teleoperation

    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

* SLAM

    ```bash
    # Launch SLAM node
    ros2 launch limo_bringup cartographer.launch.py

    # To save map
    ros2 run nav2_map_server map_saver_cli -f [filename]
    ```

* Navigation

    ```bash
    # In seperate terminals
    ros2 launch limo_navigation limo_localization.launch.py

    ros2 launch limo_navigation limo_controller.launch.py

    # Single launch
    ros2 launch limo_navigation limo_navigation.launch.py
    ```

## Running your own navigation stack or simulation on local PC

**Note: Limo container is running on cyclone DDS, you need to set the following environment variable**

```bash
# Install package
sudo apt install ros-humble-rmw-cyclonedds-cpp
# Export environment variable
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Sample gazebo and navigation are provided for differential motion model in the repo, to use them

    * Clone the repo into your local pc
    * Feel free to use provided Dockerfile as a reference
    * Rebuild new container for local PC