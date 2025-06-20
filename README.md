# ros_gz_project_template
A template project integrating ROS 2 and Gazebo simulator.

## Included packages

* `gz_models` - holds the sdf description of the simulated system and any other assets.

* `gz_worlds` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_application` - holds ros2 specific code and configurations.

* `ros_gz_bringup` - holds launch files and high level utilities.


## Install

For using the template with Gazebo Fortress switch to the `fortress` branch of this repository, otherwise use the default branch `main` for Gazebo Harmonic onwards.

### Requirements

1. Choose a ROS and Gazebo combination https://gazebosim.org/docs/latest/ros_installation

   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

    ```bash
    export GZ_VERSION=harmonic
    ```
    Also need to build [`ros_gz`](https://github.com/gazebosim/ros_gz) and [`sdformat_urdf`](https://github.com/ros/sdformat_urdf) from source if binaries are not available for your chosen combination.

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

### Use as template
Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

   ```bash
   mkdir -p ~/template_ws/src
   cd ~/template_ws/src
   git clone https://github.com/gazebosim/ros_gz_project_template.git
   ```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_example_bringup diff_drive.launch.py
    ```
1. Publish /cmd_vel to drive vehicle to move
    ```bash
    ros2 topic pub --once /diff_drive_robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
    ```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
