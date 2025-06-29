# simple_ros_gz_project
**作者：hulehe**  
**创建日期：2025/6/26**  
**最近更新日期：2025/6/29**  
这是一个在gazebo仿真环境下使用ros控制小车移动的演示项目。项目结构源于github上的gazebosim/ros_gz_project_template的修改。

## 项目包含的包 Included packages

* `gz_models` - 存放model的xacro、urdf和sdf文件

* `gz_worlds` - 存放gazebo仿真世界的sdf文件和插件代码

* `ros_application` - 存放ros2代码和配置文件

* `ros_gz_bringup` - 存放启动文件和高层应用（如rviz和ros_gz_bridge）的配置文件


## 安装 Install

### 配置运行环境和依赖 Requirements

1. 不限但推荐使用Ubuntu24.04
    - 注意：如果你使用的是wsl ubuntu desktop，并且你的电脑配有nvidia显卡，请.bashrc中配置`export LIBGL_ALWAYS_SOFTWARE=1`以保证gazebo的正常运行。因为wsl环境无法直接调用电脑的显卡。LIBGL_ALWAYS_SOFTWARE=1显性让OpenGL使用CPU而不是GPU进行gazebo渲染。

2. 不限但推荐使用ROS jazzy desktop版本。安装ros开发依赖。

3. 进入gazebo官网，按照说明安装和上步已经安装的ros版本匹配的gazebo
    - gazebo官网: https://gazebosim.org/docs/latest/ros_installation/
    - gazebo安装命令
        ```bash
        sudo apt-get install ros-${ROS_DISTRO}-ros-gz
        ```
    - 安装后可以使用gz sim进行gazebo测试。
        - 注意：gazebo库默认被安装到/opt/ros/jazzy/opt目录下。需要重新执行`source /opt/ros/jazzy/setup.bash`才能让命令行找到`gz sim`命令。


## 使用 Use

1. 创建工作空间，克隆代码，把simple_ros_gz_project改成src

   ```bash
   mkdir -p ~/template_ws
   cd ~/template_ws
   git clone https://github.com/hulehe/simple_ros_gz_project.git
   mv simple_ros_gz_project src
   ```

2. 安装ros依赖 Install dependencies

    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro $ROS_DISTRO
    ```

3. 编译项目 Build the project

    ```bash
    colcon build
    ```

4. 配置项目运行环境 Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

5. 启动程序 Launch the simulation

    ```bash
    ros2 launch ros_gz_bringup sensor_world.launch.py
    ```

6. 发布/cmd_vel启动小车 Publish /cmd_vel to drive vehicle to move
    ```bash
    ros2 topic pub --once /diff_drive_robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}"
    ```

7. 启动ros节点，控制小车躲避障碍物
    ```bash
    ros2 run ros_application obstacle_avoider --ros-args \
        -p robot_name:=diff_drive_robot \
        -p lidar_offset:=1.0 \
        -p x_speed:=0.5
    ```

---
**作者主页：** www.lehe.life