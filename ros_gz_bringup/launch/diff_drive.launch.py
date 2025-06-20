# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro
from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_ros_gz_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_gz_worlds = get_package_share_directory('gz_worlds')
    pkg_gz_models = get_package_share_directory('gz_models')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    robot_xacro_file  =  os.path.join(pkg_gz_models, 'models', 'urdf', 'diff_drive.xacro')
    robot_desc = xacro.process_file(
        robot_xacro_file,
        mappings={
            'lidar_topic': '/model/diff_drive/scan',  # 设置外部参数
        }
    ).toxml()
    # with open(robot_urdf_file, 'r') as infp:
    #     robot_desc = infp.read()
    wall_sdf_file  =  os.path.join(pkg_gz_models, 'models', 'sdf', 'wall', 'model.sdf')
    with open(wall_sdf_file, 'r') as infp:
        wall_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_gz_worlds,
            'worlds',
            'diff_drive.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {'wall_description': wall_desc},
        ]
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ros_gz_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ros_gz_bringup, 'config', 'ros_gz_bridge_config.yaml'),
            'lidar_topic': LaunchConfiguration('lidar_topic', default='scan'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    # 加载机器人到世界
    robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'diff_drive'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        DeclareLaunchArgument('lidar_topic', default_value='/diff_drive_robot/scan',
                              description='Lidar topic.'), 
        robot_node,
        bridge,
        robot_state_publisher,
        rviz
    ])
