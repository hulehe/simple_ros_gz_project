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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro
from launch_ros.actions import Node
from launch import LaunchContext

def launch_setup(context, *args, **kwargs):
    # Setup project paths
    pkg_ros_gz_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_gz_worlds = get_package_share_directory('gz_worlds')
    pkg_gz_models = get_package_share_directory('gz_models')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    robot_model_name = LaunchConfiguration('robot_model_name').perform(context)

    # Load the SDF file from "description" package
    # robot_xacro_file  =  os.path.join(pkg_gz_models, 'models', 'sdf', 'diff_drive', 'model.sdf')
    # with open(robot_xacro_file, 'r') as infp:
    #     robot_desc = infp.read()
    
    robot_xacro_file  =  os.path.join(pkg_gz_models, 'models', 'urdf', 'diff_drive.xacro')
    robot_desc = xacro.process_file(
        robot_xacro_file,
        mappings={
            'robot_model_name': robot_model_name  # 设置外部参数
        }
    ).toxml()
    

    wall_sdf_file  =  os.path.join(pkg_gz_models, 'models', 'sdf', 'wall', 'model.sdf')
    with open(wall_sdf_file, 'r') as infp:
        wall_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    world_file = os.path.join(pkg_gz_worlds,'worlds','sensor_world.sdf')

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
            {'frame_prefix': robot_model_name+'/'},  
        ],
        remappings=[
            ('robot_description', robot_model_name + '_description'),
            ('joint_states', robot_model_name + '/joint_states')
        ]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ros_gz_bringup, 'config', 'ros_gz_bridge_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file}.items(),
    )
    
    # 加载机器人到世界
    diff_drive = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/'+robot_model_name+'_description', 
                   '-name', robot_model_name, 
                   '-z', '1'],
        output='screen'
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ros_gz_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return [
        robot_state_publisher,         
        gz_sim, 
        diff_drive, 
        bridge, 
        rviz
        ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('robot_model_name', default_value='diff_drive_robot', description='Robot model name in world.'),
        OpaqueFunction(function=launch_setup)
    ])
