# Copyright 2022 Open Source Robotics Foundation, Inc.
# Modified by Lehe Hu, 2025
# Changes: Refactored class structure.
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

def launch_setup(context, *args, **kwargs):
    # launch_setup() 函数：定义实际的启动逻辑

    # 获取依赖包的 install 路径，用于定位模型、世界文件和配置文件。
    pkg_ros_gz_bringup = get_package_share_directory('ros_gz_bringup')
    pkg_gz_worlds = get_package_share_directory('gz_worlds')
    pkg_gz_models = get_package_share_directory('gz_models')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 动态获取启动参数 robot_model_name，用于后续命名机器人实体与设置命名空间。
    robot_model_name = LaunchConfiguration('robot_model_name').perform(context)

    # Load the SDF file from "description" package
    # robot_xacro_file  =  os.path.join(pkg_gz_models, 'models', 'sdf', 'diff_drive', 'model.sdf')
    # with open(robot_xacro_file, 'r') as infp:
    #     robot_desc = infp.read()
    
    # 解析机器人模型（xacro 文件），注入变量后转换成 XML 字符串供 robot_state_publisher 使用。
    robot_xacro_file  =  os.path.join(pkg_gz_models, 'models', 'urdf', 'diff_drive.xacro')
    robot_desc = xacro.process_file(
        robot_xacro_file,
        mappings={
            'robot_model_name': robot_model_name  # 设置外部参数
        }
    ).toxml()
    
    # wall_sdf_file  =  os.path.join(pkg_gz_models, 'models', 'sdf', 'wall', 'model.sdf')
    # wall_sdf_file  =  os.path.join(pkg_gz_models, 'models', 'urdf', 'output', 'wall.sdf')
    # with open(wall_sdf_file, 'r') as infp:
    #     wall_desc = infp.read()

    # 解析 wall 模型，注入变量后转换成 XML 字符串供 wall_state_publisher 使用。
    wall_xacro_file  =  os.path.join(pkg_gz_models, 'models', 'urdf', 'wall.xacro')
    wall_desc = xacro.process_file(
        wall_xacro_file,
        # mappings={
        #     'robot_model_name': robot_model_name  # 设置外部参数
        # }
    ).toxml()

    # 加载仿真世界文件路径
    world_file = os.path.join(pkg_gz_worlds,'worlds','sensor_world.sdf')

    # 创建 robot_state_publisher ros 节点
    # robot_state_publisher 读取解析后的 urdf 模型，订阅 joint_states topic
    # 把机器人 joint 状态发布到 tf 和 tf_static 上
    # 把解析后的 urdf 模型发布到 robot_description topic上，供 ros_gz_sim 读取
    # 参数 robot_description 指向解析后的 urdf 模型
    # 参数 frame_prefix 将 TF 框架命名空间前缀化，支持多机器人
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',  # 节点名
        output='both',
        parameters=[
            {'robot_description': robot_desc},
            {'frame_prefix': robot_model_name+'/'},  
        ],
        remappings=[
            ('robot_description', robot_model_name + '_description'),  # 映射发布话题 robot_description -> robot_model_name + '_description'
            ('joint_states', robot_model_name + '/joint_states')   # 映射订阅话题 joint_states -> robot_model_name + '/joint_states'
        ]
    )

    # 为静态物体 wall 创建 robot_state_publisher 节点
    wall_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='wall_state_publisher',  # 节点名
        output='screen',
        parameters=[
            {'robot_description': wall_desc},
        ],
        remappings=[('robot_description', 'wall_description')]  # 映射发布话题 robot_description -> wall_description
    )

    # 启动 ros_gz_bridge
    # 使用yaml配置文件映射ros话题和gazebo话题的关系    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_ros_gz_bringup, 'config', 'ros_gz_bridge_config.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    
    # 启动 gz_sim.launch.py
    # 启动参数 gz_args 指向 world sdf 文件
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': world_file}.items(),
    )
    
    # 启动 ros_gz_sim create 节点
    # 动态在 gazebo 世界中添加机器人
    # 机器人由 robot_state_publisher 发布在 '/'+robot_model_name+'_description' 话题上
    diff_drive = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/'+robot_model_name+'_description', 
                   '-name', robot_model_name,  # 定义机器人名字
                   '-z', '1'],  # 定义初始位置，机器人坐标原点高出地面1米
        output='screen'
    )

    # 动态在 gazebo 世界中添加 wall
    wall = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/wall_description', 
                   '-name', 'wall',
                   '-x', '5',
                   '-z', '1'
                   ],
        output='screen'
    )

    # 启动 rviz
    # 使用 rviz 配置文件
    # 设置启动条件：启动参数 rviz=true；否则，不启动rviz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_ros_gz_bringup, 'config', 'diff_drive.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # 启动 ros_application obstacle_advoider 节点
    obstacle_avoider = Node(
        package='ros_application',
        executable='obstacle_avoider',
        arguments=['-robot_name', robot_model_name, 
                   '-lidar_offset', '1.0',
                   '-x_speed', '0.5',
                   ],
        output='screen'
    )

    # 返回所有节点组合
    # 需要考虑节点启动顺序。例如：必须启动 gz_sim 后，才能正常启动 diff_drive 和 wall
    return [
            robot_state_publisher,  
            wall_state_publisher, 
            gz_sim, 
            diff_drive, 
            wall,
            bridge, 
            rviz,
            obstacle_avoider
        ]

def generate_launch_description():
    # 定义 launch 文件入口和启动参数
    # 定义启动参数 rviz 和 robot_model_name
    # 通过 OpaqueFunction 动态执行 launch_setup() 以支持 Python 脚本逻辑

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz.'),
        DeclareLaunchArgument('robot_model_name', default_value='diff_drive_robot', description='Robot model name in world.'),
        OpaqueFunction(function=launch_setup)
    ])
