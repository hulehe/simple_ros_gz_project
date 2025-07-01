# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.5)

# 定义项目名称，必须与 package.xml 中的 <name> 一致
project(ros_application)

# 查找并引入 ROS 2 的构建系统支持（ament_cmake 是 ROS 2 的标准构建工具）
find_package(ament_cmake REQUIRED)

# 查找 ROS 2 所需依赖包，确保它们在构建和运行时可用
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 构建一个可执行文件 obstacle_avoider，源码文件为 nodes/obstacle_avoider.cc
add_executable(obstacle_avoider nodes/obstacle_avoider.cc)

# 为 obstacle_avoider 可执行目标添加依赖库（来自 find_package）
ament_target_dependencies(obstacle_avoider rclcpp sensor_msgs geometry_msgs)

# 安装可执行文件到安装目录的 lib/<项目名>/ 下
install(TARGETS obstacle_avoider DESTINATION lib/${PROJECT_NAME})

# 通知 ROS 2 构建系统这是一个有效的 ROS 2 包，并导出依赖、生成环境配置等
ament_package()
