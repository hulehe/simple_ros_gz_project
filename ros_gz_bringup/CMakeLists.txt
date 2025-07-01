# 设置 CMake 最低版本要求
cmake_minimum_required(VERSION 3.5)

# 定义项目名称，必须与 package.xml 中的 <name> 一致
project(ros_gz_bringup)

# 查找并引入 ROS 2 的构建系统支持（ament_cmake 是 ROS 2 的标准构建工具）
find_package(ament_cmake REQUIRED)

# 安装ros2 launch文件到安装目录的 share/<项目名>/ 下
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 安装ros2 launch文件到安装目录的 share/<项目名>/ 下
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
