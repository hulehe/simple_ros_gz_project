cmake_minimum_required(VERSION 3.5)

project(ros_application)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(obstacle_avoider nodes/obstacle_avoider.cc)
ament_target_dependencies(obstacle_avoider rclcpp sensor_msgs geometry_msgs)
install(TARGETS obstacle_avoider DESTINATION lib/${PROJECT_NAME})

ament_package()
