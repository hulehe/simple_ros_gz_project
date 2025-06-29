cmake_minimum_required(VERSION 3.5)

project(gz_models)

find_package(ament_cmake REQUIRED)
# ament_cmake包是ros团队开发并维护的，用于ros项目构建的包。
# ament_environment_hooks和ament_package都源于该包。

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  # 将当前源目录下models目录下的全部内容复制到安装目录下的share/${PROJECT_NAME}/models
  DIRECTORY
    models/
  DESTINATION share/${PROJECT_NAME}/models
)
# 测试install （注意：测试中使用WARNING而不是STATUS是因为colcon build默认不显示STATUS消息）
# 打印当前源目录
# message(WARNING "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
# 打印安装目录
# message(WARNING "CMAKE_INSTALL_PREFIX = ${CMAKE_INSTALL_PREFIX}")

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
# ament环境变量钩子
# 在命令行中输入source install/setup.bash后，会根据dsv.in文件中的规则配置环境变量
# 测试方法：使用colcon build构建后，在source install/setup.bash前和后，分别使用echo $GZ_SIM_RESOURCE_PATH
# 可以观察到GZ_SIM_RESOURCE_PATH环境变量值的变化

ament_package()
