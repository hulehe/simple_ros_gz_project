cmake_minimum_required(VERSION 3.5)

# 定义包名。必须和package.xml一致。
project(gz_worlds)

# ament_cmake包是ros团队开发并维护的，用于ros项目构建的包。
# ament_environment_hooks和ament_package都源于该包。
find_package(ament_cmake REQUIRED)

# 如果代码中依赖gz_models包，则需要设置
# find_package(gz_models REQUIRED)

# 根据需要添加以下gazebo库
# find_package(gz-cmake3 REQUIRED)
# find_package(gz-plugin2 REQUIRED COMPONENTS register)
# set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
# find_package(gz-common5 REQUIRED COMPONENTS profiler)
# set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

# 添加Gazebo Harmonic库搜索路径
find_package(gz-sim8 REQUIRED)

# 给GZ_SIM_VER变量赋值为gz-sim8_VERSION_MAJOR的值
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})



# 定义一个名为 BasicSystem 的动态库（插件是一种动态库），源码文件是当前源目录下的src/BasicSystem.cc
# SHARED 表示构建为 .so 动态链接库（而不是静态库 .a）
add_library(
  BasicSystem
  SHARED
  src/BasicSystem.cc
)

# 将 include 目录添加到 BasicSystem 目标的头文件搜索路径中
# PRIVATE 表示这些头文件仅在 BasicSystem 内部使用（不会传递给依赖它的目标）
target_include_directories(
  BasicSystem PRIVATE include
)

# 将 Gazebo Simulation 库链接到 BasicSystem 目标上
# PRIVATE 表示链接仅影响 BasicSystem 本身，不传递给依赖它的其他目标
target_link_libraries(
  BasicSystem PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)


# Following directives similarly specify another target: 'FullSystem'.
add_library(FullSystem
  SHARED
  src/FullSystem.cc
)

target_include_directories(
  FullSystem PRIVATE include
)

target_link_libraries(FullSystem PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

# 将 BasicSystem 和 FullSystem 两个目标（通常是生成的库文件）安装到指定目录
# 安装路径为安装目录下的lib/<项目名>
install(
  TARGETS BasicSystem FullSystem
  DESTINATION lib/${PROJECT_NAME}
)


# 将当前源目录下worlds目录下的全部内容复制到安装目录下的share/${PROJECT_NAME}/worlds
install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)


# Following directives are used when testing.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# 使用ament环境钩子（参考gz_models/CMakeLists.txt注释）
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

# ament_package()必须定义在CMakeLists.txt文件的最后
ament_package()