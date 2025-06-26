# hooks
- 钩子方法，用于动态配置环境变量。
- hooks文件夹内存放有dsv.ini配置文件。在编译过程中，CMake通过解析CMakeLists.txt文件中的ament_environment_hooks函数，通过调用dsv.ini配置文件配置环境变量。  
- gz_models.dsv.ini文件中配置了GZ_SIM_RESOURCE_PATH环境变量，使其指向安装目录下的share/@PROJECT_NAME@/models/sdf文件夹。
- 该项目是使用了gazebo运行后以动态生成model的方式启动model，所以没有使用该钩子方法，。