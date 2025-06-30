# sdf
- 用于保存下载的gazebo model sdf文件。通过CMake安装和ament钩子函数进行GZ_SIM_RESOURCE_PATH配置。
- 可以在gazebo world文件中静态引入该文件内容，也可以使用gazebo生成功能，在gazebo启动后动态生成物体。