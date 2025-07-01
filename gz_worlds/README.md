# gz_worlds
该包用于存储gazebo world的相关文件。

## gz_world包结构
- `worlds`文件夹存储sdf文件
- `include`和`src`目录用于保存world自定义插件代码
    - BasicSystem.cc和FullSystem.cc是插件代码案例，没有执行任何功能
        - Configure方法在gazebo启动时被调用
        - PreUpdate方法在每一帧刷新前被调用，经常用于设置仿真物体的属性
        - Update方法在每一帧刷新时被调用（不经常使用）
        - PostUpdate方法在每一帧刷新后被调用，经常用于获取仿真物体的最新属性
        - Reset方法在仿真被reset后被调用，用于设置仿真物体的初始属性
- `hooks`文件夹保存被ament_environment_hooks函数调用的dsv.ini配置文件
    - gz_worlds.dsv.ini文件中配置了GZ_SIM_RESOURCE_PATH环境变量，使其指向安装目录下的share/@PROJECT_NAME@/words文件夹。
    - gz_worlds.dsv.ini文件中还配置了GZ_SIM_SYSTEM_PLUGIN_PATH变量，指向安装目录下的lib/@PROJECT_NAME@/。
- `CMakeLists.txt`文件是cmake配置文件。内置注释。
- `package.xml`文件是ros配置文件。
    - 如果world代码中需要引入gz_models中的sdf文件，则说明该包依赖gz_models包。必须在package.xml文件中添加<depend>gz_models</depend>，让构建程序按照正确顺序构建。否则，会在构建中报错。
