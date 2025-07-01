# gz_models

## gz_models包结构
- 该包用于存储gazebo model的相关文件
- model文件夹存储urdf文件和sdf文件
- 如果需要编写插件，可以保存到include和src目录下（参考gz_worlds）
- hooks文件夹保存被ament_environment_hooks函数调用的dsv.ini配置文件

## ament_environment_hooks 钩子方法的使用
- ament_environment_hooks钩子是ROS2自带的ament包中的方法，通过添加setup文件内容，在程序运行前配置需要的环境变量。
- hooks文件夹内存放有dsv.ini配置文件。在编译过程中，CMake通过解析CMakeLists.txt文件中的ament_environment_hooks函数，通过调用dsv.ini配置setup文件内容。
- gz_models.dsv.ini文件中配置了GZ_SIM_RESOURCE_PATH环境变量，使其指向安装目录下的share/@PROJECT_NAME@/models/sdf文件夹。
- gz_models.dsv.ini文件中还配置了GZ_SIM_SYSTEM_PLUGIN_PATH变量，指向安装目录下的lib/@PROJECT_NAME@/。这样在程序运行时，model才能成功加载插件。
- 该项目是使用了gazebo运行后以动态生成model的方式启动model，所以配置GZ_SIM_RESOURCE_PATH环境变量没有作用。

## 手动转化xacro文件到sdf文件
### 背景
该项目中，在使用gazebo harmonic库转化urdf文件为sdf文件时，出现collision的name无法正常解析的错误，导致contact sensor无法正常使用。为了调试程序，使用手动完成从xacro到sdf的转化。

### 手动转化命令
- xacro文件转化为urdf文件
```
ros2 run xacro xacro /path/input.xacro argument_name:=value -o /path/output.urdf
```

- urdf文件转化为sdf文件
```
gz sdf -p /path/input.urdf > /path/output.sdf
```