# ros_application

## 包结构
- `nodes`：存放ros节点c++代码
- `CMakeLists.txt`：cmake配置文件。内置注释。
- `package.xml`：ros配置文件。

## 运行
- 单独启动ros节点，并打印RCLCPP_DEBUG输出
```bash
ros2 run ros_application obstacle_avoider --ros-args --log-level DEBUG
```