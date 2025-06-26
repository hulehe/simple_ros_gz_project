# src
- 该文件夹用于存放world的插件源代码
- BasicSystem.cc和FullSystem.cc是插件代码案例，没有执行任何功能
    - Configure方法在gazebo启动时被调用
    - PreUpdate方法在每一帧刷新前被调用，经常用于设置仿真物体的属性
    - Update方法在每一帧刷新时被调用（不经常使用）
    - PostUpdate方法在每一帧刷新后被调用，经常用于获取仿真物体的最新属性
    - Reset方法在仿真被reset后被调用，用于设置仿真物体的初始属性