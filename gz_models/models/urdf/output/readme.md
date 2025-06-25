# 关于这个文件夹
## 概要
- 该项目中，在使用gazebo harmonic库转化urdf文件为sdf文件时，出现collision的name无法正常解析的错误。所以，手动完成从xacro到sdf的转化。
- 使用ros命令和gz命令，将xacro文件转化成的urdf和sdf文件保存在该文件夹。
- 该文件夹及生成文件只用于测试。没有该文件夹也不直接影响该项目的运行。

## 转化方法
### xacro文件转化为urdf文件
```
ros2 run xacro xacro /path/input.xacro argument_name:=value -o /path/output.urdf
```

### urdf文件转化为sdf文件
```
gz sdf -p /path/input.urdf > /path/output.sdf
```