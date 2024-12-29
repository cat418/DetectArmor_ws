cmake_minimum_required(VERSION 3.8)
project(armor_interfaces)#构建一个名为armor_interfaces的ROS2接口包

find_package(ament_cmake REQUIRED)#用于cmake项目的构建
find_package(ament_cmake_auto REQUIRED)#自动为 CMake 项目配置依赖关系、目标、消息和服务的生成等。
find_package(geometry_msgs REQUIRED)#包含与几何形状相关的消息类型
find_package(rosidl_default_generators REQUIRED)#提供了生成c++代码和Python代码的工具

#用于生成ROS2消息代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Armor.msg"
  "msg/Armors.msg"
  "msg/Target.msg"
  DEPENDENCIES#列出这些消息依赖的其他消息包
    std_msgs#std_msgs 是标准消息类型的包，包含常见的数据类型，如 String、Int32 等。
    geometry_msgs
)

ament_package()
