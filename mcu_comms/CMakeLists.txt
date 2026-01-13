cmake_minimum_required(VERSION 3.0.2)
project(mcu_comms)

## 1. 查找依赖包
## 必须包含 serial（用于串口）和消息相关的包
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  serial
  std_msgs
)

## 2. 系统依赖（通常保持默认）
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp sensor_msgs serial std_msgs
)

## 3. 设置头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 4. 生成可执行文件 (关键修改点)
## 这里的 mcu_bridge_node 是生成的节点程序名
## src/mcu_bridge_node.cpp 是你代码存放的路径
add_executable(mcu_bridge_node src/mcu_bridge_node.cpp)

## 5. 链接库
## 告诉编译器将 ROS 库和串口库链接到你的程序上
target_link_libraries(mcu_bridge_node
  ${catkin_LIBRARIES}
)

## 6. 添加依赖顺序
## 确保在编译代码前，所有依赖的消息头文件已经生成
add_dependencies(mcu_bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})