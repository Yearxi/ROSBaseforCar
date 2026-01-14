cmake_minimum_required(VERSION 3.5)
project(mcu_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED) # 确保上面 git clone 的 serial 库已被 colcon 识别

add_executable(mcu_bridge_node src/mcu_bridge.cpp)

# 注意：Foxy 可能需要显式链接 tf2_geometry_msgs
ament_target_dependencies(mcu_bridge_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# 链接 serial 库
target_link_libraries(mcu_bridge_node serial)

install(TARGETS
  mcu_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
