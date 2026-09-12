cmake_minimum_required(VERSION 3.8)
project(orange_sensor_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
ament_target_dependencies(laserscan_multi_merger
  rclcpp
  rcl_interfaces
  sensor_msgs
  tf2
  tf2_ros
  laser_geometry
  pcl_conversions
  pcl_ros
)
target_include_directories(laserscan_multi_merger PUBLIC ${PCL_INCLUDE_DIRS})
target_link_libraries(laserscan_multi_merger ${PCL_LIBRARIES})
install(TARGETS laserscan_multi_merger DESTINATION lib/${PROJECT_NAME})

add_executable(led_node src/led_node.cpp)
ament_target_dependencies(led_node rclcpp std_msgs serial)
install(TARGETS led_node DESTINATION lib/${PROJECT_NAME})

add_executable(wheel_imu_odom src/wheel_imu_odom.cpp)
ament_target_dependencies(wheel_imu_odom
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS wheel_imu_odom DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config firmware launch DESTINATION share/${PROJECT_NAME}/)

ament_package()
