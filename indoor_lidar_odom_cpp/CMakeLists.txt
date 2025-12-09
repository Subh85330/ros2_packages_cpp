cmake_minimum_required(VERSION 3.8)
project(indoor_lidar_odom_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)


add_executable(indoor_lidar_odom_node src/indoor_lidar_odom_node.cpp)
ament_target_dependencies(indoor_lidar_odom_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  pcl_conversions
)

add_executable(indoor_lidar_odom_ekf_node src/indoor_lidar_odom_ekf_node.cpp)
# add_executable(record_odom_to_csv src/record_odom_to_csv.py)
ament_target_dependencies(indoor_lidar_odom_ekf_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

target_link_libraries(indoor_lidar_odom_ekf_node
  Eigen3::Eigen
)

install(TARGETS
  indoor_lidar_odom_node
  indoor_lidar_odom_ekf_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
