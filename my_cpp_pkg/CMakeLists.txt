cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(my_first_node src/cpp_mini.cpp)
ament_target_dependencies(my_first_node rclcpp)

add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(led_panel src/led_panel.cpp)
ament_target_dependencies(led_panel rclcpp example_interfaces robot_interfaces)

add_executable(battery src/battery.cpp)
ament_target_dependencies(battery rclcpp example_interfaces robot_interfaces)


install(TARGETS
  my_first_node
  number_publisher
  number_counter
  led_panel
  battery
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
