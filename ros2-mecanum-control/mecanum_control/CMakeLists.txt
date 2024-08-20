cmake_minimum_required(VERSION 3.8)
project(mecanum_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Disable boost/shared_ptr in pluginlib while building on rpi4
add_compile_definitions(PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp REQUIRED)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)
add_executable(mecanum_control_node
  src/mecanum_control_node.cpp
)
ament_target_dependencies(
  mecanum_control_node
  controller_manager
  realtime_tools
  rclcpp
)

install(TARGETS
  mecanum_control_node
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
