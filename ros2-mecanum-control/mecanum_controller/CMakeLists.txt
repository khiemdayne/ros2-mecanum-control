cmake_minimum_required(VERSION 3.8)
project(mecanum_controller)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# find_package(controller_manager REQUIRED)

set(dependencies
  controller_interface
  hardware_interface
  realtime_tools
  rclcpp
  rclcpp_lifecycle
  pluginlib
)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mecanum_controller.cpp
  src/mecanum_wheel.cpp
)
target_compile_features(mecanum_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  ${dependencies}
)
pluginlib_export_plugin_description_file(controller_interface mecanum_controller_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  ${dependencies}
)

ament_package()
