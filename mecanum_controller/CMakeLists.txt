cmake_minimum_required(VERSION 3.8)
project(mecanum_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wno-unused-variable -Wno-unused-parameter -Wextra -Wpedantic -std=c++17 -O2)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcpputils
  realtime_tools
  tf2
  tf2_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(mecanum_controller_parameters
  src/mecanum_controller_parameter.yaml
)

add_library(mecanum_controller SHARED
  src/mecanum_controller.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
)
target_compile_features(mecanum_controller PUBLIC cxx_std_17)
target_include_directories(mecanum_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mecanum_controller>
)
target_link_libraries(mecanum_controller PUBLIC mecanum_controller_parameters)
ament_target_dependencies(mecanum_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mecanum_controller PRIVATE "MECANUM_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface mecanum_plugin.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_mecanum_controller
    test/test_mecanum_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_mecanum_controller.yaml)
  target_link_libraries(test_mecanum_controller
    mecanum_controller
  )
  ament_target_dependencies(test_mecanum_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
  )

  ament_add_gmock(test_load_mecanum_controller
    test/test_load_mecanum_controller.cpp
  )
  ament_target_dependencies(test_load_mecanum_controller
    controller_manager
    ros2_control_test_assets
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include/mecanum_controller
)
install(TARGETS mecanum_controller mecanum_controller_parameters
  EXPORT export_mecanum_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_mecanum_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
