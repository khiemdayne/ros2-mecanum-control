cmake_minimum_required(VERSION 3.8)
project(kinco_drive_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(hardware_interface REQUIRED)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(kinco_drive_hardware 
  kinco_drive_hardware/kinco_drive_hardware.cpp)

target_compile_features(kinco_drive_hardware PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(kinco_drive_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  kinco_drive_hardware
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_compile_definitions(kinco_drive_hardware PRIVATE "KINCO_DRIVE_HARDWARE_BUILDING_LIBRARY")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface kinco_drive_hardware.xml)

#install
install(
  DIRECTORY kinco_drive_hardware/include/
  DESTINATION include/kinco_drive_hardware
)
install(
  TARGETS kinco_drive_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  kinco_drive_hardware
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
