cmake_minimum_required(VERSION 3.8)
project(rviz2_plugins VERSION 1.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

if($ENV{ROS_DISTRO} STREQUAL "foxy")
add_definitions(-DROS2_FOXY_VERSION)
endif()

qt5_wrap_cpp(MOC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/play_panel/play_panel.h
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/play_panel/)

add_library(lidar_view_rviz2_plugins
    ${CMAKE_CURRENT_SOURCE_DIR}/play_panel/play_panel.cpp
    ${MOC_FILES}
)

target_compile_features(lidar_view_rviz2_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(lidar_view_rviz2_plugins pluginlib rviz_common geometry_msgs rclcpp)

pluginlib_export_plugin_description_file(rviz_common lidar_view_rviz2_plugins.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(lidar_view_rviz2_plugins PRIVATE "RVIZ2_PLUGIN_TEST_BUILDING_LIBRARY")

# install(
#   DIRECTORY play_panel
#   DESTINATION include/
# )

install(
  TARGETS lidar_view_rviz2_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ament_export_include_directories(
#   include
# )
ament_export_libraries(
  lidar_view_rviz2_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
