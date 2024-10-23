cmake_minimum_required(VERSION 3.8)
project(selected_points_publisher VERSION 1.0.0)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5 COMPONENTS Core REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

qt5_wrap_cpp(MOC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/selected_points_publisher/selected_points_publisher.hpp
)
add_definitions(-DQT_NO_KEYWORDS)

add_library(${PROJECT_NAME}
  src/selected_points_publisher.cpp
  ${MOC_FILES}
  )

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(${PROJECT_NAME} pluginlib rviz_common rviz_default_plugins geometry_msgs rclcpp)

pluginlib_export_plugin_description_file(rviz_common  plugin_description.xml)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME}  PRIVATE "RVIZ2_PLUGIN_TEST_BUILDING_LIBRARY")


install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include
  )


ament_export_libraries(
  ${PROJECT_NAME} 
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
