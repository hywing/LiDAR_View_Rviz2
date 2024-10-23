cmake_minimum_required(VERSION 3.5)

#-------------------------------------------------------------------------------
# System config
#-------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CUDA_STANDARD 11)

# set(CMAKE_C_STANDARD 99)
# set(CMAKE_VERBOSE_MAKEFILE on)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -g -ggdb -O2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")


project(lidar_view_sensor_proc VERSION 1.0.0)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.10 REQUIRED)

if($ENV{ROS_DISTRO} STREQUAL "foxy")
add_definitions(-DROS2_FOXY_VERSION)
endif()

include_directories(src include common ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(DIR_SRC
    src/main.cpp
    src/pcl_node.cpp
    src/frame_store/frame_store.cpp
    src/rosbag_reader/rosbag_reader.cpp
    include/read_param.cpp
)

add_executable(${PROJECT_NAME}_node  ${DIR_SRC} ${SRC_LIST})
ament_target_dependencies(${PROJECT_NAME}_node  
                            rclcpp 
                            rosidl_typesupport_cpp 
                            tf2 
                            tf2_ros 
                            std_msgs sensor_msgs  
                            rosidl_runtime_cpp 
                            rosbag2_cpp 
                            pcl_conversions)
                            
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES})

install(TARGETS 
  ${PROJECT_NAME}_node 
  DESTINATION lib/${PROJECT_NAME}/)

install(DIRECTORY config/
    DESTINATION config/
    USE_SOURCE_PERMISSIONS)

install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME})

ament_package()
