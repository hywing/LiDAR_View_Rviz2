#! /bin/bash
RUN_PATH=$(cd "$(dirname "$0")"; pwd)

LIB_PATH=${RUN_PATH}/install
CONFIG_PATH=${RUN_PATH}/lidar_view_sensor_proc/config

NODE=${LIB_PATH}/lidar_view_sensor_proc/lib/lidar_view_sensor_proc/lidar_view_sensor_proc_node

cp ${CONFIG_PATH}/config.yaml ${LIB_PATH}/lidar_view_sensor_proc/config/

${NODE} --ros-args --params-file ${CONFIG_PATH}/config.yaml &

ros2 launch lidar_view_sensor_proc lidar_view_launch.py