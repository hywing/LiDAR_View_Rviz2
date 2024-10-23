#! /bin/bash
RUN_PATH=$(cd "$(dirname "$0")"; pwd)
LIB_PATH=${RUN_PATH}/../../install
CONFIG_PATH=${RUN_PATH}/../config

${LIB_PATH}/lidar_view_sensor_proc/lib/lidar_view_sensor_proc/lidar_view_sensor_proc_node  --ros-args --params-file ${CONFIG_PATH}/config.yaml

