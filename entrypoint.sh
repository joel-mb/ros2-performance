#!/bin/bash
set -e

export PYTHONPATH=$PYTHONPATH:"${CARLA_ROOT}/PythonAPI/carla/dist/$(ls ${CARLA_ROOT}/PythonAPI/carla/dist | grep py3.)"
export PYTHONPATH=$PYTHONPATH:"${CARLA_ROOT}/PythonAPI/carla"

source "/opt/ros/$ROS_DISTRO/setup.bash"

echo "0.9.14" > ${ROS2_WS}/src/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION

exec "$@"
