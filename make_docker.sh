#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

rm -fr ${SCRIPT_DIR}/.tmp

echo "Creating a temporary folder"
mkdir -p ${SCRIPT_DIR}/.tmp

echo "Copying the ros2 workspace into the folder"
cp -fr ${SCRIPT_DIR}/scripts/ ${SCRIPT_DIR}/.tmp/scripts
cp -fr ${SCRIPT_DIR}/ws/ ${SCRIPT_DIR}/.tmp/ws
cp -fr ${SCRIPT_DIR}/_submodules/ros-bridge/carla_ros_bridge  ${SCRIPT_DIR}/.tmp/ws/src
cp -fr ${SCRIPT_DIR}/_submodules/ros-bridge/ros_compatibility  ${SCRIPT_DIR}/.tmp/ws/src
cp -fr ${SCRIPT_DIR}/_submodules/ros-bridge/carla_common  ${SCRIPT_DIR}/.tmp/ws/src
cp -fr ${SCRIPT_DIR}/_submodules/ros-bridge/carla_msgs  ${SCRIPT_DIR}/.tmp/ws/src

cp ${SCRIPT_DIR}/entrypoint.sh ${SCRIPT_DIR}/.tmp/entrypoint.sh

echo "Building the docker"
docker build --force-rm -t performance_ros2 -f Dockerfile ${SCRIPT_DIR}/.

echo "Removing the temporary folder"
rm -fr ${SCRIPT_DIR}/.tmp
