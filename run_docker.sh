#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -z "$CARLA_ROOT" ]; then
  echo "Error $CARLA_ROOT is empty. Set \$CARLA_ROOT as an environment variable first."
  exit 1
fi
echo "Using the CARLA version at '$CARLA_ROOT'"

docker run \
    -it \
    --rm \
    --privileged \
    --ipc=host \
    --pid=host \
    --net=host \
    --volume=${CARLA_ROOT}/PythonAPI:/workspace/CARLA/PythonAPI:ro \
    --volume ${SCRIPT_DIR}/_config:/config \
    --volume=${SCRIPT_DIR}/scripts:/workspace/scripts:rw \
    --volume=${SCRIPT_DIR}/ws/src/performance_ros2:/workspace/ws/src/performance_ros2:rw \
    --volume=${SCRIPT_DIR}/_submodules/ros-bridge/carla_ros_bridge:/workspace/ws/src/carla_ros_bridge:rw \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastrtps-profile.xml \
    performance_ros2:latest /bin/bash
