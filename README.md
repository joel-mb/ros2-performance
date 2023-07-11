# ROS2 performance experiments

**IMPORTANT:** You need to set a `CARLA_ROOT` env variable pointing to your CARLA target folder.

```bash
git clone --recurse-submodules https://github.com/joel-mb/ros2-performance && cd ros2-performance

# Patch CARLA ROS bridge
echo '0.9.14' > _submodules/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION
```

```bash
./make_docker.sh
```

```bash
./run_docker.sh

# CARLA native test
python3 scripts/performance_carla_native.py

# ROS2 tests
cd ws
colcon build
source install/setup.bash

# ROS2 native
ros2 launch performance_ros2 performance_ros2_native.launch.py

# ROS2 bridge
ros2 launch performance_ros2 performance_ros2_bridge.launch.py
```
