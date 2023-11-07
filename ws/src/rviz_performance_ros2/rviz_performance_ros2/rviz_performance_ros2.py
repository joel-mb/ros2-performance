# Copyright (c) 2023 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import queue
import threading

import rclpy
import rclpy.node
import rclpy.qos

import tf2_ros

import carla
import math

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3, Quaternion, Transform, TransformStamped
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Header

from transforms3d.euler import euler2quat


THRESHOLD = 0.001
FIXED_DELTA_SECONDS = 0.05

SENSORS_TIMEOUT = 10.0

##########################
## SENSOR CONFIGURATION ##
##########################
VEHICLE_CONFIGURATION = {
    "type": "vehicle.tesla.model3",
    "id": "hero",
    "sensors": [
        {
            "type": "sensor.camera.rgb",
            "id": "rgb",
            "spawn_point": {"x": -4.5, "y": 0.0, "z": 2.5, "roll": 0.0, "pitch": -20.0, "yaw": 0.0},
            "attributes": {
                "image_size_x": 1920,
                "image_size_y": 1080,
                "fov": 90.0
            },
            "ros": {
                "name": "image",
                "msg": Image
            }
        },
        {
            "type": "sensor.lidar.ray_cast",
            "id": "lidar",
            "spawn_point": {"x": 0.0, "y": 0.0, "z": 2.4, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            "attributes": {
                "range": 85,
                "channels": 64,
                "points_per_second": 2000000,
                "rotation_frequency": 10,
                "upper_fov": 10,
                "lower_fov": -30,
                "atmosphere_attenuation_rate": 0.004,
                "dropoff_general_rate": 0.45,
                "dropoff_intensity_limit": 0.8,
                "dropoff_zero_intensity": 0.4
            },
            "ros": {
                "name": "",
                "msg": PointCloud2
            }
        }
    ]
}


class ROS2PerformanceNode(rclpy.node.Node):

    def __init__(self, world):
        super().__init__("performance_ros2", parameter_overrides=[rclpy.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self._world = world
        self._map = world.get_map()
        self._blueprint_library = world.get_blueprint_library()

        self.sensors_queue = queue.Queue()

        self.vehicle = self._setup_vehicle(VEHICLE_CONFIGURATION)
        self.sensors = self._setup_sensors(self.vehicle, VEHICLE_CONFIGURATION.get("sensors", []))

    def _setup_vehicle(self, config):
        bp = self._blueprint_library.filter(config.get("type"))[0]
        bp.set_attribute("role_name", config.get("id"))
        bp.set_attribute("ros_name", config.get("id")) 

        self.get_logger().info("Spawning vehicle...")

        return  self._world.spawn_actor(
            bp,
            self._map.get_spawn_points()[0],
            attach_to=None)

    def _callback(self, data):
        stamp = data.header.stamp
        self.sensors_queue.put_nowait(stamp.sec + stamp.nanosec*1e-9)

    def _setup_sensors(self, vehicle, sensors_config):
        sensors = []

        for sensor in sensors_config:
            self.get_logger().info("Spawning sensor: {}".format(sensor))

            bp = self._blueprint_library.filter(sensor.get("type"))[0]
            bp.set_attribute("ros_name", sensor.get("id")) 
            bp.set_attribute("role_name", sensor.get("id")) 
            for key, value in sensor.get("attributes", {}).items():
                bp.set_attribute(str(key), str(value))

            wp = carla.Transform(
                location=carla.Location(
                    x=sensor["spawn_point"]["x"],
                    y=sensor["spawn_point"]["y"],
                    z=sensor["spawn_point"]["z"]
                ),
                rotation=carla.Rotation(
                    roll=sensor["spawn_point"]["roll"],
                    pitch=sensor["spawn_point"]["pitch"],
                    yaw=sensor["spawn_point"]["yaw"]
                )
            )

            sensors.append(
                self._world.spawn_actor(
                    bp,
                    wp,
                    attach_to=vehicle
                )
            )

            sensors[-1].enable_for_ros()

            topic_name = "/carla/{}/{}".format(VEHICLE_CONFIGURATION["id"], sensor["id"])
            if sensor["ros"]["name"]:
                topic_name += "/" + sensor["ros"]["name"] 

            self.create_subscription(
                sensor["ros"]["msg"],
                topic_name,
                self._callback,
                qos_profile=10
            )

        return sensors

    def destroy_node(self):
        self.get_logger().info("Destroying carla actors...")

        for sensor in self.sensors:
            sensor.destroy()
        self.vehicle.destroy()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    client = carla.Client("localhost", 2000)
    client.set_timeout(60.0)

    world = client.get_world()
    original_settings = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    node = ROS2PerformanceNode(world)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    spin_thread.start()

    tf_broadcaster = tf2_ros.TransformBroadcaster(node)

    try:
        total_frames = 0

        while True:
            _ = world.tick()
            snapshot = world.get_snapshot()

            if total_frames == 0: node.vehicle.set_autopilot(True)

            received_sensors = []
            while len(received_sensors) < len(VEHICLE_CONFIGURATION["sensors"]):
                try:
                    timestamp = node.sensors_queue.get(True, SENSORS_TIMEOUT)

                    if abs(timestamp - snapshot.timestamp.elapsed_seconds) < THRESHOLD:
                        received_sensors.append(timestamp)
                    else:
                        node.get_logger().warn("We are receiving data from a different frame: {} vs {}".format(snapshot.timestamp.elapsed_seconds, timestamp))

                except queue.Empty:

                    node.get_logger().warn("A sensor took too long to send their data. Frame {}".format(total_frames))
                    break
                except Exception as e:
                    break

            assert node.sensors_queue.empty()

            # Publish ego tf
            sec = snapshot.timestamp.elapsed_seconds
            _time = Time(sec=int(sec), nanosec=int((sec - int(sec)) * 1000000000))
            
            quat = euler2quat(
                math.radians(node.vehicle.get_transform().rotation.roll),
                -math.radians(node.vehicle.get_transform().rotation.pitch),
                -math.radians(node.vehicle.get_transform().rotation.yaw)
            )
            _transform = Transform()
            _transform.translation = Vector3(x=node.vehicle.get_transform().location.x, y=-node.vehicle.get_transform().location.y, z=node.vehicle.get_transform().location.z)
            _transform.rotation = Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[3])

            tf_broadcaster.sendTransform(TransformStamped(
                header=Header(frame_id="map", stamp=_time),
                child_frame_id="hero",
                transform=_transform)
            )

            total_frames += 1

    except Exception as e:
        node.get_logger().warn(e)

    finally:
        if original_settings:
            world.apply_settings(original_settings)

        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
