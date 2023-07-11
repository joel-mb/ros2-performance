# Copyright (c) 2023 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import queue
import threading
import time

import rclpy
import rclpy.node
import rclpy.qos

import carla

from sensor_msgs.msg import Image

THRESHOLD = 0.001
SKIP_TICKS = 100
DURATION_TICKS = 250
FIXED_DELTA_SECONDS = 0.05

SENSORS_TIMEOUT = 0.5

##################
## EXPERIMENT 1 ##
##################
VEHICLE_CONFIGURATION = {
    "type": "vehicle.tesla.model3",
    "id": "ego_vehicle",
    "sensors": [
        {
            "type": "sensor.camera.rgb",
            "id": "rgb_view",
            "spawn_point": {"x": -4.5, "y": 0.0, "z": 2.8, "roll": 0.0, "pitch": 20.0, "yaw": 0.0},
            "attributes": {
                "image_size_x": 400,
                "image_size_y": 400,
                "fov": 90.0
            },
            "ros": {
                "name": "image",
                "msg": Image
            }
        }
    ]
}

##################
## EXPERIMENT 2 ##
##################
# VEHICLE_CONFIGURATION = {
#     "type": "vehicle.tesla.model3",
#     "id": "ego_vehicle",
#     "sensors": [
#         {
#             "type": "sensor.camera.rgb",
#             "id": "rgb_view",
#             "spawn_point": {"x": -4.5, "y": 0.0, "z": 2.8, "roll": 0.0, "pitch": 20.0, "yaw": 0.0},
#             "attributes": {
#                 "image_size_x": 400,
#                 "image_size_y": 400,
#                 "fov": 90.0
#             },
#             "ros": {
#                 "name": "image",
#                 "msg": Image
#             }
#         }
#     ]
# }

##################
## EXPERIMENT 3 ##
##################
# VEHICLE_CONFIGURATION = {
#     "type": "vehicle.tesla.model3",
#     "id": "ego_vehicle",
#     "sensors": [
#         {
#             "type": "sensor.camera.rgb",
#             "id": "rgb_view",
#             "spawn_point": {"x": -4.5, "y": 0.0, "z": 2.8, "roll": 0.0, "pitch": 20.0, "yaw": 0.0},
#             "attributes": {
#                 "image_size_x": 400,
#                 "image_size_y": 400,
#                 "fov": 90.0
#             },
#             "ros": {
#                 "name": "image",
#                 "msg": Image
#             }
#         }
#     ]
# }


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
        bp.set_attribute("role_name", "ego_vehicle")
        bp.set_attribute("ros_name", config.get("id")) 

        self.get_logger().info("Spawning vehicle...")

        return  self._world.spawn_actor(
            self._blueprint_library.filter(config.get("type"))[0],
            self._map.get_spawn_points()[0],
            attach_to=None)

    def _callback(self, data, id):
        stamp = data.header.stamp
        self.sensors_queue.put_nowait((id, stamp.sec + stamp.nanosec*1e-9))

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
                    attach_to=vehicle,
                    attachment_type=carla.AttachmentType.SpringArmGhost
                )
            )

            self._world.tick()
            self._world.tick()

            sensors[-1].listen(lambda data: None)

            self.create_subscription(
                Image,
                "/carla/{}/{}/{}".format(VEHICLE_CONFIGURATION["id"], sensor["id"], sensor["ros"]["name"]),
                lambda msg: self._callback(msg, sensor["id"]),
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
    settings.fixed_delta_seconds = FIXED_DELTA_SECONDS
    world.apply_settings(settings)

    node = ROS2PerformanceNode(world)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
    spin_thread.start()

    mean_elapsed = 0
    total_frames = 0

    try:
        node.get_logger().info(">>> Initializing test ({} ticks)...".format(SKIP_TICKS)) 

        while total_frames < DURATION_TICKS:

            start_time = time.time()
            frame = world.tick()
            snapshot = world.get_snapshot()

            received_sensors = {}
            while len(received_sensors) < len(VEHICLE_CONFIGURATION["sensors"]):
                try:
                    sensor, timestamp = node.sensors_queue.get(True, SENSORS_TIMEOUT)
                    received_sensors[sensor] = ((frame, timestamp))

                    if abs(timestamp - snapshot.timestamp.elapsed_seconds) > THRESHOLD:
                         node.get_logger().warn("We are receiving data from a different frame: {} vs {}".format(snapshot.timestamp, timestamp))
    
                except queue.Empty:

                    node.get_logger().warn("A sensor took too long to send their data. Frame {}".format(total_frames))
                    break
                except Exception as e:
                    break

            assert node.sensors_queue.empty()

            end_time = time.time()


            total_frames += 1

            if total_frames <= SKIP_TICKS:
                continue
            else:
                elapsed = end_time - start_time
                mean_elapsed = (mean_elapsed * (total_frames - SKIP_TICKS - 1) + elapsed) / (total_frames - SKIP_TICKS)
                node.get_logger().info(">>> {}, {}, {}".format(total_frames, elapsed, mean_elapsed)) 

    except Exception as e:
        node.get_logger().warn(e)

    finally:
        # Final results
        node.get_logger().info(">>> FINAL RESULTS") 
        node.get_logger().info(">>>     Duration test: {0:.4f} s ({1:.2f} ticks)".format(DURATION_TICKS*FIXED_DELTA_SECONDS, DURATION_TICKS)) 
        node.get_logger().info(">>>     Mean sec     : {0:.4f} s".format(mean_elapsed)) 
        node.get_logger().info(">>>     Mean fps     : {0:.4f} fps".format(1. / mean_elapsed)) 


        if original_settings:
            world.apply_settings(original_settings)

        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
