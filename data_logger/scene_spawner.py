import math
import os
import random
import time
from threading import Event

import rclpy
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from tf_transformations import quaternion_from_euler


class SceneSpawner(Node):
    def __init__(self):
        super().__init__("scene_spawner")

        # Categorize obstacles by size
        self.small_obstacles = [
            "Beer",
            "Black_ball",
            "Black_ball_transparent",
            "Cinder block wide",
            "Coke Can",
            "Green ball",
            "Red Ball",
        ]

        self.normal_obstacles = [
            "Cabinet",
            "Car Wheel",
            "Cardboard Box",
            "Cinder block new",
            "Construction Barrel",
            "Construction Cone",
            "Fire Hydrant",
            "Mailbox",
            "Standing Person",
            "Transparent_Cone",
            "Mesh",
            "Mesh round",
        ]
        # "Transparent_sheet", #add this back in normal_obstacles when we want normal obstacles

        self.large_obstacles = [
            "Hatchback",
            "Dumpster",
            "Jersey barrier",
        ]

        # Combine all obstacles for iteration
        self.obstacles = (
            self.small_obstacles + self.normal_obstacles + self.large_obstacles
        )

        self.model_path_base = os.path.expanduser("~/.gazebo/models")

        self.spawn_client = self.create_client(SpawnEntity, "/spawn_entity")
        self.delete_client = self.create_client(DeleteEntity, "/delete_entity")
        self.trigger_pub = self.create_publisher(Float32, "/trigger_sample", 10)
        self.obstacle_ready_event = Event()
        self.obstacle_ready_event.clear()
        self.obstacle_ready_sub = self.create_subscription(
            Bool, "/obstacle_ready", self.ready_callback, 10
        )

        # Create parameter client for GroundTruthListener
        self.param_client = self.create_client(
            SetParameters, "/ground_truth_listener/set_parameters"
        )
        # Declare parameter for obstacle_name
        self.declare_parameter("obstacle_name", "unknown")
        self.obstacle_name = (
            self.get_parameter("obstacle_name").get_parameter_value().string_value
        )

        # New parameter for transparency mode
        self.declare_parameter("transparent_mode", False)
        self.transparent_mode = (
            self.get_parameter("transparent_mode").get_parameter_value().bool_value
        )

        # Track spawned objects for cleanup
        self.spawned_objects = []

        self.obstacle_pub = self.create_publisher(String, "/current_obstacle", 10)

        self.run_sequence()

    def ready_callback(self, msg):
        if msg.data:
            self.get_logger().info("🟢 Ground truth ready signal received.")
            self.obstacle_ready_event.set()

    def wait_for_services(self):
        self.get_logger().info(
            "Waiting for /spawn_entity and /delete_entity services and /ground_truth_listener/set_parameters service..."
        )
        self.spawn_client.wait_for_service()
        self.delete_client.wait_for_service()
        self.param_client.wait_for_service()
        self.get_logger().info("Services are available.")

    def run_sequence(self):
        self.wait_for_services()

        for i in range(25):  # Outer loop to iterate twice
            for idx, obstacle in enumerate(self.obstacles):  # Use enumerate for index
                self.get_logger().info(
                    f"\n🔄 Iteration {i + 1} — Spawning {obstacle}..."
                )

                # ✅ Delete previously spawned obstacles
                self.cleanup_spawned_objects()

                if self.transparent_mode or obstacle == "transparent":
                    # Special mode: spawn Transparent_sheet with another random obstacle
                    self.get_logger().info(
                        f"🔍 Transparent mode active - spawning Transparent_sheet with {obstacle}"
                    )

                    # Spawn the transparent sheet
                    self.spawn_transparent_sheet()

                    # Spawn the regular obstacle
                    self.spawn_model(obstacle)

                    # Add both to tracking list
                    self.spawned_objects.extend(["Transparent_sheet", obstacle])

                    # ✅ Set ground truth listener parameter to Transparent_sheet
                    self.set_ground_truth_obstacle_name("Transparent_sheet")

                    # ✅ Set obstacle_name parameter on data_logger
                    self.get_logger().info(
                        "🛠 Setting data_logger parameter: obstacle_name = Transparent_sheet"
                    )
                    self.set_logger_obstacle_param("Transparent_sheet")
                else:
                    # Regular mode: spawn single obstacle
                    # ✅ Spawn new obstacle
                    self.spawn_model(obstacle)
                    self.spawned_objects.append(obstacle)

                    # ✅ Set ground truth listener parameter
                    self.set_ground_truth_obstacle_name(obstacle)

                    # ✅ Set obstacle_name parameter on data_logger
                    self.get_logger().info(
                        f"🛠 Setting data_logger parameter: obstacle_name = {obstacle}"
                    )
                    self.set_logger_obstacle_param(obstacle)

                self.get_logger().info("⏳ Waiting for scene to stabilize...")
                time.sleep(10)

                self.get_logger().info("📸 Triggering data collection...")
                self.trigger_pub.publish(Float32(data=1.0))

                # Publish the primary obstacle name
                primary_obstacle = (
                    "Transparent_sheet"
                    if self.transparent_mode or obstacle == "transparent"
                    else obstacle
                )
                self.obstacle_pub.publish(String(data=primary_obstacle))

                time.sleep(2)

        # ✅ Clean up last obstacles after all loops
        self.cleanup_spawned_objects()

        self.get_logger().info("✅ All samples collected. Process complete.")

    def cleanup_spawned_objects(self):
        """Delete all currently spawned objects"""
        for obj in self.spawned_objects:
            self.get_logger().info(f"🧹 Removing: {obj}")
            self.delete_model(obj)
        self.spawned_objects = []

    def spawn_transparent_sheet(self):
        """Spawn a transparent sheet in front of the robot"""
        req = SpawnEntity.Request()
        req.name = "Transparent_sheet"
        req.xml = open(
            f"{self.model_path_base}/Transparent_sheet/model.sdf", "r"
        ).read()
        req.robot_namespace = "Transparent_sheet"
        req.reference_frame = "world"

        # Position within 1m of the robot, in front
        radius = random.uniform(0.5, 1.0)
        angle = random.uniform(-math.pi / 10, math.pi / 10)  # Narrow angle in front

        req.initial_pose.position.x = radius * math.cos(angle)
        req.initial_pose.position.y = radius * math.sin(angle)
        req.initial_pose.position.z = 0.0

        # Random orientation
        yaw = random.uniform(0, math.pi / 3)
        quat = quaternion_from_euler(0, 0, yaw)
        req.initial_pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
                f"Spawned Transparent_sheet at x={req.initial_pose.position.x:.2f}, "
                f"y={req.initial_pose.position.y:.2f}, distance={radius:.2f}m, yaw={math.degrees(yaw):.1f}°"
            )
        else:
            self.get_logger().error("Failed to spawn Transparent_sheet")

    def spawn_model(self, model_name):
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = open(f"{self.model_path_base}/{model_name}/model.sdf", "r").read()
        req.robot_namespace = model_name
        req.reference_frame = "world"

        # Determine distance range based on object size
        if model_name in self.small_obstacles:
            # Small objects: 1.0 to 3.0 meters
            radius = random.uniform(0.8, 2.5)
            self.get_logger().info("🔹 Small obstacle - using closer range (0.8-2.5m)")
        elif model_name in self.normal_obstacles:
            # Normal objects: 1.0 to 5.0 meters
            radius = random.uniform(1.5, 5.0)
            self.get_logger().info("🔸 Normal obstacle - using medium range (1.5-5.0m)")
        else:  # Large obstacles
            # Large objects: 4.0 to 8.0 meters
            radius = random.uniform(3.0, 7.0)
            self.get_logger().info("🔶 Large obstacle - using farther range (3.0-7.0m)")

        # Random angle in front of the robot (±30°)
        angle = random.uniform(-math.pi / 6, math.pi / 6)

        req.initial_pose.position.x = radius * math.cos(angle)
        req.initial_pose.position.y = radius * math.sin(angle)
        req.initial_pose.position.z = 0.0

        # random orientation
        yaw = random.uniform(-math.pi / 3, math.pi / 3)
        quat = quaternion_from_euler(0, 0, yaw)
        req.initial_pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
                f"Spawned {model_name} at x={req.initial_pose.position.x:.2f}, "
                f"y={req.initial_pose.position.y:.2f}, distance={radius:.2f}m, yaw={math.degrees(yaw):.1f}°"
            )
        else:
            self.get_logger().error(f"Failed to spawn {model_name}")

    def delete_model(self, model_name):
        req = DeleteEntity.Request()
        req.name = model_name

        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Deleted {model_name}")
        else:
            self.get_logger().error(f"Failed to delete {model_name}")

    def set_ground_truth_obstacle_name(self, obstacle_name):
        param = Parameter()
        param.name = "obstacle_name"
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=obstacle_name
        )

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(
                f"🔧 Set GroundTruthListener obstacle_name = {obstacle_name}"
            )
        else:
            self.get_logger().error(
                f"❌ Failed to set parameter on GroundTruthListener obstacle_name = {obstacle_name}"
            )

    def set_logger_obstacle_param(self, obstacle_name):
        param = Parameter()
        param.name = "obstacle_name"
        param.value = ParameterValue(
            type=ParameterType.PARAMETER_STRING, string_value=obstacle_name
        )

        req = SetParameters.Request()
        req.parameters = [param]

        # Assuming the data_logger node has the same naming convention as ground_truth_listener
        # Replace this with the actual node name if different
        if not hasattr(self, "logger_param_client"):
            self.logger_param_client = self.create_client(
                SetParameters, "/data_logger/set_parameters"
            )
            self.logger_param_client.wait_for_service()

        future = self.logger_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"🧾 Set DataLogger obstacle_name = {obstacle_name}")
        else:
            self.get_logger().error("❌ Failed to set parameter on DataLogger")


def main(args=None):
    rclpy.init(args=args)
    node = SceneSpawner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
