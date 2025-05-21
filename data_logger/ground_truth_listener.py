import json

import numpy as np
import rclpy
from gazebo_msgs.msg import ModelStates
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from tf_transformations import euler_from_quaternion


class GroundTruthListener(Node):
    def __init__(self):
        super().__init__("ground_truth_listener")
        self.robot_name = "mobile_bot"

        # Publish obstacle ready flag
        self.ready_pub = self.create_publisher(Bool, "/obstacle_ready", 10)
        self.last_obstacle_name = None

        # ✅ NEW: Publish ground truth ready flag
        self.gt_ready_pub = self.create_publisher(Bool, "/ground_truth_ready", 10)

        # ✅ NEW: Publish ground truth data in JSON format
        self.gt_data_pub = self.create_publisher(String, "/ground_truth_data", 10)

        # Declare a dynamic obstacle_name parameter
        self.declare_parameter("obstacle_name")
        self.obstacle_name = (
            self.get_parameter("obstacle_name").get_parameter_value().string_value
        )

        # ✅ Add scene_condition parameter
        self.declare_parameter("scene_condition", "unknown")
        self.scene_condition = (
            self.get_parameter("scene_condition").get_parameter_value().string_value
        )

        # trigger flag
        self.trigger_active = False

        self.model_states_subscriber = self.create_subscription(
            ModelStates, "/model_states", self.model_states_callback, 10
        )

        self.trigger_subscriber = self.create_subscription(
            Float32, "/trigger_sample", self.trigger_callback, 10
        )

        self.distance_publisher = self.create_publisher(
            Float32, "/ground_truth/distance", 10
        )
        self.angle_publisher = self.create_publisher(Float32, "/ground_truth/angle", 10)
        self.obstacle_name_publisher = self.create_publisher(
            String, "/obstacle_name", 10
        )

        # Monitor parameter changes
        self.add_on_set_parameters_callback(self.parameter_callback)

        self.current_obstacle = ""
        self.create_subscription(
            String, "/current_obstacle", self.obstacle_callback, 10
        )

    def parameter_callback(self, params):
        for param in params:
            if (
                param.name == "obstacle_name"
                and param.type_ == rclpy.Parameter.Type.STRING
            ):
                self.obstacle_name = param.value
                self.get_logger().info(
                    f"🔄 Obstacle name updated to: {self.obstacle_name}"
                )
                self.last_obstacle_name = None  # reset sync status
        return SetParametersResult(successful=True)

    def trigger_callback(self, msg):
        self.get_logger().info("🔔 Trigger flag activated")
        self.trigger_active = True

    def model_states_callback(self, msg):
        try:
            if not self.obstacle_name:
                return

            # Phase 1: Obstacle confirmation
            if self.last_obstacle_name != self.obstacle_name and self.obstacle_name in msg.name:
                self.last_obstacle_name = self.obstacle_name
                self.get_logger().info(
                    f"🟢 Obstacle '{self.obstacle_name}' visible in model_states. Sending ready signal."
                )
                self.ready_pub.publish(Bool(data=True))
                return  # 🔁 Don't run GT logic in same cycle

            # Phase 2: Ground truth calculation (after trigger)
            if not self.trigger_active:
                return

            robot_idx = msg.name.index(self.robot_name)
            robot_pose = msg.pose[robot_idx]
            robot_pos = np.array([robot_pose.position.x, robot_pose.position.y])

            quaternion = (
                robot_pose.orientation.x,
                robot_pose.orientation.y,
                robot_pose.orientation.z,
                robot_pose.orientation.w,
            )
            _, _, yaw = euler_from_quaternion(quaternion)
            robot_forward = np.array([np.cos(yaw), np.sin(yaw)])

            # ✅ Check for Transparent_sheet specifically
            if "Transparent_sheet" in msg.name:
                transparent_idx = msg.name.index("Transparent_sheet")
                transparent_pose = msg.pose[transparent_idx]
                transparent_pos = np.array([transparent_pose.position.x, transparent_pose.position.y])

                vector_to_transparent = transparent_pos - robot_pos
                distance = np.linalg.norm(vector_to_transparent)

                if distance > 0:
                    direction = vector_to_transparent / distance
                    angle_deg = np.degrees(np.arccos(np.clip(np.dot(robot_forward, direction), -1.0, 1.0)))

                    if angle_deg < 45:
                        self.distance_publisher.publish(Float32(data=float(distance)))
                        self.angle_publisher.publish(Float32(data=float(angle_deg)))

                        ground_truth_data = {
                            "distance": float(distance),
                            "angle": float(angle_deg),
                            "scene_condition": self.scene_condition,
                            "obstacle_name": "Transparent_sheet",
                        }
                        self.gt_data_pub.publish(String(data=json.dumps(ground_truth_data)))
                        self.gt_ready_pub.publish(Bool(data=True))

                        self.get_logger().info(
                            f"✅ Transparent GT — Distance: {distance:.3f} m, Angle: {angle_deg:.1f}°"
                        )

            self.trigger_active = False

        except ValueError:
            return


    def obstacle_callback(self, msg):
        self.current_obstacle = msg.data


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
