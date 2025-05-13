import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
import numpy as np
from tf_transformations import euler_from_quaternion

class GroundTruthListener(Node):
    def __init__(self):
        super().__init__('ground_truth_listener')
        self.robot_name = 'mobile_bot'
        self.obstacle_name = 'red_ball'

        self.model_states_subscriber = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

    def model_states_callback(self, msg):
        try:
            robot_idx = msg.name.index(self.robot_name)
            obstacle_idx = msg.name.index(self.obstacle_name)

            # Log model names
            self.get_logger().info(f"Model list: {msg.name}")
            self.get_logger().info(f"Robot index: {robot_idx}, Obstacle index: {obstacle_idx}")

            # Robot position and orientation
            robot_pose = msg.pose[robot_idx]
            robot_pos = np.array([robot_pose.position.x, robot_pose.position.y])
            robot_orientation = robot_pose.orientation

            self.get_logger().info(f"Robot position: {robot_pos}")
            self.get_logger().info(
                f"Robot orientation (quaternion): x={robot_orientation.x:.6f}, y={robot_orientation.y:.6f}, "
                f"z={robot_orientation.z:.6f}, w={robot_orientation.w:.6f}"
            )

            # Convert quaternion to Euler angles
            quaternion = (
                robot_orientation.x,
                robot_orientation.y,
                robot_orientation.z,
                robot_orientation.w
            )
            roll, pitch, yaw = euler_from_quaternion(quaternion)
            self.get_logger().info(
                f"Robot orientation (Euler): roll={np.degrees(roll):.2f}°, pitch={np.degrees(pitch):.2f}°, yaw={np.degrees(yaw):.2f}°"
            )

            # Robot's forward-facing unit vector
            robot_forward = np.array([np.cos(yaw), np.sin(yaw)])
            self.get_logger().info(f"Robot forward vector: {robot_forward}")

            # Obstacle position
            obstacle_pose = msg.pose[obstacle_idx]
            obstacle_pos = np.array([obstacle_pose.position.x, obstacle_pose.position.y])
            self.get_logger().info(f"Obstacle position: {obstacle_pos}")

            # Vector to obstacle
            vector_to_obstacle = obstacle_pos - robot_pos
            self.get_logger().info(f"Vector to obstacle: {vector_to_obstacle}")

            # Normalize and compute angle
            distance = np.linalg.norm(vector_to_obstacle)
            if distance == 0:
                self.get_logger().warn("Robot and obstacle are at the same position. Skipping.")
                return
            direction_to_obstacle = vector_to_obstacle / distance
            self.get_logger().info(f"Normalized direction to obstacle: {direction_to_obstacle}")
            self.get_logger().info(f"Distance to obstacle: {distance:.3f} meters")

            # Compute angle
            dot_product = np.dot(robot_forward, direction_to_obstacle)
            angle_deg = np.degrees(np.arccos(np.clip(dot_product, -1.0, 1.0)))
            self.get_logger().info(f"Angle between robot forward and obstacle: {angle_deg:.1f}°")

            # New range-based categorization
            if angle_deg < 45:
                self.get_logger().info(
                    f"✅ Obstacle IN FRONT (Angle = {angle_deg:.1f}°)"
                )
            elif angle_deg < 120:
                self.get_logger().info(
                    f"🔄 Obstacle ON SIDE (Angle = {angle_deg:.1f}°)"
                )
            else:
                self.get_logger().info(
                    f"⛔ Obstacle BEHIND (Angle = {angle_deg:.1f}°)"
                )

        except ValueError:
            self.get_logger().warn("Robot or obstacle not found in model states")

def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
