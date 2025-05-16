import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import json
from std_msgs.msg import Float32, Bool  # ✅ Added Bool


class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.bridge = CvBridge()

        # Parameters
        self.declare_parameter('save_dir', 'sensor_data')
        self.declare_parameter('scene_condition', 'unknown')  # e.g., "foggy", "dark_room"
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        self.scene_condition = self.get_parameter('scene_condition').get_parameter_value().string_value

        os.makedirs(self.save_dir, exist_ok=True)

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.ground_truth_distance_sub = self.create_subscription(
            Float32, '/ground_truth/distance', self.ground_truth_distance_callback, 10)
        self.ground_truth_angle_sub = self.create_subscription(
            Float32, '/ground_truth/angle', self.ground_truth_angle_callback, 10)
        self.trigger_sub = self.create_subscription(
            Float32, '/trigger_sample', self.trigger_callback, 10)

        # ✅ New subscriber to GT ready signal
        self.gt_ready_sub = self.create_subscription(
            Bool, '/ground_truth_ready', self.gt_ready_callback, 10)

        self.latest_image = None
        self.latest_lidar = None
        self.sample_index = 0
        self.ground_truth_distance = None
        self.ground_truth_angle = None

        # ✅ Flag to track readiness
        self.waiting_for_gt = False

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def lidar_callback(self, msg):
        self.latest_lidar = np.array(msg.ranges)

    def ground_truth_distance_callback(self, msg):
        self.ground_truth_distance = float(msg.data)

    def ground_truth_angle_callback(self, msg):
        self.ground_truth_angle = float(msg.data)

    def trigger_callback(self, msg):
        # ✅ Begin waiting for GT confirmation before saving
        self.waiting_for_gt = True
        self.get_logger().info("📸 Trigger received — waiting for ground truth confirmation...")

    def gt_ready_callback(self, msg):
        if self.waiting_for_gt and msg.data:
            self.get_logger().info("📍 Ground truth confirmed — saving sample")
            self.save_sample()
            self.waiting_for_gt = False

    def save_sample(self):
        if self.latest_image is None or self.latest_lidar is None:
            self.get_logger().info("Waiting for sensor data...")
            return

        img_path = os.path.join(self.save_dir, f'image_{self.sample_index}.jpg')
        lidar_path = os.path.join(self.save_dir, f'lidar_{self.sample_index}.npy')
        label_path = os.path.join(self.save_dir, f'ground_truth_{self.sample_index}.json')

        cv2.imwrite(img_path, self.latest_image)
        np.save(lidar_path, self.latest_lidar)

        label = {
            "distance": self.ground_truth_distance if self.ground_truth_distance is not None else -1.0,
            "angle": self.ground_truth_angle if self.ground_truth_angle is not None else -1.0
        }

        with open(label_path, 'w') as f:
            json.dump(label, f)

        self.get_logger().info(f"✅ Saved sample {self.sample_index} [{self.scene_condition}]")
        self.sample_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
