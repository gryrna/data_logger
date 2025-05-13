import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import json

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.bridge = CvBridge()

        self.declare_parameter('save_dir', 'sensor_data')
        self.save_dir = self.get_parameter('save_dir').get_parameter_value().string_value
        os.makedirs(self.save_dir, exist_ok=True)

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        self.latest_image = None
        self.latest_lidar = None
        self.sample_index = 0

        self.timer = self.create_timer(2.0, self.save_sample)

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def lidar_callback(self, msg):
        self.latest_lidar = np.array(msg.ranges)

    def save_sample(self):
        if self.latest_image is None or self.latest_lidar is None:
            self.get_logger().info("Waiting for sensor data...")
            return

        img_path = os.path.join(self.save_dir, f'image_{self.sample_index}.jpg')
        lidar_path = os.path.join(self.save_dir, f'lidar_{self.sample_index}.npy')
        label_path = os.path.join(self.save_dir, f'ground_truth_{self.sample_index}.json')

        cv2.imwrite(img_path, self.latest_image)
        np.save(lidar_path, self.latest_lidar)

        # TODO: Replace with real label if possible
        label = {
            "distance": float(np.min(self.latest_lidar))  # crude ground-truth
        }
        with open(label_path, 'w') as f:
            json.dump(label, f)

        self.get_logger().info(f"Saved sample {self.sample_index}")
        self.sample_index += 1

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
