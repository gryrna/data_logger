import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_msgs.msg import Float32, Bool
from threading import Event
import time
import os
import random
import math
from geometry_msgs.msg import Quaternion
from tf_transformations import quaternion_from_euler


class SceneSpawner(Node):
    def __init__(self):
        super().__init__('scene_spawner')
        self.obstacles = [
            'black_ball', 'car_wheel', 'cinder_block',
            'construction_barrel', 'construction_cone', 'green_ball', 'red_ball'
        ]
        self.model_path_base = os.path.expanduser('~/.gazebo/models')

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.trigger_pub = self.create_publisher(Float32, '/trigger_sample', 10)
        self.obstacle_ready_event = Event()
        self.obstacle_ready_event.clear()
        self.obstacle_ready_sub = self.create_subscription(Bool, '/obstacle_ready', self.ready_callback, 10)

        # Create parameter client for GroundTruthListener
        self.param_client = self.create_client(SetParameters, '/ground_truth_listener/set_parameters')

        self.run_sequence()

    def ready_callback(self, msg):
        if msg.data:
            self.get_logger().info("🟢 Ground truth ready signal received.")
            self.obstacle_ready_event.set()
    
    
    def wait_for_services(self):
        self.get_logger().info('Waiting for /spawn_entity and /delete_entity services and /ground_truth_listener/set_parameters service...')
        self.spawn_client.wait_for_service()
        self.delete_client.wait_for_service()
        self.param_client.wait_for_service()
        self.get_logger().info('Services are available.')

    def run_sequence(self):
        self.wait_for_services()

        for i, obstacle in enumerate(self.obstacles):
            self.get_logger().info(f'\nSpawning {obstacle}...')

            # Delete previous obstacle
            if i > 0:
                self.delete_model(self.obstacles[i - 1])

            # Spawn new obstacle
            self.spawn_model(obstacle)

            # Set the obstacle name in GroundTruthListener
            self.set_ground_truth_obstacle_name(obstacle)
            
            self.get_logger().info('Waiting for scene to stabilize...')
            time.sleep(10)  # Wait for sensors to settle

            self.get_logger().info('Triggering data collection...')
            self.trigger_pub.publish(Float32(data=1.0))

            time.sleep(2)  # Wait for data logging to complete

        # Delete the last model
        self.delete_model(self.obstacles[-1])
        self.get_logger().info('All samples collected. Process complete.')

    def spawn_model(self, model_name):
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = open(f'{self.model_path_base}/{model_name}/model.sdf', 'r').read()
        req.robot_namespace = model_name
        req.reference_frame = 'world'
        #random position
        # Position: Inside FOV cone of camera and LiDAR
        angle = random.uniform(-math.pi / 6, math.pi / 6)  # ±30°
        radius = random.uniform(0.8, 3.0)  # Distance from robot
        req.initial_pose.position.x = radius * math.cos(angle)
        req.initial_pose.position.y = radius * math.sin(angle)
        req.initial_pose.position.z = 0.0

        #random orientation
        yaw = random.uniform(0, math.pi)
        quat = quaternion_from_euler(0, 0, yaw)
        req.initial_pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(
            f'Spawned {model_name} at x={req.initial_pose.position.x:.2f}, '
            f'y={req.initial_pose.position.y:.2f}, yaw={math.degrees(yaw):.1f}°'
            )
        else:
            self.get_logger().error(f'Failed to spawn {model_name}')

    def delete_model(self, model_name):
        req = DeleteEntity.Request()
        req.name = model_name

        future = self.delete_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Deleted {model_name}')
        else:
            self.get_logger().error(f'Failed to delete {model_name}')

    def set_ground_truth_obstacle_name(self, obstacle_name):
        param = Parameter()
        param.name = 'obstacle_name'
        param.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=obstacle_name)

        req = SetParameters.Request()
        req.parameters = [param]

        future = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'🔧 Set GroundTruthListener obstacle_name = {obstacle_name}')
        else:
            self.get_logger().error(f'❌ Failed to set parameter on GroundTruthListener obstacle_name = {obstacle_name}')



def main(args=None):
    rclpy.init(args=args)
    node = SceneSpawner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
