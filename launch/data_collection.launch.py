from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # 1. Launch GroundTruthListener node first
        Node(
            package='data_logger',
            executable='ground_truth_listener',
            name='ground_truth_listener',
            output='screen',
            parameters=[{'obstacle_name': ''}]
        ),

        # 2. Launch DataLogger node
        Node(
            package='data_logger',
            executable='data_logger_node',
            name='data_logger',
            output='screen',
        ),

        # 3. Delay SceneSpawner slightly to ensure GroundTruthListener is up
        TimerAction(
            period=2.0,  # Wait 2 seconds before launching scene_spawner
            actions=[
                Node(
                    package='data_logger',  # Replace with actual package name
                    executable='scene_spawner',
                    name='scene_spawner',
                    output='screen',
                )
            ]
        )
    ])
