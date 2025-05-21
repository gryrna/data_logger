from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    scene_condition_arg = DeclareLaunchArgument(
        "scene_condition",
        default_value="default_scene",
        description="Scene condition name",
    )
    save_dir_arg = DeclareLaunchArgument(
        "save_dir", default_value="sensor_data", description="Directory to save data"
    )
    obstacle_name_arg = DeclareLaunchArgument(
        "obstacle_name", default_value="", description="Name of the obstacle (optional)"
    )

    return LaunchDescription(
        [
            scene_condition_arg,
            save_dir_arg,
            obstacle_name_arg,
            # 1. GroundTruthListener
            Node(
                package="data_logger",
                executable="ground_truth_listener",
                name="ground_truth_listener",
                output="screen",
                parameters=[
                    {"obstacle_name": LaunchConfiguration("obstacle_name")},
                    {"scene_condition": LaunchConfiguration("scene_condition")},
                ],
            ),
            # 2. DataLogger
            Node(
                package="data_logger",
                executable="data_logger_node",
                name="data_logger",
                output="screen",
                parameters=[
                    {"scene_condition": LaunchConfiguration("scene_condition")},
                    {"save_dir": LaunchConfiguration("save_dir")},
                    {"obstacle_name": LaunchConfiguration("obstacle_name")},
                ],
            ),
            # 3. Delayed SceneSpawner
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="data_logger",
                        executable="scene_spawner",
                        name="scene_spawner",
                        output="screen",
                        parameters=[
                            {"obstacle_name": LaunchConfiguration("obstacle_name")}
                        ],
                    )
                ],
            ),
        ]
    )
