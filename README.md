 # Data Logger

A ROS 2 package for logging sensor data from a mobile robot, including camera images, LiDAR scans, and ground truth information.

## Overview

The data_logger package provides tools for collecting and storing sensor data from a mobile robot. This data can be used for:
- Creating datasets for machine learning algorithms
- Verifying sensor accuracy
- Offline analysis of robot performance

## Features

- Captures and stores camera images in JPEG format
- Records LiDAR scans as NumPy arrays
- Tracks ground truth position information for the robot and obstacles
- Calculates relative positions and orientations between the robot and obstacles
- Configurable storage directory

## Nodes

### data_logger_node

Subscribes to camera and LiDAR sensor topics, saves data at regular intervals.

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image)
- `/scan` (sensor_msgs/LaserScan)

**Parameters:**
- `save_dir` (string, default: 'sensor_data'): Directory where sensor data will be stored

### ground_truth_listener

Listens to Gazebo model states to determine ground truth positions of the robot and obstacles.

**Subscribed Topics:**
- `/model_states` (gazebo_msgs/ModelStates)

**Tracked Models:**
- Robot model: 'mobile_bot'
- Obstacle model: 'red_ball'

## Installation

```bash
# Clone this package to your ROS 2 workspace src directory
cd ~/your_ros2_ws/src
git clone <repository_url> data_logger

# Build the package
cd ~/your_ros2_ws
colcon build --packages-select data_logger

# Source the workspace
source ~/your_ros2_ws/install/setup.bash
```

## Usage

### Run the data logger node:

```bash
ros2 run data_logger data_logger_node
```

### Run with custom parameters:

```bash
ros2 run data_logger data_logger_node --ros-args -p save_dir:=/path/to/custom/directory
```

### Run the ground truth listener:

```bash
ros2 run data_logger ground_truth_listener
```

## Data Format

Data is stored in the specified directory with the following naming convention:
- `image_{index}.jpg`: Camera images
- `lidar_{index}.npy`: LiDAR scan data as NumPy arrays
- `ground_truth_{index}.json`: Position information

## Dependencies

- rclpy
- sensor_msgs
- cv_bridge
- std_msgs
- gazebo_msgs
- numpy
- tf_transformations

## License

MIT

## Author

Maintained by gryrna (gumeshrana1@gmail.com)