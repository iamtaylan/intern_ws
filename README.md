# TurtleBot3 Basic ROS Application

A foundational ROS application for TurtleBot3 that demonstrates core robotics concepts including odometry tracking, velocity control, and autonomous navigation primitives. This project provides both C++ and Python implementations for learning and prototyping robot motion control.

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Usage](#usage)
  - [Configuration](#configuration)
  - [Running the C++ Node](#running-the-c-node)
  - [Running the Python Node](#running-the-python-node)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
- [Contributing](#contributing)
- [Support](#support)
- [License](#license)

## Features

- **Dual Implementation**: Both C++ and Python versions for flexibility and learning
- **Autonomous Motion Control**: Sequential task execution with forward motion and rotation
- **Odometry Integration**: Real-time position and orientation tracking
- **Configurable Parameters**: Easy adjustment of robot speeds and motion duration via ROS parameters
- **Error Handling**: Robust angle normalization and motion control logic
- **ROS Native**: Full integration with ROS ecosystem (subscribers, publishers, launch files)

## Requirements

- **ROS Distribution**: ROS Melodic or newer (Noetic recommended)
- **Operating System**: Ubuntu 18.04 LTS or newer (for Melodic/Noetic)
- **Python Version**: Python 3.6+ (if using Python implementation)
- **ROS Packages**:
  - `roscpp` and `rospy`
  - `geometry_msgs`
  - `nav_msgs`
  - `tf2_geometry_msgs`
  - `turtlebot3` (for actual hardware operation)

## Installation

### 1. Set Up ROS Workspace

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Clone the Repository

```bash
cd ~/catkin_ws/src
git clone https://github.com/iamtaylan/intern_ws.git
cd ..
```

### 3. Install Dependencies

Install ROS package dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

If running on actual TurtleBot3 hardware, also install:

```bash
sudo apt-get install ros-$(rosversion -d)-turtlebot3 \
                    ros-$(rosversion -d)-turtlebot3-msgs \
                    ros-$(rosversion -d)-turtlebot3-simulations
```

### 4. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Verify the build was successful:

```bash
rospack find intern_ros_case
```

## Quick Start

### Running with Simulator

```bash
# Terminal 1: Start Gazebo simulator with TurtleBot3
export TURTLEBOT3_MODEL=burger  # or waffle
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Terminal 2: Run the movement node
source ~/catkin_ws/devel/setup.bash
roslaunch intern_ros_case bringup.launch
```

### Running on Real Hardware

```bash
# On TurtleBot3 (robot)
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_core.launch

# On remote PC
export ROS_MASTER_URI=http://<robot_ip>:11311
export ROS_HOSTNAME=<pc_ip>
source ~/catkin_ws/devel/setup.bash
roslaunch intern_ros_case bringup.launch
```

## Usage

### Configuration

Robot motion parameters are defined in `config/params.yaml`:

```yaml
linear_speed: 0.2       # Forward velocity (m/s)
angular_speed: 0.3      # Rotation velocity (rad/s)
move_duration: 5.0      # Forward motion duration (seconds)
```

Modify these parameters to adjust robot behavior:

```bash
# Override parameters at runtime
roslaunch intern_ros_case bringup.launch \
  linear_speed:=0.3 \
  angular_speed:=0.5 \
  move_duration:=10.0
```

### Running the C++ Node

The C++ implementation offers better performance for production scenarios:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun intern_ros_case movement_node
```

**What it does:**
1. Moves forward at specified speed for `move_duration` seconds
2. Stops and pauses for 1 second
3. Rotates 90 degrees using angular error control
4. Continues monitoring odometry data via ROS topics

### Running the Python Node

The Python implementation is ideal for learning and rapid prototyping:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun intern_ros_case movement_node.py
```

To use the Python node instead of C++, modify `launch/bringup.launch`:

```xml
<!-- Change from: -->
<node pkg="intern_ros_case" type="movement_node" ...>

<!-- To: -->
<node pkg="intern_ros_case" type="movement_node.py" ...>
```

### Monitoring Robot State

Watch the robot's position and orientation in real-time:

```bash
# Terminal 1: Run the movement node
roslaunch intern_ros_case bringup.launch

# Terminal 2: Echo odometry data
rostopic echo /odom
```

## Project Structure

```
intern_ros_case/
├── src/
│   ├── movement_node.cpp          # C++ implementation
│   └── scripts/
│       └── movement_node.py        # Python implementation
├── include/
│   └── intern_ros_case/
│       └── movement_node.hpp       # C++ header file
├── launch/
│   └── bringup.launch              # ROS launch file
├── config/
│   └── params.yaml                 # Robot configuration parameters
├── CMakeLists.txt                  # CMake build configuration
└── package.xml                     # ROS package metadata
```

## Architecture

### Movement Controller

Both C++ and Python implementations follow the same control architecture:

```
┌─────────────────────────────────────────┐
│   Movement Controller Node              │
├─────────────────────────────────────────┤
│ Publishers:                             │
│  • /cmd_vel (geometry_msgs/Twist)      │
│                                         │
│ Subscribers:                            │
│  • /odom (nav_msgs/Odometry)           │
├─────────────────────────────────────────┤
│ Tasks:                                  │
│  1. Forward motion (time-based)        │
│  2. Stop and pause                     │
│  3. 90° rotation (angle-based)         │
│  4. Continuous odometry monitoring     │
└─────────────────────────────────────────┘
```

### Key Classes

**MovementController** (main control class)
- `odom_callback()`: Processes odometry updates and extracts position/orientation
- `normalize_angle()`: Ensures angle continuity (-π to +π)
- `rotate_90_degrees()`: Performs precise 90° rotation using error control
- `run_task()`: Executes the motion sequence

## Contributing

We welcome contributions! Please follow these guidelines:

1. **Fork the repository** and create a feature branch:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** following ROS coding standards:
   - Use consistent naming conventions (snake_case for functions/variables)
   - Add comments for non-obvious logic
   - Test with both C++ and Python implementations

3. **Commit with descriptive messages**:
   ```bash
   git commit -m "Add: feature description"
   ```

4. **Push and open a Pull Request** with details about your changes


## Support

### Getting Help

- **ROS Documentation**: [wiki.ros.org](http://wiki.ros.org)
- **TurtleBot3 Wiki**: [wiki.ros.org/turtlebot3](http://wiki.ros.org/turtlebot3)
- **Issue Tracker**: Open an issue on [GitHub Issues](https://github.com/iamtaylan/TurtleBot3_basic_ROS_application/issues)

### Troubleshooting

**Node fails to connect to robot odometry:**
- Verify `/odom` topic exists: `rostopic list | grep odom`
- Check topic message type: `rostopic info /odom`
- Ensure TurtleBot3 base drivers are running

**Robot not responding to velocity commands:**
- Verify `/cmd_vel` topic is being published: `rostopic echo /cmd_vel`
- Confirm robot is in teleop mode or accepting command_velocity
- Check robot battery level

**Build failures:**
- Ensure all dependencies are installed: `rosdep install --from-paths src --ignore-src`
- Clean and rebuild: `catkin_make clean && catkin_make`

## License

This project is released under the [MIT License](LICENSE). See the LICENSE file for details.

---

**Maintainer**: [taylan](https://github.com/iamtaylan)

**Repository**: [intern_ws](https://github.com/iamtaylan/intern_ws)
