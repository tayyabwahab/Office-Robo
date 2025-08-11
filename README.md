# Office-Robo: Autonomous Robot Navigation System

## Overview

Office-Robo is a comprehensive ROS (Robot Operating System) based autonomous robot navigation project that implements advanced pathfinding and obstacle avoidance algorithms. The project features a simulated robot that can navigate through complex environments using a combination of go-to-point navigation and wall-following behaviors.

## Features

### 🤖 Core Navigation Capabilities
- **Autonomous Navigation**: Robot can navigate to specified target points autonomously
- **Obstacle Avoidance**: Advanced wall-following algorithm for obstacle avoidance
- **Hybrid Navigation**: Combines direct path navigation with wall-following when obstacles are detected
- **Real-time Path Planning**: Dynamic path adjustment based on sensor data

### 🎯 Key Algorithms
- **Go-to-Point Algorithm**: Direct navigation to target coordinates
- **Wall-Following Algorithm**: Intelligent obstacle avoidance using laser scan data

### 🛠️ Technical Features
- **Laser Scanner Integration**: 720-degree laser scan for environment perception
- **Odometry Tracking**: Real-time position and orientation tracking
- **Service-based Architecture**: Modular design with ROS services
- **Gazebo Simulation**: Full 3D simulation environment
- **RViz Visualization**: Real-time visualization of robot state and sensor data

## Project Structure

```
Office-Robo/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package dependencies and metadata
├── requirements.txt            # Python dependencies
├── README.md                   # This file
├── launch/                     # ROS launch files
│   ├── AR_PROJECT_algo.launch  # Main algorithm launcher
│   ├── AR_PROJECT_map.launch   # Mapping configuration
│   ├── AR_PROJECT_world.launch # World simulation launcher
│   ├── gazeboCar.launch        # Gazebo robot spawner
│   ├── gmapping.launch         # SLAM mapping
│   └── rviz.launch            # Visualization launcher
├── scripts/                    # Python navigation algorithms
│   ├── follow_wall.py         # Wall-following algorithm
│   ├── go_to_point.py         # Point-to-point navigation
│   └── serviceAlgo.py         # Main service orchestrator
├── urdf/                      # Robot model definition
│   ├── AR_PROJECT.xacro       # Main robot model
│   ├── AR_PROJECT.gazebo      # Gazebo-specific properties
│   ├── macros.xacro           # Reusable robot components
│   └── materials.xacro        # Visual materials
├── worlds/                    # Simulation environments
│   └── AR_PROJECT_world.world # Main simulation world
└── maps/                      # Navigation maps
    ├── AR_PROJECT_map.pgm     # Occupancy grid map
    └── AR_PROJECT_map.yaml    # Map metadata
```

## Robot Architecture


## Navigation Algorithms

### 1. Go-to-Point Algorithm (`go_to_point.py`)
**Purpose**: Direct navigation to target coordinates

### 2. Wall-Following Algorithm (`follow_wall.py`)
**Purpose**: Obstacle avoidance and wall following

**States**:
- **State 0**: Find the wall
- **State 1**: Turn left
- **State 2**: Follow the wall
- **State 3**: Turn right

**Behavior Cases**:
- Case 1: No obstacles detected
- Case 2: Front obstacle detected
- Case 3: Front-right obstacle
- Case 4: Front-left obstacle
- Case 5: Front and front-right obstacles
- Case 6: Front and front-left obstacles
- Case 7: Front, front-left, and front-right obstacles
- Case 8: Front-left and front-right obstacles

### 3. Service Algorithm (`serviceAlgo.py`)
**Purpose**: Orchestrates between navigation modes

**Logic**:
- Starts with go-to-point navigation
- Switches to wall-following when obstacles are detected
- Returns to go-to-point when path is clear
- Considers angle and distance thresholds for mode switching

## Installation and Setup

### Prerequisites
- ROS Noetic (Ubuntu 20.04) or ROS Melodic (Ubuntu 18.04)
- Gazebo simulator
- RViz for visualization
- Python 2.7 or Python 3 (depending on ROS version)

### Dependencies

#### ROS Dependencies (package.xml)
```xml
<depend>rospy</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf</depend>
<depend>gazebo_msgs</depend>
<depend>std_srvs</depend>
```


**Install Python dependencies:**
```bash
pip install -r requirements.txt
```

### Installation Steps
1. **Clone the repository**:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/tayyabwahab/Office-Robo.git
   ```

2. **Build the package**:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

3. **Install dependencies**:
   ```bash
   # Install ROS dependencies
   rosdep install --from-paths src --ignore-src -r -y
   
   # Install Python dependencies
   pip install -r requirements.txt
   ```

## Usage

### Basic Navigation
1. **Launch the simulation**:
   ```bash
   roslaunch AR_PROJECT AR_PROJECT_world.launch
   ```

2. **Launch the navigation algorithm** (specify target coordinates):
   ```bash
   roslaunch AR_PROJECT AR_PROJECT_algo.launch x:=5.0 y:=3.0
   ```

3. **Visualize with RViz**:
   ```bash
   roslaunch AR_PROJECT rviz.launch
   ```

### Advanced Usage

#### Custom Target Coordinates
```bash
# Navigate to specific coordinates
roslaunch AR_PROJECT AR_PROJECT_algo.launch x:=10.0 y:=5.0
```

#### Mapping and Navigation
```bash
# Create a map of the environment
roslaunch AR_PROJECT gmapping.launch

# Navigate using the created map
roslaunch AR_PROJECT AR_PROJECT_map.launch
```

#### Visualization Options
```bash
# Launch with RViz for real-time visualization
roslaunch AR_PROJECT rviz.launch

# Launch Gazebo GUI for 3D simulation view
roslaunch AR_PROJECT AR_PROJECT_world.launch gui:=true
```

## ROS Topics and Services

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist): Robot velocity commands
- `/odom` (nav_msgs/Odometry): Robot odometry data
- `/AR_PROJECT/laser/scan` (sensor_msgs/LaserScan): Laser scanner data

### Subscribed Topics
- `/AR_PROJECT/laser/scan` (sensor_msgs/LaserScan): Laser scanner data
- `/odom` (nav_msgs/Odometry): Robot odometry data

### Services
- `/go_to_point_switch` (std_srvs/SetBool): Enable/disable go-to-point navigation
- `/wall_follower_switch` (std_srvs/SetBool): Enable/disable wall-following
- `/gazebo/set_model_state` (gazebo_msgs/SetModelState): Set robot position in simulation

### Parameters
- `des_pos_x`: Target X coordinate
- `des_pos_y`: Target Y coordinate

## Simulation Environment

### World Description
The simulation world includes:
- **Ground Plane**: 100m × 100m flat surface
- **Obstacles**: Various barriers and walls for testing navigation
- **Lighting**: Directional lighting for realistic simulation
- **Physics**: ODE physics engine with realistic parameters

### Robot Model
- **Differential Drive**: Two-wheel drive system
- **Collision Detection**: Proper collision geometry for all components
- **Visual Representation**: Blue chassis with white laser scanner

## Algorithm Details

### Navigation State Machine
```
[Start] → [Go-to-Point] → [Check Obstacles] → [Wall-Following] → [Return to Go-to-Point]
```

### Obstacle Detection Logic
The robot divides its environment into five regions and makes navigation decisions based on:
- **Distance thresholds**: 1.0m for general obstacles, 1.5m for safe navigation
- **Angle considerations**: 30° and 90° thresholds for path planning
- **Region combinations**: Different behaviors for various obstacle configurations

### Path Planning Strategy
1. **Direct Path**: Attempts straight-line navigation to target
2. **Obstacle Detection**: Monitors laser scan data for obstacles
3. **Wall Following**: Engages when obstacles block direct path
4. **Path Recovery**: Returns to direct navigation when path clears


## Troubleshooting

### Common Issues

1. **Robot not moving**:
   - Check if services are properly initialized
   - Verify target coordinates are set
   - Ensure laser scanner is publishing data

2. **Navigation getting stuck**:
   - Check obstacle detection thresholds
   - Verify state machine transitions
   - Monitor laser scan data quality

3. **Simulation not loading**:
   - Ensure Gazebo is properly installed
   - Check world file path
   - Verify URDF model loading

### Debug Commands
```bash
# Check robot position
rostopic echo /odom

# Monitor laser data
rostopic echo /AR_PROJECT/laser/scan

# Check service status
rosservice list

# View robot model
rosrun urdf_tutorial view_robot.py
```

## License

This project is licensed under the TODO license - see the package.xml file for details.


## Acknowledgments

- ROS community for the excellent framework
- Gazebo team for the simulation environment
- Contributors to the navigation algorithms
