# Quickstart Guide: Digital Twin using Gazebo and Unity

## Overview
This guide will help you set up a basic digital twin simulation combining Gazebo physics simulation with Unity visualization. By the end of this guide, you'll have a working environment to explore digital twin concepts for robotics.

## Prerequisites
- Ubuntu 20.04+ (for Gazebo) or Windows/Mac (for Unity)
- ROS 2 (recommended: Humble Hawksbill)
- Unity Hub with Unity 2022.3 LTS
- Git
- Python 3.8+

## Setup Steps

### 1. Clone the Repository
```bash
git clone https://github.com/your-org/humanoid-robotics-textbook.git
cd humanoid-robotics-textbook
```

### 2. Install Gazebo (for Ubuntu)
```bash
# Add ROS repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install Gazebo
sudo apt update
sudo apt install gazebo libgazebo-dev
```

### 3. Install ROS 2 Packages
```bash
# Install ROS 2 Humble dependencies
sudo apt update
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-rosbridge-suite
```

### 4. Set up Unity Environment
1. Download and install Unity Hub from unity3d.com
2. Install Unity 2022.3 LTS through Unity Hub
3. Install the Unity Robotics Hub package from the Unity Asset Store
4. Install the ROS-TCP-Connector package from the Unity Asset Store

### 5. Configure the Docusaurus Site
```bash
cd website
npm install
```

### 6. Build and Run the Documentation Site
```bash
cd website
npm run build
npm run serve
```

## Running Your First Digital Twin Simulation

### Gazebo Simulation
1. Source ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

2. Launch a basic robot simulation:
```bash
# Navigate to the examples directory
cd examples/digital-twin/gazebo-simulations

# Launch the simulation
ros2 launch basic_robot.launch.py
```

### Unity Visualization
1. Open Unity Hub and create a new 3D project
2. Import the ROS-TCP-Connector package
3. Open the "DigitalTwinVisualization" scene
4. Configure the ROS connection settings to match your Gazebo instance

### Connecting Gazebo and Unity
1. Start the ROS bridge:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

2. In Unity, configure the ROS connection:
   - Set the ROS IP to your machine's IP
   - Use the default port (9090)

## Educational Content Structure

### Module 2: Digital Twin Simulation
The digital twin module is organized as follows:

```
website/docs/module-2-digital-twin/
├── intro.md                    # Module introduction
├── chapter-1-gazebo/           # Physics Simulation and Environment Building
│   └── index.md                # Complete chapter content
├── chapter-2-unity/            # High-Fidelity Rendering and Human-Robot Interaction
│   └── index.md                # Complete chapter content
└── chapter-3-sensors/          # Sensor Simulation and Integration
    └── index.md                # Complete chapter content
```

### Creating New Content
1. Create a new module intro file:
```bash
# In website/docs/ create your module directory and intro file
mkdir -p website/docs/module-2-digital-twin
touch website/docs/module-2-digital-twin/intro.md
```

2. Create chapter directories and index files:
```bash
mkdir -p website/docs/module-2-digital-twin/chapter-1-gazebo
touch website/docs/module-2-digital-twin/chapter-1-gazebo/index.md

mkdir -p website/docs/module-2-digital-twin/chapter-2-unity
touch website/docs/module-2-digital-twin/chapter-2-unity/index.md

mkdir -p website/docs/module-2-digital-twin/chapter-3-sensors
touch website/docs/module-2-digital-twin/chapter-3-sensors/index.md
```

## Testing Your Setup

### Validate Gazebo Installation
```bash
gazebo --version
```

### Validate ROS Connection
```bash
# In one terminal
ros2 run demo_nodes_cpp talker

# In another terminal
ros2 run demo_nodes_py listener
```

### Validate Documentation Build
```bash
cd website
npm run build
```

## Next Steps
1. Complete Chapter 1: Physics Simulation and Environment Building in Gazebo
2. Move to Chapter 2: High-Fidelity Rendering and Human-Robot Interaction in Unity
3. Explore Chapter 3: Sensor Simulation (LiDAR, Depth Cameras, IMUs) and Integration
4. Try the hands-on exercises in each chapter
5. Experiment with creating your own digital twin scenarios

## Troubleshooting
- If Gazebo fails to start, ensure your graphics drivers are up to date
- If Unity cannot connect to ROS, verify that both systems are on the same network
- If documentation fails to build, check that all required npm packages are installed