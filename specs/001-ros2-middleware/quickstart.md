# Quickstart Guide: ROS2 Middleware Educational Module

## Prerequisites

Before starting with this module, ensure you have the following:

### System Requirements
- Ubuntu 22.04 LTS (recommended) or compatible Linux distribution
- At least 4GB RAM (8GB recommended)
- 20GB free disk space
- Python 3.8 or higher
- Node.js 16.x or higher (for Docusaurus site)
- Git for version control

### Software Dependencies
- ROS 2 Humble Hawksbill (or newer LTS version)
- colcon build system
- rclpy Python client library
- Docusaurus prerequisites (Node.js, npm/yarn)

## Installation and Setup

### 1. Install ROS 2 Humble Hawksbill

Follow the official installation guide for your platform:
```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-rosdep2 python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

### 2. Set up ROS 2 Environment

```bash
# Source the ROS 2 setup script
source /opt/ros/humble/setup.bash

# Add to your shell profile to make it permanent
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 3. Install Python Dependencies

```bash
pip3 install rclpy
pip3 install ros2cli
```

### 4. Set up Docusaurus Development Environment

```bash
# Clone the repository (if not already done)
git clone <repository-url>
cd <repository-directory>

# Navigate to website directory
cd website

# Install Node.js dependencies
npm install
```

## Running the Examples

### 1. Working with Chapter Content

1. Navigate to the module directory:
```bash
cd website/docs/module-1-ros2/
```

2. The module structure contains:
   - `intro.md` - Module introduction
   - `chapter-1-ros2/index.md` - Complete first chapter with exercises
   - `chapter-2-python-agents/index.md` - Complete second chapter with exercises
   - `chapter-3-urdf-modeling/index.md` - Complete third chapter with exercises

3. Each chapter contains integrated code examples that can be executed following the instructions within the content.

## Building and Running the Docusaurus Site

### 1. Local Development Server

```bash
cd website
npm start
```

This will start a local development server at http://localhost:3000

### 2. Build Static Site

```bash
cd website
npm run build
```

The built site will be available in the `build/` directory.

### 3. Serve Built Site Locally

```bash
cd website
npm run serve
```

## Testing Your Setup

Verify your ROS 2 installation with basic commands:

```bash
# Check ROS 2 installation
ros2 --version

# Verify Python client library
python3 -c "import rclpy; print('rclpy available')"

# Check available ROS 2 commands
ros2 --help
```

## Next Steps

1. Complete Chapter 1: Fundamentals of ROS 2 Nodes, Topics, and Services
2. Move on to Chapter 2: Integrating Python Agents with ROS 2 via rclpy
3. Complete Chapter 3: Modeling Humanoid Robots with URDF
4. Try the hands-on exercises in each chapter

## Troubleshooting

### Common Issues

**Issue**: "command 'ros2' not found"
**Solution**: Make sure you've sourced the ROS 2 setup script: `source /opt/ros/humble/setup.bash`

**Issue**: Python import errors for rclpy
**Solution**: Make sure ROS 2 environment is sourced before running Python scripts

**Issue**: Docusaurus site not building
**Solution**: Ensure Node.js and npm are properly installed and dependencies are installed with `npm install`

## Development Workflow

1. Make changes to content files
2. Run local Docusaurus server with `npm start` to see changes in real-time
3. Test code examples in ROS 2 environment
4. Run validation tests to ensure content quality
5. Commit changes following the repository's contribution guidelines

This quickstart guide provides the essential steps to get started with the ROS2 middleware educational module. For detailed instructions, refer to the individual chapters in the module.