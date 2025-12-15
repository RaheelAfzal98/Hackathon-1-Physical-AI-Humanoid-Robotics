# ROS 2 Humble Hawksbill Development Environment Setup

## Prerequisites
- Ubuntu 22.04 LTS (recommended) or Windows Subsystem for Linux (WSL)
- Python 3.10
- Internet connection for package downloads

## Installation Steps

### For Ubuntu 22.04:
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /etc/apt/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Hawksbill
sudo apt update
sudo apt install ros-humble-desktop

# Set up environment
source /opt/ros/humble/setup.bash
```

### For Windows (using WSL):
1. Install Windows Subsystem for Linux (WSL) with Ubuntu 22.04
2. Follow the Ubuntu installation steps above
3. Install the ROS 2 Visual Studio Code extension for development support

## Verification
```bash
# Verify installation
ros2 --version
ros2 topic list
```

## Additional Tools
```bash
# Install colcon (build tool)
sudo apt install python3-colcon-common-extensions

# Install rclpy (Python client library)
pip3 install rclpy
```

## Environment Setup
Add the following to your ~/.bashrc file:
```bash
source /opt/ros/humble/setup.bash
```

## Documentation Resources
- Official ROS 2 Documentation: https://docs.ros.org/en/humble/
- ROS 2 Tutorials: https://docs.ros.org/en/humble/Tutorials.html
- ROS 2 API Documentation: https://docs.ros2.org/humble/api/