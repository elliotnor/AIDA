#!/bin/bash

# Ensure sudo doesn't require a password for Docker
sudo -n true
if [ $? -ne 0 ]; then
  echo "You must configure sudo to not require a password for Docker commands."
  exit 1
fi

# Run the Docker container
sudo docker run -it --rm --device=/dev/video0 --device=/dev/snd/controlC0 --device=/dev/snd/pcmC0D0c --network host ros2-aida

# Change directory to the appropriate workspace
cd /path/to/your/workspace

# Pull the latest changes from git
git pull

# Build the project using colcon
colcon build

# Source the local setup file
source install/local_setup.bash

# Change to the launch directory
cd launch

# Launch the ROS2 project
ros2 launch rpi5.yaml
