#!/bin/bash

# Function to run the commands
run_commands() {
    local chmod_path=$1

    # Change to the workspace directory
    # cd /home/aida/AIDA/AIDA/ros2_humble_ws/
    # echo "Changed directory to /home/aida/AIDA/AIDA/ros2_humble_ws/"

    # Source the ROS 2 setup.bash script
    source /opt/ros/humble/setup.bash
    echo "Sourced /opt/ros/humble/setup.bash"

    # Change to the launch directory
    # cd launch/
    # echo "Changed directory to launch/"

    # Source the local setup.bash script
    source /home/aida/AIDA/AIDA/ros2_humble_ws/install/local_setup.bash
    echo "Sourced /home/aida/AIDA/AIDA/ros2_humble_ws/install/local_setup.bash"

    # Change permissions for /dev/ttyUSB0
    sudo $chmod_path 666 /dev/ttyUSB0
    echo "Changed permissions for /dev/ttyUSB0"

    # Launch the ROS 2 application
    /opt/ros/humble/bin/ros2 launch /home/aida/AIDA/AIDA/ros2_humble_ws/launch/rpi4.yaml
    echo "Launched ros2 launch rpi4.yaml"
}

# Main execution
chmod_path="/usr/bin/chmod"
run_commands $chmod_path
