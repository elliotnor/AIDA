#!/bin/bash

# Function to run the commands
run_commands() {
    local chmod_path=$1

    # Change to the workspace directory
    cd /home/aida/AIDA/AIDA/ros2_humble_ws/
    
    #Build the working directory    
    colcon build    

    # Source the ROS 2 setup.bash script
    source /opt/ros/humble/setup.bash
    
    # Change to the launch directory
    cd launch/
    
    # Source the local setup.bash script
    source /home/aida/AIDA/AIDA/ros2_humble_ws/install/local_setup.bash
    
    #Launch all ros nodes on rpi4 
    ros2 launch /home/aida/AIDA/AIDA/ros2_humble_ws/launch/rpi4.yaml
    echo "Launched ros2 launch rpi4.yaml"
}

# Main execution
chmod_path="/usr/bin/chmod"
run_commands $chmod_path
