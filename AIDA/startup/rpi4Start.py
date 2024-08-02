import subprocess
import os
import time

#Set path
chmod_path = "/usr/bin/chmod"

#Start the boot
def run_commands(path):
    try:
        # Change to the workspace directory
        os.chdir('/home/aida/AIDA/AIDA/ros2_humble_ws/')
        print("Changed directory to /home/aida/AIDA/AIDA/ros2_humble_ws/")

        # Source the ROS 2 setup.bash script
        subprocess.run('source /opt/ros/humble/setup.bash', shell=True, check=True, executable='/bin/bash')
        print("Sourced /opt/ros/humble/setup.bash")
        time.sleep(3)

        # Change to the launch directory
        os.chdir('launch/')
        print("Changed directory to launch/")

        # Source the local setup.bash script
        subprocess.run('source /home/aida/AIDA/AIDA/ros2_humble_ws/install/local_setup.bash', shell=True, check=True, executable='/bin/bash')
        print("Sourced ../install/local_setup.bash")
        time.sleep(3)

        # Change permissions for /dev/ttyUSB0
        subprocess.run(['sudo', path ,'666', '/dev/ttyUSB0'], check=True)
        #subprocess.run(['sudo chmod 666 /dev/ttyUSB0'], shell=True, check=True)
        print("Changed permissions for /dev/ttyUSB0")

        # Launch the ROS 2 application
        subprocess.run('ros2 launch /home/aida/AIDA/AIDA/ros2_humble_ws/launch/rpi4.yaml', shell=True, check=True)        
        print("Launched ros2 launch rpi4.yaml")

    except subprocess.CalledProcessError as e:
        print(f"Command failed with error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
        run_commands(chmod_path)
