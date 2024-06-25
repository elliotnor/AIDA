import subprocess
import os

def run_commands():
    try:
        # Change to the workspace directory
        os.chdir('aida/AIDA/ros2_humble_ws/')
        print("Changed directory to aida/AIDA/ros2_humble_ws/")

        # Source the ROS 2 setup.bash script
        subprocess.run('source /opt/ros/humble/setup.bash', shell=True, check=True, executable='/bin/bash')
        print("Sourced /opt/ros/humble/setup.bash")

        # Change to the launch directory
        os.chdir('launch/')
        print("Changed directory to launch/")

        # Source the local setup.bash script
        subprocess.run('source ../install/local_setup.bash', shell=True, check=True, executable='/bin/bash')
        print("Sourced ../install/local_setup.bash")

        # Change permissions for /dev/ttyUSB0
        subprocess.run('sudo chmod 666 /dev/ttyUSB0', shell=True, check=True)
        print("Changed permissions for /dev/ttyUSB0")

        # Launch the ROS 2 application
        subprocess.run('ros2 launch rpi4.yaml', shell=True, check=True)
        print("Launched ros2 launch rpi4.yaml")

    except subprocess.CalledProcessError as e:
        print(f"Command failed with error: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

if __name__ == "__main__":
        run_commands()
