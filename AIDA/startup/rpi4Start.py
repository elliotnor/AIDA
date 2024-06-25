import subprocess

file_path = "/dev/ttyUSB0"
permissions = "666"

chmod_command = ['sudo', 'chmod', permissions, file_path]

try:
    result = subprocess.run(chmod_command, check=True, text=True, capture_output=True)

    if result.returncode == 0:
        print ("Success")
    else:
        print("Failure")

except subprocess.CalledProcessError as e:
    print(f"Command failed as: {e}")
except Exception as e:
    print(f"An unexepected error occured: {e}")
    