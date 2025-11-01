packages installiert : scipy, transforms3d

sudo apt install -y ros-jazzy-moveit-py

roboter.move_to_home() - Move to safe starting position
roboter.move_to_pose(x, y, z, roll, pitch, yaw) - Move to specific pose
roboter.gripper_open() - Open gripper
roboter.gripper_close() - Close gripper
roboter.wait(seconds) - Wait for specified time



ABLAUF ///////////////////////////////

// Verbinden mit Roboter
ros2 launch igus_rebel rebel.launch.py

