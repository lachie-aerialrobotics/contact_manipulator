# contact_manipulator
Code to run 2PRS-1PRU manipulator for surface contact/sensor placement from an aerial vehicle
# Setup
Install ROS (tested on noetic but any distro probably works)
Install dynamixel workbench from these instructions: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_workbench/
clone this repo and catkin build
# Quickstart
type: "roslaunch contact_manipulator simple_moves.launch" to start
# Explanation
## src/publishers/example_trajectories.py
Publishes position setpoints for pitch/yaw/extension of manipulator, parameters can be adjusted through rqt_reconfigure
## src/Inverse-kinematics.py
Subscribes to position setpoints and publishes servo angles
## src/Servo_sync_writer
Subsribes to servo angles and efficiently pings target angles to dynamixels
# msgs
The main ros message you'll care about is [robot_name]/tip_position. The message type is PointStamped.
x: angle theta in radians, y: angle phi in radians, z: distance z in m
(bit janky but it works :p)
If you get confused about which angle is what or +ve/-ve directions, they're drawn on the manipulator.
# todo:
Upload and run on nuc
Implement current/force control
