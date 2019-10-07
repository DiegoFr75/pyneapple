# Pyneapple

### Package Description

This repository is structured as a ROS package. The folder organization is as follows:
lauch – contains a ROS launch file;
msg – message files needed to interact with the simulation;
script – nodes to control the robot during the simulation;
The package’s name is “pyneapple” and its dependencies are: rospy roscpp rosi_defy std_msgs message
generation

### Installation

The simulator was conceived using Ubuntu 18.4.2, ROS Melodic and V-REP 3.6.2.

1. Clone and download this repository package to your ROS Workspace folder.
2. To create the package used, and its dependencies, type in a new terminal window:

$ catkin_create_pkg pyneapple rospy roscpp rosi_defy std_msgs message_generation

3. Go back to the root of catkin workspace and type in a terminal window:

$ catkin build

To compile your catkin workspace.

### ROSI publishes to:

/direction
/move_base/cancel
/move_base/feedback
/move_base/goal
/move_base/result
/move_base/status
/rosi/arms_joints_position
/rosi/command_arms_speed
/rosi/command_kinect_joint
/rosi/command_traction_speed
/rosi/kniect_joint

/rosout
/rosout_agg
/sensor/gps
/sensor/hokuyo
/sensor/imu
/sensor/kinect_depth
/sensor/kinect_info
/sensor/kinect_rgb
/sensor/ur5toolCam
/sensor/velodyne
/simulation/time
/statistics
/tf
/ur5/forceTorqueSensorOutput
/ur5/jointsPosTargetCommand
/ur5/jointsPositionCurrentState

### ROSI subcribes to:

/sensor/hokuyo
/sensor/gps
/direction
/ur5/forceTorqueSensorOutput
/sensor/Kinect_rgb

