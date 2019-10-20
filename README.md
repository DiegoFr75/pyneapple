# Pyneapple

### Package Description

This repository is structured as a ROS package. The folder organization is as follows:
launch – contains ROS launch files;
script – nodes to control the robot during the simulation;
config - navigation stack for the SLAM process
The package’s name is “pyneapple” and its dependencies are: rospy roscpp rosi_defy std_msgs message
generation

### Installation

The simulator was conceived using Ubuntu 18.4.2, ROS Melodic and V-REP 3.6.2.

1. Clone and download this repository package to your ROS Workspace folder.
2. To create the package used, and its dependencies, type in a new terminal window:

~~~
$ catkin_create_pkg pyneapple rospy roscpp rosi_defy std_msgs message_generation
~~~

3. Install some support packages:

~~~
$ sudo apt-get install ros-melodic-pointcloud-to-laserscan ros-melodic-vision-opencv ros-melodic-slam-gmapping ros-melodic-gmapping ros-melodic-amcl
~~~

4. Go back to the root of catkin workspace and type in a terminal window:

~~~
$ catkin build
~~~

To compile your catkin workspace.

### ROSI publishes to:


    /rosi/arms_joints_position - <rosi_defy/RosiMovementArray> - Rosi tracked arms position in [radians].

    /rosi/kinect_joint - <std_msgs/Float32> - Kinect joint position in [radians].

    /sensor/gps - <sensor_msgs/NavSatFix> - Emulated GPS sensor output.

    /sensor/imu - <sensor_msgs/Imu> - Emulated IMU sensor output.

    /sensor/kinect_depth - <sensor_msgs/Image> - Emulated kinect depth image output.

    /sensor/kinect_rgb - <sensor_msgs/Image> - Emulated kinect rgb image output.

    /sensor/kinect_info - <sensor_msgs/CameraInfo> - Emulated kinect information.

    /sensor/velodyne - <sensor_msgs/PointCloud2> - Emulated Velodyne output.

    /sensor/hokuyo - <rosi_defy/HokuyoReading> - Emulated hokuyo output. It gives a vector of 3D coordinates of detected point with respect to hokuyo.

    /sensor/ur5toolCam - <sensor_msgs/Image - Emulated camera on UR5 tool.

    /simulation/time - <std_msgs/Float32> - V-REP simulation time in [seconds]

    /ur5/jointsPositionCurrentState - <rosi_defy/ManipulatorJoints> - UR-5 robotic manipulator joints current position.

    /ur5/forceTorqueSensorOutput - <geometry_msgs/TwistStamped> - UR-5 Force/Torque sensor output. It gives two vector of linear and angular forces and torques, respectively. Axis order is x, y, z.
    
    /odom - <nav_msgs/Odometry> - Pose and Orientation of the base_link in relation to the world.
    
    /scan - <sensor_msgs/LaserScan> - Transform the PointCloud2 message into LaserScan message


### ROSI subcribes to:


    /rosi/command_arms_speed - <rosi_defy/RosiMovementArray> - Sets the tracked arms angular velocity in [radians/s]. Command limits in [-0.52, 0.52] rad/s

    /rosi/command_traction_speed - <rosi_defy/RosiMovementArray> - Sets the traction system joint angular velocity in [radians/s]. The traction system drives simultaneously both the wheel and the tracks. Command limits in [-37.76, 37.76] rad/s.

    /rosi/command_kinect_joint - <std_msgs/Float32>- Sets the kinect joint angular position set-point. Joint limits are [-45°,45° ]. It has a built-in PID controller with maximum joint speed of |0.35| rad/s.

    /ur5/jointsPosTargetCommand - <rosi_defy/ManipulatorJoints> - Sets the UR-5 joints desired angular position. Each joint has a built-in PID controller. One may find more UR-5 info in here.



### To Execute:

~~~
roslaunch pyneapple move.launch
~~~
