# contrutle
Inverse kinematics interface between console controller and turtlebot3 manipulator. Ros learning project


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

Linux, Turtlebot or Gazeebo.
move it


### Installing
TODO
python 2.7
ros kinetic
turtlebot3 manipulator

ros kinetic
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```
turtlebot3 setup
```
sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
```

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

manipulator
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_manipulation_simulations.git
$ cd ~/catkin_ws && catkin_make
```





## Deployment

1.
```
roscore
```
if using gazebo:
skip step 2 and 3 and tun
```
roslaunch turtlebot3_manipulation_gazebo turtlebot3_manipulation_gazebo.launch
```

if using real turtle:
2.
ssh into turtle and run
```
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
3.
back you your computer run
```
$ roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
```
4.
then start the moveit node
```
$ roslaunch turtlebot3_manipulation_moveit_config move_group.launch
```

5.
setup controller
```
sudo xboxdrv --silent
```
if you get LIBUSB_ERROR_BUSY error 
run then try step 5 again.
```
sudo rmmod xpad
```

setup joy node parameters
```
rosparam set joy_node/deadzone "0.05"

rosparam set joy_node/autorepeat_rate "4.0"
```

start joy node // TODO dev/
```
rosrun joy joy_node
```

and finaly start conturtl
```
rosrun conturlter.py
```
