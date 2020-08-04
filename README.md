# contrutle
Inverse kinematics interface between console controller and turtlebot3 manipulator. Ros learning project


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development. See deployment for notes on how to deploy the project on a live system.

## Prerequisites

Linux Ubuntu 16.04 or 18.04.
A Turtlebot3 waffle with a manipulator or Gazebo.
> conturtle can be run on live turtlebot3 or one simulated in gazebo
Ros kinetic or melodic
python 2.7


## Setup



### turtlebot3 and manipulator
Follow insturctions [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/) and [here](
https://emanual.robotis.com/docs/en/platform/turtlebot3/manipulation/#software-setup):

### python modules
rospy and liberies
```
pip install roslibpy
```

```
pip install  moveit_commander
```

### Installing

```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/Ehrensverd/contrutle
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
>if you get LIBUSB_ERROR_BUSY error then run 
```
sudo rmmod xpad
```
>then try step 5 again.


6.
conturtle
```
 roslaunch conturtle conturtle.launch
```
