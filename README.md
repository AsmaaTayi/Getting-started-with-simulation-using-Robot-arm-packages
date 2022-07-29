# Getting-started-with-simulation-using-Robot-arm-packages
a simulation using ROS packages that can be used to plan and execute motion trajectories for a robot arm.

I used **ROS Melodic** with **Ubuntu 18.04**.These packages were also tested under **ROS kinetic** and it works perfectly on **ROS Noetic** too.

The robot arm uses **Moveit** plugin to apply kinematics by the KDL solver. These packages can be tested in the gazebo simulation tool and the real robot arm.


## Dependencies
run this instruction inside your workspace:

```$ rosdep install --from-paths src --ignore-src -r -y```

make sure you installed all these packages:

for kinetic distro

```
$ sudo apt-get install ros-kinetic-moveit
$ sudo apt-get install ros-kinetic-joint-state-publisher ros-kinetic-joint-state-publisher-gui
$ sudo apt-get install ros-kinetic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-kinetic-ros-controllers ros-kinetic-ros-control
```

for melodic distro

```
$ sudo apt-get install ros-melodic-moveit
$ sudo apt-get install ros-melodic-joint-state-publisher ros-melodic-joint-state-publisher-gui
$ sudo apt-get install ros-melodic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-melodic-ros-controllers ros-melodic-ros-control
```

for noetic distro

```
$ sudo apt-get install ros-noetic-moveit
$ sudo apt-get install ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
$ sudo apt-get install ros-noetic-gazebo-ros-control joint-state-publisher
$ sudo apt-get install ros-noetic-ros-controllers ros-noetic-ros-control
```

## Robot Arm
The robot arm has 5 joints only 4 joints can be fully controlled via ROS and Rviz, the last joint (gripper) has a default motion executed from the Arduino code directly.
### Circuit diagram 
![circuit](https://user-images.githubusercontent.com/93100711/180415478-f4a169b0-a034-41d1-9ede-5e285cb0b767.png)

### Robot initial positions
![positions](https://user-images.githubusercontent.com/93100711/180415517-eee1fbc4-ff3d-4bdf-a155-6a33d2bcbd3b.png)


## Usage


Run the following instructions to use gazebo
```
$ roslaunch robot_arm_pkg check_motors.launch
$ roslaunch robot_arm_pkg check_motors_gazebo.launch
$ rosrun robot_arm_pkg joint_states_to_gazebo.py
```
(You may need to change the permission)

```$ sudo chmod +x ~/catkin_ws/src/arduino_robot_arm/robot_arm_pkg/scripts/joint_states_to_gazebo.py```





### Controlling the robot arm by Moveit and kinematics
```$ roslaunch moveit_pkg demo.launch```

Run the following instruction to use gazebo

```$ roslaunch moveit_pkg demo_gazebo.launch```

***
 
you can see the moveit packge inside the Rviz:






## Using OpenCV with the robot arm in ROS 

### In simulation (Gazebo)
- In a terminal run

```$ roslaunch moveit_pkg demo_gazebo.launch```

this will run Rviz and gazebo




- In another terminal 

```$ rosrun moveit_pkg get_pose_openCV.py```

This will detect **blue** color and publish the x,y coordinates to /direction topic





- Open another terminal
 
```$ rosrun moveit_pkg move_group_node```

This will subscribe to /direction topic and execute motion by using Moveit move group

