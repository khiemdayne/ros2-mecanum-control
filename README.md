# Ros2 Mecanum control
This package offers a controller module designed to oversee a mecanum wheel platform utilizing the ROS2 Control framework.
Presently, the controller is optimized for a mecanum platform equipped with four wheels, although this specification could evolve in the coming times.

The underlying architecture of the controller draws from the principles of the [differential drive](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller) controller.

$~$


# Tai lieu tham khao
* https://github.com/deborggraever/ros2-mecanum-bot.git 
* https://github.com/roboTHIx/mecanum_controller.git
## Packages Description
* mecanum_controller: calculate dynamic
* mecanum_control:
* mecanum_description: description mecanum robot
* mecanum_hardware:

#### Prerequisites
This project is build and tested on Ubuntu 22.04 LTS with ROS 2 Humble LTS.  
* [ROS install instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [Colcon install instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

## Getting started

#### Setup workspace
```
cd src
git clone https://github.com/khiemdayne/ros2-mecanum-control.git
```

#### Install dependencies
```
cd ~/ your workspace
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

#### Build and run
```
cd ~/your workspace
colcon build --symlink-install
source install/setup.bash
ros2 launch mecanum_control mecanum_hardware_launch.py
```

### Run
```
ros2 control list_hardware_interfaces
```
#### You can see any some
```
command interfaces
	fl_wheel_joint/velocity [unavailable] [unclaimed]
	fr_wheel_joint/velocity [unavailable] [unclaimed]
	rl_wheel_joint/velocity [unavailable] [unclaimed]
	rr_wheel_joint/velocity [unavailable] [unclaimed]
state interfaces
	fl_wheel_joint/position
	fl_wheel_joint/velocity
	fr_wheel_joint/position
	fr_wheel_joint/velocity
	rl_wheel_joint/position
	rl_wheel_joint/velocity
	rr_wheel_joint/position
	rr_wheel_joint/velocity
```
### Open new terminal and run to control robot
```
ros2 topic pub --rate 10 mecanum_controller/cmd_vel geometry_msgs/msg/TwistStamped "
    twist:
      linear:
        x: 0.7
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 1.0"
```