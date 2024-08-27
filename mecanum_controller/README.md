# Mecanum Controller

This package offers a controller module designed to oversee a mecanum wheel platform utilizing the ROS2 Control framework.
Presently, the controller is optimized for a mecanum platform equipped with four wheels, although this specification could evolve in the coming times.

The underlying architecture of the controller draws from the principles of the [differential drive](https://github.com/ros-controls/ros2_controllers/tree/master/diff_drive_controller) controller.

$~$

## Table of Content
  * [Updates](#updates)
  * [Installation](#installation)
  * [Usage](#usage)
    * [Parameters](#parameters)
  * [TODO](#todo)

$~$

## Updates
**2023-08-30**
- Resolved mathematical issues related to wheel velocities and odometry
- Added functionality to check and adjust maximum wheel rotational speed

$~$

## Installation
1. #### Install the [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) distribution
   Follow the [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) installation guide.

2. #### Install the [ros2_control](https://github.com/ros-controls/ros2_control/tree/humble) framework and [ros2_controllers](https://github.com/ros-controls/ros2_controllers)
   Install the necessary packages using the following commands:
   ```bashrc
   sudo apt install -y ros-<ROS_DISTRO>-ros2-control ros-<ROS_DISTRO>-ros2-controllers
   ```

3. #### Create a new workspace
   Create a workspace and navigate to the source directory:
   ```bashrc
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

5. #### Clone this controller repository into '~/ros2_ws/src/'
   Clone the controller repository using the following command:
   ```bashrc
   git clone https://github.com/roboTHIx/mecanum_controller.git --branch=humble
   ```

7. #### Build and source
   Build the workspace and source the workspace:
   ```bashrc
   cd ~/ros2_ws
   colcon build --symlink-install --packages-select mecanum_controller
   source ~/ros2_ws/install/setup.bash
   ```

$~$

## Usage
### Parameters
[Parameter File](https://github.com/roboTHIx/mecanum_controller/blob/humble/src/mecanum_controller_parameter.yaml)
- **publish_rate**:
  - Defines the rate at which the controller calculates the required wheel velocities and, if chosen, the odometry based on these velocities.
- **DIR_SIDE_wheel_name**:
  - Specifies the name of each wheel joint (e.g., "wheel_front_left_joint"), which must be defined in the URDF file for the robot's description.
- **wheel_separation_AXIS**:
  - Represents the separation between the wheel treads in the X and Y directions. [m]
- **wheel_max_rotational_speed**:
  - Sets the maximum rotational speed of the wheels. Setting this to 0.0 will disregard this option. [rad/s]
- **wheel_radius**:
  - Defines the radius of the wheels. [m]
- **wheel_separation_multiplier_AXIS**:
  - Provides a correction factor for the wheel separation in the X and Y directions.
- **DIR_SIDE_wheel_radius_multiplier**:
  - Offers a correction factor when the radius of the front left wheel deviates from the nominal value specified in the **DIR_SIDE_wheel_radius** parameter.
- **tf_frame_prefix**:
  - Prefix appended to the tf frames; added to **odom_id** and **base_frame_id** before publishing. If empty, the controller's namespace will be used.
- **odom_frame_id**:
  - Name of the frame for odometry. This frame is the parent of **base_frame_id** when the controller publishes odometry.
- **base_frame_id**:
  - Name of the robot's base frame, which is a child of the odometry frame.
- **pose_covariance_diagonal**:
  - Odometry covariance for the robot's pose based on encoder output. Initial values: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
- **twist_covariance_diagonal**:
  - Odometry covariance for the robot's speed based on encoder output. Initial values: [0.001, 0.001, 0.001, 0.001, 0.001, 0.01]
- **open_loop**:
  - When set to true, the robot's odometry will be calculated from the calculated wheel speed values rather than using feedback.
- **position_feedback**:
  - Indicates the presence of position feedback from the hardware.
- **enable_odom_tf**:
  - Publishes the transformation between **odom_frame_id** and **base_frame_id**.
- **publish_limited_velocity**:
  - Publish the limited velocity value.
- **use_stamped_vel**:
  - Uses the timestamp from the input velocity message to calculate the command's actual age.
- **MOTION.DIR.LIMIT**:
  - Limits for each motion direction and limit (velocity, acceleration, or jerk). Refer to the parameter file for more information.

$~$

## TODO
- [x] Implement a wheel max rotational speed checker and scaler
- [ ] Revise wheel setup to be parametric, allowing for an arbitrary number of wheels instead of being limited to 4.
