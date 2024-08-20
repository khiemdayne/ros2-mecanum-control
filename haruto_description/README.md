# haruto_description
This repository is for haruto robot related urdf model

## Dependencies involved
- [ ] robot_state_publisher
- [ ] joint_state_publisher_gui

## Install dependencies
```
  rosdep install --from-paths src/haruto_description -y --ignore-src
```

## Build and Execution
- To build the package

```
  colcon build --packages-select haruto_description
```

- To visualize the robot in rviz
```
  ros2 launch haruto_description visualize_robot_standalone.launch.py
```

- To visualize the robot in gazebo and rviz (for omnidirectional robot)
```
  ros2 launch haruto_description visualize_robot_simulation.launch.py robot_type:=omni
```

- To visualize the robot in gazebo and rviz (for differential robot)
```
  ros2 launch haruto_description visualize_robot_simulation.launch.py robot_type:=diff
```


#### Working video 

[![Omni directional drive robot](https://img.youtube.com/vi/hTd9ykmecRg/0.jpg)](https://youtu.be/hTd9ykmecRg)

[![Differential drive robot](https://img.youtube.com/vi/BXSWMmkk77E/0.jpg)](https://youtu.be/BXSWMmkk77E)
