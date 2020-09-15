# Table of Contents

- [Important](#important) 
- [Supported Versions](#supported-versions)
- [ARMada Workstation](#armada-workstation)
  - [Camera Topics](#camera-topics)

# Important
This package is currently under construction, a more complete and accurate model/URDF will be provided in the future as development of simulated work environments progresses.  

# Supported Versions
All branches have only been tested with ROS Kinetic.  

The xacro files and URDF files contained within this package should be compatible with other distributions of ROS.  

The user should confirm that the Gazebo plugins used to simulate RGBD cameras in this package are consistent with the ROS distribution that they are using (make sure to check the Gazebo documentation if there are issues/conflicts).  

# ARMada Workstation
This package is meant to provide a simulated alternative to the workstations constructed for experimentation/testing of industrial robotic arms within the NERVE Center.  

Currently, only a simple model is provided which uses simple shapes to define the rough size and shape of the workstation. L-shaped arms protruding from both sides and the rear of the workstation represent structures from which cameras could be affixed and serve such purpose in the simulated model. The fourth camera is suspended above the workstation, pointing down, and does not have any connected architecture.  

## Camera Topics
The workstation model includes four simulated RGBD cameras which Gazebo puhlishes on various topics:  

1. /left_camera
2. /right_camera
3. /rear_camera
4. /top_camera

Each camera has a color and depth component.  

You can use a command such as:  
```
rostopic list -v | grep camera
```
in order to view the full list of camera depth and color topics when in use.  
