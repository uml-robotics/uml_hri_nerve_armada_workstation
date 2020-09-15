# Table of Contents

- [Important](#important) 
- [Supported Versions](#supported-versions)
- [ARMada Workstation](#armada-workstation)
  - [Camera Topics](#camera-topics)

# Important
I am testing this.  

# Supported Versions
All branches have only been tested with ROS Kinetic.  
The xacro files and URDF files contained within this package should be compatible with other distributions of ROS. The user should however confirm that the Gazebo plugins used to support RGBD cameras are consistent with the ROS distribution that they are using (particularly the version of Gazebo they are using).  

# ARMada Workstation
This package is meant to provide a simulated alternative to the workstations constructed for experimentation/testing of industrial robotic arms within the NERVE Center.  

Currently, only a simple model is provided which uses simple shapes to define the rough size and shape of the workstation. L-shaped arms protruding from both sides and the rear of the workstation represent structures from which cameras could be affixed and serve such purpose in the simulated model. The fourth camera is suspended above the workstation, pointing down, and does not have any connected architecture.  

## Camera Topics
The workstation model includes four simulated RGBD cameras which have the following topics published by Gazebo:  

1. /left_camera
2. /right_camera
3. /rear_camera
4. /top_camera

Each camera has a color and depth component.  
