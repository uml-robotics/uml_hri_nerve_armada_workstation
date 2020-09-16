# Table of Contents

- [Important](#important) 
- [Supported Versions](#supported-versions)
- [ARMada Workstation](#armada-workstation)
  - [Camera Topics](#camera-topics)
  - [Usage](#usage)
  - [Integrated Packages](#integrated-packages)

# Important
This package is currently under construction, a more complete and accurate model/URDF will be provided in the future as development of simulated work environments progresses.  

# Supported Versions
All branches have only been tested with ROS Kinetic.  

The xacro files and URDF files contained within this package should be compatible with other distributions of ROS.  

The user should confirm that the Gazebo plugins used to simulate RGBD cameras in this package are consistent with the ROS distribution that they are using (make sure to check the Gazebo documentation if there are issues/conflicts).  

# ARMada Workstation
The `uml_hri_nerve_armada_workstation` package is meant to provide a simulated alternative to the workstations constructed for experimentation/testing of industrial robotic arms within the NERVE Center.  

Currently, only a simple model is provided which uses simple shapes to define the rough size and shape of the workstation. L-shaped arms protruding from both sides and the rear of the workstation represent structures from which cameras could be affixed and serve such purpose in the simulated model. The fourth camera is suspended above the workstation, pointing down, and does not have any connected architecture.  

## Camera Topics
The workstation model includes four simulated RGBD cameras which Gazebo puhlishes on various topics:  

1. /left_camera
2. /right_camera
3. /rear_camera
4. /top_camera

You can use a command such as:  
```
rostopic list -v | grep camera
```
in order to view the full list of camera depth and color topics when in use.  

## Usage
This package supplements other moveit config packages which have the proper launch and configuration files to bring up the robot on the workstation in a gazebo simulation  

After ensuring that your robot can be simulated within Gazebo, the proper way to integrate the workstation into the simulated environment is to make a separate set of launch files that will semantically attach the robot to the workstation and parse its URDF along with that of the robot in order to be published by the robot_state_publisher.  

In a typical robot Moveit! configuration package there are launch files for bringing up a real or simulated robot in Rviz with Moveit! controls, as well as those to bring up the robot in a Gazebo environment which can subsequently be controlled via Moveit! in Rviz using the motion_planning GUI.  

In the robot_description (or similar location) there should be an .xacro file that 

```
  <xacro:include filename="$(find kinova_description)/urdf/j2s7s300.xacro"/>
  <xacro:include filename="$(find uml_hri_nerve_armada_workstation)/workstation_description/workstation_model.urdf.xacro" />

  <xacro:workstation prefix="table_"/>

  <!-- for gazebo -->
  <link name="world"/>
  
  <joint name="connect_world_and_table" type="fixed">
    <child link="table_base_link" />
    <parent link="world" />
    <origin xyz="0.0 0.0 0.41" rpy="0.0 0.0 0.0" />    
  </joint> 

  <link name="root"/>

  <joint name="connect_table_and_root" type="fixed">
    <child link="root" />
    <parent link="table_mounting_plate_link" />
    <origin xyz="0.0 0.0 0.03625" rpy="0.0 0.0 ${-pi/2}" />    
  </joint> 

  <xacro:property name="robot_root" value="root" />

  <xacro:j2s7s300  base_parent="${robot_root}"/>
```

## Integrated Packages
The following is a list of industrial robot repositories/moveit config packages which are already configured to launch the robot and workstation together in a simulated Gazebo environment:  

1. [uml-robotics/kinova-ros](https://github.com/uml-robotics/kinova-ros/tree/master)
  - A forked version of the [Kinovarobotics/kinova-ros](https://github.com/Kinovarobotics/kinova-ros) repository for the Jaco2 and Mico robotic arms
  - The simulation can be run with the following command for the workstation and j2s7s300 version of the Jaco2 robot: 'roslaunch j2s7s300_moveit_config j2s7s300_gazebo_workstation_and_robot.launch`

More repositories will be added to the list, this process is a work in progress.  




