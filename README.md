# Table of Contents

- [Important](#important) 
- [Supported Versions](#supported-versions)
- [ARMada Workstation](#armada-workstation)
  - [Workstation Description](#workstation-description)
  - [Camera Topics](#camera-topics)
  - [Usage](#usage)
  - [Integrated Packages](#integrated-packages)

# Important
This package is currently under construction, a more complete and accurate model/URDF will be provided in the future as development of simulated work environments progresses.  

# Supported Versions
All branches have only been tested with ROS Kinetic.  

- The xacro files and URDF files contained within this package should be compatible with other distributions of ROS.  

- The user should confirm that the Gazebo plugins used to simulate RGBD cameras in this package are consistent with the ROS distribution that they are using (make sure to check the Gazebo documentation if there are issues/conflicts).  

# ARMada Workstation
The `uml_hri_nerve_armada_workstation` package is meant to provide a simulated alternative to the workstations constructed for experimentation/testing of industrial robotic arms within the NERVE Center.  

## Workstation Description
Currently only a rough model is provided which uses simple shapes to define the size and shape of the workstation, connected features, and a pedestal and oversized mounting plate for the robot.  

- L-shaped arms protruding from both sides and the rear of the workstation represent structures from which cameras are affixed and serve such purpose in the simulated model, allowing the simulated cameras a mounting point. 

- The fourth camera is suspended above the workstation, pointing down, and does not have any connected architecture. This can be accomplished many ways in the real world but is not necessary to model for our purposes at this moment.  

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

After ensuring that your robot can be simulated within Gazebo, the proper way to integrate the workstation into the simulated environment is to make a separate set of launch files that will semantically attach the robot to the workstation and parse its URDF along with that of the robot so that the robot\_state\_publisher can publish all transforms properly.  

In a typical robot Moveit! configuration package there are launch files for bringing up a real or simulated robot in Rviz with Moveit! controls as well as those to bring up the robot in a Gazebo environment which can subsequently be controlled via Moveit! in Rviz using the motion_planning GUI or otherwise with code. As mentioned above, we need to follow this process while adding another component to the model which is generated.  

In the robot_description (or similar location) there should be an .xacro file that contains instructions for how to assemble the robot and workstation model. The result should look something similar to the code provided below:  

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

The file which describes the workstation `workstation_model.urdf.xacro` is contained within this package and included as shown above.  

This method allows the user to create a virtual joint (in this case `connect_table_and_root`) between the robot and workstation, as well as a joint (`connect_world_and_table`) which will connect the table to the common reference frame, `world`, which normally the robot would be attached to.  

Typically there are two launch files in a moveit config; the first will launch a Gazebo world with the robot generated and the second will bring up Rviz with the Moveit! controls and robot visulaization. Both of these files need to point to the .xacro file mentioned above.  

It may be useful to make an edit to the planning\_context.launch file which is generally located in the \<robot\>_moveit_config/launch folder. Any files associated with launching the robot without the simulated workstation will need to refer to the old .xacro file that does not include the workstation. In order to preserve the original functionality while adding the workstation files, the planning\_context.launch file can be adjusted to include the following lines:  

```
  <!-- By default we are not loading the workstation into the environment -->
  <arg name="use_workstation" default="false"/>

  <!-- Load universal robot description format (URDF) (No workstation) -->
  <group unless="$(arg use_workstation)">
    <param if="$(arg load_robot_description)" name="$(arg robot_description)" command="$(find xacro)/xacro.py '$(find kinova_description)/urdf/j2s7s300_standalone.xacro'"/>
  </group>

  <!-- Load universal robot description format (URDF) -->
  <group if="$(arg use_workstation)">
    <param if="$(arg load_robot_description)" name="$(arg robot_description)" command="$(find xacro)/xacro.py '$(find kinova_description)/urdf/j2s7s300_workstation.xacro'"/>
  </group>
```

When using this method, it is important that you pass in an argument when including the planning\_context.launch file in another launch file or when launching from the terminal. An example of this is shown below (from the launch file `j2s7s300\_gazebo_workstation_demo.launch` in the [j2s7s300_moveit_config/launch](https://github.com/uml-robotics/kinova-ros/tree/master/kinova_moveit/robot_configs/j2s7s300_moveit_config/launch) folder in the forked [uml-robotics/kinova-ros](https://github.com/uml-robotics/kinova-ros/tree/master) repository:  

```
  <!-- Load the URDF, SRDF and other .yaml configuration files on the param server -->
  <include file="$(find j2s7s300_moveit_config)/launch/planning_context.launch">
    <arg name="load_robot_description" value="true"/>
    <arg name="use_workstation" value="true"/>
  </include>
```

## Integrated Packages
The following is a list of industrial robot repositories/moveit config packages which are already configured to launch the robot and workstation together in a simulated Gazebo environment:  

1. [uml-robotics/kinova-ros](https://github.com/uml-robotics/kinova-ros/tree/master)
  - A forked version of the [Kinovarobotics/kinova-ros](https://github.com/Kinovarobotics/kinova-ros) repository for the Jaco2 and Mico robotic arms
  - The simulation can be run with the following command for the workstation and j2s7s300 version of the Jaco2 robot: 
	`roslaunch j2s7s300_moveit_config j2s7s300_gazebo_workstation_and_robot.launch`

More repositories will be added to the list as they are properly configured. These packages will contain edited, forker robot repositories as well as custom moveit configs for robots whose structure is altered slightly for certain additional equipment where applicable.  




