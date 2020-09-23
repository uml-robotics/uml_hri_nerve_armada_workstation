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

1. camera_left
2. camera_right
3. camera_rear
4. camera_top

You can use the following command to view the full list of camera depth and color topics when in use:  
```
rostopic list -v | grep camera
```

## Usage
This package supplements other moveit config packages which have the proper launch and configuration files to bring up the robot on the workstation in a gazebo simulation  

- The following changes have already been applied to any repositories in the [Integrated Packages](#integrated-packages) section of this README and this information serves as a guide to include additional repositories in the future.  

- In the robot\_description directory (or similar location) of a new robot's moveit_config package, the user should create an .xacro file that contains instructions for how to assemble the robot and workstation model. The result should look something similar to the code provided below from the forked kinova-ros repository linked in [Integrated Packages](#integrated-packages) below:  

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

- This method allows the user to create a virtual joint (in this case `connect_table_and_root`) between the robot and workstation as well as a joint (`connect_world_and_table`) to connect the table to the common reference frame, `world`, which normally the robot would be attached to.  

- Typically two launch files are used to bring up the robot (real or simulation) and to launch Rviz with Moveit! controls and visualizations. Both of these files need to point to the .xacro file mentioned above in order to generate the correct workstation model.  

- It may be useful to make an edit to the planning\_context.launch file which is generally located in the *your_robot*\_moveit_config/launch folder.  
  - Any existing files associated with launching the robot without the simulated workstation will need to refer to the old .xacro file that does not include the workstation.  
  - Loading different robot parameters depending on an input argument, `use_workstation`, simplifies this process and does not require you to edit any other existing files.  

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

- The launch file that includes (launches) the planning\_context.launch file needs an additonal argument when bringing up the workstation configuration:  

```
  <!-- Load the URDF, SRDF and other .yaml configuration files on the param server -->
  <include file="$(find j2s7s300_moveit_config)/launch/planning_context.launch">
    <arg name="load_robot_description" value="true"/>
    <arg name="use_workstation" value="true"/>
  </include>
```

- More information regarding how to launch the workstation configurations will be included in the modified/custom moveit_config repositories as well as in the [Integrated Packages](#integrated-packages) section of this README.  

## Integrated Packages
The following is a list of industrial robot repositories/moveit config packages which are already configured to launch the robot and workstation together in a simulated Gazebo environment:  

1. [uml-robotics/kinova-ros](https://github.com/uml-robotics/kinova-ros/tree/master)
  - A forked version of the [Kinovarobotics/kinova-ros](https://github.com/Kinovarobotics/kinova-ros) repository for the Jaco2 and Mico robotic arms
  - The simulation can be run with the following command for the workstation and multiple versions of the Jaco2/Mico robot: 
	`roslaunch kinova_gazebo gazebo_robot.launch kinova_robotType:=<robot_version> sim_workstation:=<true_or_false>`
  - The default value for argument `use_sim_workstation` is `false`, so it can be omitted if you wish to lauch the robot in gazebo without the workstation
  - The following robot versions are supported: `j2n6s300`, `j2s6s300`, `j2s7s300`, `m1n6s300`
  - The following robot versions currently have some odd behavior in gazebo when launched with the workstation: `j2n6s300`, `j2s6s300` 

2. [uml-robotics/ros_kortex](https://github.com/uml-robotics/ros_kortex)
  - A forked version of the [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex) repository for Kinova Kortex (Gen3/Gen3 lite) robotic arms
  - The simulation can be run with the following command for the workstation and Gen3/Gen3 lite robot
	`roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=<robot_version> gripper:=<robot_gripper> sim_workstation:=<true_or_false>`
  - The default value for argument `use_sim_workstation` is `false`, so it can be omitted if you wish to lauch the robot in gazebo without the workstation
  - The following robot versions are supported: `gen3`, `gen3_lite`
  - The following robot grippers are supported: `gen3_lite_2f`, `robotiq_2f_140`, `robotiq_2f_85`

More repositories will be added to the list as they are properly configured. These packages will contain edited, forker robot repositories as well as custom moveit configs for robots whose structure is altered slightly for certain additional equipment where applicable.  

3. ur5e package
  - TODO:  

4. omron techman package
  - TODO:  

5. kuka package
  - TODO:  

6. yaskawa motoman package
  - TODO:  




