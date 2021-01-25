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

- The following changes have already been applied to any repositories in the [Integrated Packages](#integrated-packages) section of this README so this information serves as a supplementary guide for including additional repositories in the future. There are typically minor differences in how each repository is set up so the following steps won't all necessarily be correct in each case. The reader should instead use the information presented as clues rather than rules for any related future work or additions. The best way to gain insight in any case is to follow the main launch file to better understand any arguments or parameters used and to find out exactly which files are called, and how decisions are made based upon any inputs from the user.  

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

- In the case of the kinova-ros package for the Mico and Jaco2 robots, two launch files are used to bring up the robot (real or simulation) and to launch Rviz with Moveit! controls and visualizations. Both of these files point to the .xacro file mentioned above in order to generate the correct workstation model.  

- In this case it may be useful to make an edit to the planning\_context.launch file, which is generally located in the *your_robot*\_moveit_config/launch folder, since it refers to the file that generates your robot description. ***Note that this is not always the case in every moveit\_config, examine your files before you make arbitrary changes to detemine if they are actually required.***  
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

1. [uml-robotics/kinova-ros](https://github.com/uml-robotics/kinova-ros/tree/dev/bflynn)
  - A forked version of the [Kinovarobotics/kinova-ros](https://github.com/Kinovarobotics/kinova-ros) repository for the Jaco2 and Mico robotic arms
  - The simulation can be run with the following command for the workstation and multiple versions of the Jaco2/Mico robot:  
	`roslaunch kinova_gazebo gazebo_robot.launch kinova_robotType:=<robot_version> sim_workstation:=<true_or_false>`
  - The default value for argument `use_sim_workstation` is `false`, so it can be omitted if you wish to lauch the robot in gazebo without the workstation
  - The following robot versions are supported: `j2n6s300`, `j2s6s300`, `j2s7s300`, `m1n6s300`
  - The following robot versions currently have some odd behavior in gazebo when launched with the workstation: `j2n6s300`, `j2s6s300`
  - At the NERVE Center, our physical model is the `j2s7s300`

2. [uml-robotics/ros_kortex](https://github.com/uml-robotics/ros_kortex/tree/dev/bflynn)
  - A forked version of the [Kinovarobotics/ros_kortex](https://github.com/Kinovarobotics/ros_kortex) repository for Kinova Kortex (Gen3/Gen3 lite) robotic arms
  - The simulation can be run with the following command for the workstation with a Gen3 or Gen3 lite robot:  
	`roslaunch kortex_gazebo spawn_kortex_robot.launch arm:=<robot_version> gripper:=<robot_gripper> sim_workstation:=<true_or_false> dof:=<6_or_7>'`
  - The default value for argument `use_sim_workstation` is `false`, so it can be omitted if you wish to lauch the robot in gazebo without the workstation
  - The following robot versions are supported: `gen3`, `gen3_lite`
  - The following robot grippers are supported: `gen3_lite_2f`, `robotiq_2f_85`, `robotiq_2f_140`
  - At the NERVE Center, our physical model is the `gen3` with a `robotiq_2f_85` gripper

3. [uml-robotics/universal_robot](https://github.com/uml-robotics/universal_robot/tree/dev/bflynn)
  - A forked version of the [ros-industrial/universal_robot](https://github.com/ros-industrial/universal_robot.git) repository for the UR3, UR5, and UR10 CB-Series (older) and e-Series (newer) industrial robot arms
  - The simulation can be run with either of the following commands, depending on which robot series you want to use, for the workstation with a UR3/UR3e, UR5/UR5e, and UR10/UR10e robots with standard or joint-limited configurations:  
  1. `roslaunch ur_e_gazebo gazebo_universal_robot.launch limited:=<true_or_false> sim_workstation:=<true_or_false> model:=<robot_model>`  
  2. `roslaunch ur_gazebo gazebo_universal_robot.launch limited:=<true_or_false> sim_workstation:=<true_or_false> model:=<robot_model>`  
  - Use the `ur_e_gazebo` package to launch any e-series robots and the `ur_gazebo` package to launch any CB-series robots
  - The following robot versions are supported for either launch file: `ur3`, `ur5`, `ur10`
  - The robots behave less unexpectedly in simulation when their joints are limited, however it is not a requirement to do so. To limit the joints, set the `limited` argument to `true`
  - The following robot grippers are supported: `robotiq_2f_85`, `robotiq_2f_140`
  - At the NERVE Center, our physical model is the e-Series `ur5` with a `robotiq_2f_85` gripper

4. [uml-robotics/ros-driver-techman-robot](https://github.com/uml-robotics/ros-driver-techman-robot/tree/dev/bflynn)
  - A forked version of the [viirya/ros-driver-techman-robot](https://github.com/viirya/ros-driver-techman-robot) repository for the tm700 (tm5-700) and tm900 (tm5-900) industrial robot arms
  - The simulation can be run with the following command for the workstation with a tm5-700 robot:  
	`roslaunch tm_gazebo tm700_gazebo_moveit.launch model:=<robot_model> sim_workstation:=<true_or_false> gripper:=<robot_gripper>`
  - The default value for argument `sim_workstation` is `false`, so it can be omitted if you wish to lauch the robot in gazebo without the workstation
  - The following robot version is supported: `iiwa7`
  - The following robot grippers are supported: `robotiq_2f_85`, `robotiq_2f_140`
  - At the NERVE Center, our physical model is the `tm700` with `eih` and no gripper

5. [uml-robotics/iiwa_stack](https://github.com/uml-robotics/iiwa_stack/tree/dev/bflynn)
  - A forked version of the [IFL-CAMP/iiwa_stack](https://github.com/IFL-CAMP/iiwa_stack) repository for the ROS integration for the KUKA LBR IIWA R800/R820 (7/14 Kg)
  - The simulation can be run with the following command for the workstation with an LBR iiwa 7 robot:  
	`roslaunch iiwa_gazebo gazebo_iiwa_robot.launch model:=<robot_model> sim_workstation:=<true_or_false> gripper:=<robot_gripper> eih:=<true_or_false>`
  - The default value for argument `limited`, `sim_workstation` and `eih` is `false`, so it can be omitted if you wish to lauch the robot in gazebo without the workstation
  - The following robot version is supported: `tm700`
  - The following robot grippers are supported: `robotiq_2f_85`, `robotiq_2f_140`
  - At the NERVE Center, our physical model is the `iiwa7` with no gripper

6. [uml-robotics/motoman](https://github.com/uml-robotics/motoman/tree/dev/bflynn)
  - A forked version of the [ros-industrial/motoman](https://github.com/ros-industrial/motoman) repository for ROS-Industrial Motoman support
  - The simulation can be run with the following command for the workstation with a motoman gp7 robot:  
	`roslaunch motoman_gazebo gazebo_motoman.launch model:=<robot_model> sim_workstation:=<true_or_false> gripper:=<robot_gripper>`
  - The default value for argument `sim_workstation` is `false`, so it can be omitted if you wish to lauch the robot in gazebo without the workstation
  - The following robot version is supported: `gp7`
  - The following robot grippers are supported: `robotiq_2f_85`, `robotiq_2f_140`
  - At the NERVE Center, our physical model is the `gp7` with no gripper




