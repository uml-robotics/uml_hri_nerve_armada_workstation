# Table of Contents

- [Important](#important) 
- [Supported Versions](#supported-versions)
- [UML HRI NERVE ARMada Workstation](#uml-hri-nerve-armada-workstation)
  - [Supported versions](#supported-versions)

# Important
I am testing this.  

# Supported Versions
All branches have only been tested with ROS Kinetic  

# UML HRI NERVE ARMada Workstation
In order to use this package, you will need Moveit! installed  
http://docs.ros.org/kinetic/api/moveit\_tutorials/html/doc/getting\_started/getting\_started.html   

### Installing ros_kortex package:
(https://github.com/Kinovarobotics/ros_kortex)  
As always, checkout the package on Github for more information (click on the wiki link along the top for guides if there are any)  

sudo apt install python3 python3-pip  
sudo python3 -m pip install conan  
sudo pip3 install --upgrade jinja2  
conan config set general.revisions_enabled=1  
conan profile new default --detect > /dev/null  
conan profile update settings.compiler.libcxx=libstdc++11 default  

git clone -b kinetic-devel https://github.com/Kinovarobotics/ros_kortex.git  
catkin build  
rosdep install --from-paths src --ignore-src -y  
  
### Installing kinova_ros package:
(https://github.com/Kinovarobotics/kinova-ros/wiki/Gazebo)  
As always, checkout the package on Github for more information (click on the wiki link along the top for guides if there are any)  
  
sudo apt-get install ros-kinetic-gazebo-ros-control  
sudo apt-get install ros-kinetic-ros-controllers*  
sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin  

git clone -b master https://github.com/Kinovarobotics/kinova-ros  
catkin build  
rosdep install --from-paths src --ignore-src -y  

### Gripper Control information for the Gen3 Robot

#### Controlling the gripper via publishing a message to the terminal
You can command the gripper directly from your terminal using the following command:  
rostopic pub /my_gen3/robotiq_2f_85_gripper_controllerripper_cmd/goal control_msgs/GripperCommandActionGoal '{goal: {command: {position: 1, max_effort: 10}}}'  

The gripper's position goes from 0.0 (open) to 0.8 (closed)  

### Workstation Camera Topics
There are four (4) cameras attached to the workstation currently  
They are labeled from the robot's perspective:  

1. left\_camera
2. right\_camera
3. rear\_camera
4. top\_camera

The cameras are simulated within the Gazebo environment and as such are not viewable in Rviz (until you publish transforms for them!)  
You can use the following command to bring up a useful camera viewer GUI:  

rosrun rqt\_image\_view rqt\_image\_view  

## Workstation Jaco2 Sim
With no sim workstation:  
1st Terminal: roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2s7s300  
2nd Terminal: roslaunch j2s7s300_moveit_config j2s7s300_gazebo_demo.launch  

With sim workstation:
1st Terminal: roslaunch uml_hri_nerve_pick_and_place robot_launch.launch kinova_robotType:=j2s7s300  
2nd Terminal: roslaunch uml_hri_nerve_pick_and_place j2s7s300_gazebo_workstation_demo.launch  

## Workstation Gen3 Sim
With no sim workstation:  
roslaunch kortex_gazebo spawn_kortex_robot.launch  

With sim workstation:  
roslaunch uml_hri_nerve_pick_and_place gen3_gazebo_workstation.launch  
