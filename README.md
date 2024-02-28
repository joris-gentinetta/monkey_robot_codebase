# Instructions for setup of the control interface for the robot monkey 

## Introduction

This ReadMe contains instructions for the setup and usage of the *control interface* for the robot monkey, which was started during Jaú Gretlers bachelor's thesis:  "Assembly and Programming of a Robot Monkey to Study Imitation Learning in Marmosets".


## Abbreviations and explanations of some terms 
- Raspberry Pi (RPP)
- Controller device: The device on which you want to run the simulation and interact with the robot

# Usage
## 1. Setup 
On the RPP:
- Start the ROS network by running ```roscore``` on the RPP.
On the Controller:
- Run ```roslaunch monkey demo.launch``` to start Rviz. 
- In a second terminal run ```python3 /home/robot-user/monkey_robot_codebase/monkey_interface/src/robot_gui.py``` to start the GUI.
- Arrange the windows such that you have Rviz on the left side of your screen and the GUI on the right side.

## 2. GUI usage
There are three options:
- ```Collect waypoints```: Lets you choose waypoints for the left and right arm iteratively until you choose to save. Whenever you have moved the EEF of the robot to a desired waypoint you must manipulate the rotation of one of the interactive markers (red/green/blue circles). This ensures that the last interactive marker position recorded by the monkey_interface node corresponds to the position of the hand. 

- ```Load and edit waypoints``` Lets you load a previously saved collection of waypoints. You can then add waypoints that will be appended.
- ```Plan/Execute``` When using a collection of waypoints for the first time, you first ```Load Waypoints```, then ```Plan Carthesian Path```, then ```Save Plan```, then choose the number of interpolation steps between waypoints, then ```Execute Plan```. When the robot is ready to start the movement, the ```Start``` button is activated. In consequent usage, it will be ```Load Plan```, then choose the number of interpolation steps between waypoints, then ```Execute Plan```.

# Installation and first setup
## 1. Install Ubuntu 20.04 on your controller device
You can get the image from [here](https://releases.ubuntu.com/20.04/).

## 2. Install Ubuntu 20.04 Server on the RPP
Use the [Raspberry Pi Imager](https://www.raspberrypi.com/software/) to flash a Ubuntu 20.04 Server Distribution to the RPP. Select "Other general-purpose OS/Ubuntu/Ubuntu Server 20.04.5 LTS (64 bit)" from the options. The RPP Imager will offer you to include the SSID and password of a local network in the image to be flashed, which can be useful to already set at this stage.
 
## 3. Setup a static IP on the RPP
1. SSH into the RPP
2. To enable a connection over ethernet:
- (on computer) https://docs.phillycommunitywireless.org/en/latest/device-configs/shared-connection/
- (on RPP) Enable ethernet interface: ```sudo ifconfig eth0 up```
3. Navigate to /etc/netplan
4. Create config.yaml and write the following (adapt the wlan0 settings to the connection you choose in the Rasperry Pi Imager):
```
network:
    version: 2
    wifis:
        renderer: networkd
        wlan0:
            access-points:
                JorisPhone:
                    password: genti111  
            dhcp4: no
            optional: true
            addresses:
                - 172.20.10.3/28              
            gateway4: 172.20.10.1                              
            nameservers:
                addresses:
                    - 8.8.8.8
    ethernets:
        eth0:
            addresses: [10.42.0.2/24] #<RPP_IP>
            gateway4: 10.42.0.1  #<PC_IP>
            nameservers:
                addresses: [8.8.8.8,8.8.4.4]
```

## 4. Install ROS on the controller device
Follow the steps described [here](https://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS on the controller device. Use *ros-noetic-desktop-full* 
 
## 5. Install ROS on the RPP
Follow the steps described [here](https://wiki.ros.org/noetic/Installation/Ubuntu) to install ROS on the RPP.

For the RPP it  suffices to install *ros-noetic-ros-base* instead of *ros-noetic-desktop-full*. The latter includes Rviz and Gazebo (simulations softwares) which are of no use on the headless RPP.

## 6. Setup the ROS Environment Variables
For the RPP with IP-Address <RPP_IP>, write these lines in the .bashrc file:
```
source /opt/ros/noetic/setup.bash
export ROS_IP=<RPP_IP>
export ROS_MASTER_URI=http://<RPP_IP>:11311
#e.g.:
source /opt/ros/noetic/setup.bash
export ROS_IP=10.42.0.2
export ROS_MASTER_URI=http://10.42.0.2:11311
```
For the desktop PC with IP-Address: <PC_IP>, write these lines in the .bashrc file:
```
source /opt/ros/noetic/setup.bash
export ROS_IP=<PC_IP>
export ROS_MASTER_URI=http://<RPP_IP>:11311
#e.g.:
source /opt/ros/noetic/setup.bash
export ROS_IP=10.42.0.1
export ROS_MASTER_URI=http://10.42.0.2:11311
```

Note that the ROS_MASTER_URI refers to the device which is the ROS master, i.e who starts and controls the ROS network. With these environment variables you must start the ROS network by running 
```
roscore
``` 
on the RPP. If you dont do that the setup assistant and all other nodes you try to launch will fail because they are trying to find the ROS master but can't, since roscore wasn't run on the device whose IP was exported as ROS_MASTER_URI. 

If you just want to use Rviz and not control the physical robot, just use the PC_IP in the ROS_MASTER_URI. Then the network will be started by your PC.

## 7. Install MoveIt on the controller device
Follow the instructions [here](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) to install MoveIt on the controller device. From this point onwards I am assuming you have a catkin workspace setup, to which I will refer as "ws_moveit" from now on. Note that this simply means that there is a folder in your /home directory called *ws_moveit*, in which you have executed all commands listed in the tutorial mentioned above. One command which you don't have to run is this one:
```
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel #unnecessary
```
 If you run it "fatal error" will be displayed, but that is irrelevant for our purposes.

## 8.1 Install package responsible for data saving/loading on the controller device
The package in question is called "Rospy message converter". You can install it with: 

```
sudo apt install ros-noetic-rospy-message-converter
```
Note that any other package which turns out to be missing should be installed analogously.

## 8.2 Install packages on the RPP
```
sudo apt install python3-roslaunch
```
For the servo hat:
```
sudo apt install python3-smbus
sudo apt-get install i2c-tools
sudo apt install python3-pip
sudo pip install adafruit-circuitpython-servokit
pip install RPi.GPIO
```

## 9. Generate a Moveit Config Package
These instructions assume you have setup a catkin workspace.
1. Download the folder "monkey_complete" from this repository. Place it in *ws_moveit/src*.
4. In *ws_moveit*, open a terminal and run ```catkin build monkey_complete``` to build the monkey_complete package.
5. Source the workspace by running ```source devel/setup.bash``` in *ws_moveit*.
6. Launch the moveit setup assistant with ``` roslaunch moveit_setup_assistant setup_assistant.launch```. Note that this will only work if:
- ```roscore``` is running on the RPP
- you have installed and setup moveit according to this [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
- you have sourced your workspace with ```source devel/setup.bash```
7. In the setup assistant, select "Create New Moveit Configuration Package"
8. Click on "Browse" and select your URDF file 
9. Click on "Load Files"
10. In **Self-Collision**: Set "Sampling Density to max" and click "Generate Collision Matrix"
11. In **Virtual Joints**: Create an entry according to the following picture:

![virtual_joints_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/d13b2786-2f2d-4cb3-a9d1-381b475a8d9a)

12. In **Planning groups**:

a. Add new planning groups by clicking on "Add group".

b. For the "Kinematic Solver" choose "kdl_kinematics_plugin".

c. For "Group Default Planner" choose "RRT".

d. Add components to the group by clicking on "Add Kin. Chain". First select the "Base Link" (in our case base_link) and press "Choose Selected". Second, select the "Tip Link" (e.g L_Hand_Link) and press "Choose Selected".

e. Add single links to a planning group by clicking "Add Links".

With this knowledge you can create the planning groups according to the following picture:

![planning_groups_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/bfb11fae-02b0-4fc5-a143-46e951aad534)

13. In **Robot Poses**: Create entries according to the following pictures:


![Screenshot from 2023-06-08 13-42-08](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/229191b2-0624-47ec-8534-6de6b541d00c)
![Screenshot from 2023-06-08 13-43-14](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/11759217-dc7c-43cf-b7c4-5f5431a7c3ce)
![Screenshot from 2023-06-08 13-44-13](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/43201653-b480-4239-a40e-ad5f28059e59)


14. In **End Effectors**: Create entries according to the following picture:
![eef_in_sa](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/67d41e35-fa33-42ad-b33a-849618700e0c)

15. In **Author Information**, type in a name and a valid email, otherwise we can't save the config 
16. In **Configuration Files**, specify the name of the package and place it in the source folder of your catkin workspace, e.g ```/home/<user>/ws_moveit/src/monkey```
17. Click "Generate Package" 
18. Now we need to build the moveit config package. To do that, open your catkin ws in a terminal and run:
 ```
 source devel/setup.bash
 catkin build monkey
 ``` 
19. In order to test the freshly generated config package, run (in the same terminal as before):

```
 source devel/setup.bash
 roslaunch monkey demo.launch
``` 

Now you should see a window popping up containing the simulation environment Rviz. It should look like this:
![Screenshot from 2023-06-08 13-59-16](https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/fbc231dd-3a55-407f-bff5-a6328b919ae9)

Note that when Rviz starts up, it will display a lot of logs, among other things potentially the warning, that the link "base_link" has an inertia specified in the URDF and that this is a problem. Do not try to change the inertia of this link in the URDF, that wont work. This warning means that your URDF is missing the snippet mentioned under 2. or that the snippet contains a mistake.

## 10. Modify the standard Rviz setup
In order to have a efficient workflow and make use of all the tools and ideas conceived during the thesis, the following modifications have to be made inside of Rviz and saved.
1. In the "Displays" panel, click on "Add". In the then appearing panel (named "Rviz"), order the visualizations "By display type", select "MarkerArray" and click "Ok". 

<img src="https://github.com/multiplexcuriosus/monkey_robot_codebase/assets/50492539/c7b48752-9687-4233-907a-78f788a528e0" width="400">

2. In the MotionPlanning Panel, tick the tickbox labeled "Approximate IK solutions". Without that enabled, we cannot manipulate the end effectors of the robots arms. 
3. In the Displays Panel, go to the MotionPlanning/Planned Path rider and uncheck the tickbox "Show Robot Visual". This will deactivate the animation of the planned robot arm trajectory, which can get annoying. 

4. Press "CTRL+S" to save the current Rviz config

## 11. Modify the standard kinematics config
In order to have a efficient workflow and make use of all the tools and ideas conceived during the thesis, a few modifications have to be made to the **Kinematics.yaml** file, which can be found in the "config" folder of your moveit-config-pkg. The file should look as follows:
```yaml
monkey_left_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 0.005
  goal_joint_tolerance: 0.0001
  goal_position_tolerance: 0.0001
  goal_orientation_tolerance: 0.001
  position_only_ik: True
monkey_right_arm:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 0.005
  goal_joint_tolerance: 0.0001
  goal_position_tolerance: 0.0001
  goal_orientation_tolerance: 0.001
  position_only_ik: True
monkey_head:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_search_resolution: 0.01
  kinematics_solver_timeout: 0.005
  goal_joint_tolerance: 0.0001
  goal_position_tolerance: 0.0001
  goal_orientation_tolerance: 0.001
  position_only_ik: True
```
List of changes made:
- added line ```position_only_ik: True``` to each planning group
- changed "kinematics_solver_search_resolution" to 0.01 for each planning group


## 12. Perform a quick test
If you did everything right up until here, you should be able to drag around the hands of the robot quite freely around in space (of course only inside the space which is reachable by the robot and permitted by his joint limits). If you can't drag around the hands, it might help to untick and retick the "Approximate IK solutions" box. This setting seems to deactivate itself sometimes. 


## 13. Download and build the monkey_interface 
1. From this repo, download the folder "monkey_interface" and place it in the "src" folder of your catkin ws
2. Run ```catkin build monkey_interface``` to build the package.
3. If you haven't done so already, run ```caktin build ``` in the src folder, to build all packages. This will take about 10min (if you never build them before).
4. Run ```chmod +x ~/ws_moveit/src/monkey_interface/src/monkey_interface.py``` to allow monkey_interface.py to be executed.

 