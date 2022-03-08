The Aerial Alter-EGO Simulator is based on RotorS(https://github.com/ethz-asl/rotors_simulator).

This simulator is compatible with the Aerial Alter-EGO onboard software(https://github.com/NMMI/aerial-alter-ego)

Installation Instructions - Ubuntu 20.04 with ROS Noetic
---------------------------------------------------------
 1. Install and initialize ROS Noetic desktop full, additional ROS packages, catkin-tools, and wstool:

 ```
 $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
 $ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 $ sudo apt-get update
 $ sudo apt-get install ros-noetic-desktop-full ros-noetic-joy ros-noetic-octomap-ros ros-noetic-mavlink python-wstool python-catkin-tools protobuf-compiler libgoogle-glog-dev ros-noetic-control-toolbox ros-noetic-mavros
 $ sudo rosdep init
 $ rosdep update
 $ source /opt/ros/noetic/setup.bash
 ```
 2. If you don't have ROS workspace yet you can do so by

 ```
 $ mkdir -p ~/catkin_ws/src
 $ cd ~/catkin_ws/src
 $ catkin_init_workspace  # initialize your catkin workspace
 $ wstool init
 ```
 3. Get the simulator and additional dependencies
  ```
  $ cd ~/catkin_ws/src
  $ git clone git@github.com:NMMI/aerial-alter-ego-simulator.git
  $ git clone git@github.com:ethz-asl/mav_comm.git
  $ git clone git@github.com:ethz-asl/glog_catkin.git
  $ git clone git@github.com:catkin/catkin_simple.git
  ```
  > **Note** if you want to use wstool you can replace the above commands with bash wstool set --git local_repo_name git@github.com:organization/repo_name.git and then do bash wstool update.

 4. Build your workspace with `python_catkin_tools` (therefore you need `python_catkin_tools`)

   ```
   $ cd ~/catkin_ws/
   $ catkin init
   $ catkin build
   ```

 5. Add sourcing to your `.bashrc` file

   ```
   $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   $ source ~/.bashrc
   ```

Gazebo Version
--------------

Gazebo 11.x is recommanded as a combination with ROS Noetic.
Reference: http://gazebosim.org/tutorials/?tut=ros_wrapper_versions

Basic Usage
-----------

Launch the simulator

```
$ roslaunch aerial-ego-simulation yigle_hovering.launch
```

The control command is received from the onboard software(https://github.com/NMMI/aerial-alter-ego).

Or we can send the commands through ROS topic:

Left arm joint0 position command(in rad):

```
$ rostopic pub /yigle/joint0_position_controller_left/command std_msgs/Float64 "data: 0.0" 
```

Neck joint1 position command(in rad):

```
$ rostopic pub /yigle/joint_position_controller_neck_1/command std_msgs/Float64 "data: 0.0" 
```

Drone velocity commands:

vx topic: /RH_joy_y, unit: 0.1*m/s

vy topic: /RH_joy_x, unit: 0.1*m/s

vz topic: /LH_joy_y, unit: 0.1*m/s

yawrate topic: /LH_joy_x, unit: 0.1*rad/s

i.e. let vz be 0.1m/s:

```
rostopic pub /LH_joy_y std_msgs/Float64 "data: 1.0" 
```

