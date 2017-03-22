# Industrial Core

[![Build Status](http://build.ros.org/job/Idev__industrial_core__ubuntu_trusty_amd64/badge/icon)](http://build.ros.org/job/Idev__industrial_core__ubuntu_trusty_amd64)

[ROS-Industrial][] core meta-package. See the [ROS wiki][] page for more
information.

## Contents

Branch naming follows the ROS distribution they are compatible with. `-devel`
branches may be unstable. Releases are made from the distribution branches
(`hydro`, `indigo`, `jade`).

Older releases may be found in the old ROS-Industrial [subversion repository][].


[ROS-Industrial]: http://wiki.ros.org/Industrial
[ROS wiki]: http://wiki.ros.org/industrial_core
[subversion repository]: https://github.com/ros-industrial/swri-ros-pkg


For this master branch, user must implement the PVT algorithm for the pt_full_message.

Usage:
```
cd ~
mkdir -p m_catkin_ws/src && cd my_catkin_ws/src
catkin_init_workspace
git clone https://github.com/fengzhuye/industrial_core.git
git checkout master
cd .. && catkin_make
source devel/setup.sh
roslaunch industrial_robot_client robot_interface_streaming.launch robot_ip:=YOU_ROBOT_IP_ADDR 
```
And you should connect to robot controller server successfully. If error ocurred, make sure the robot server have implenented the robot state message and pt_full_message.

If no errors, then open another terminal:
```
source ~/my_catkin_ws/devel/setup.sh
roslaunch moveit_pkg demo.launch
```

For full moveit configuration, check the moveit_pkg package config and launch files.
