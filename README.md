[**Go to MFI Main Page**](https://github.com/cmu-mfi/)

## Important Assets

#### **Support package from motoman**
ROS support packages are [provided by motoman](https://github.com/ros-industrial/motoman) for each robot. Below are the links for GP4

- [URDF files](https://github.com/cmu-mfi/motoman_ros1/tree/master/depend-packages/motoman/motoman_gp4_support/urdf)
- [Mesh files](https://github.com/cmu-mfi/motoman_ros1/tree/master/depend-packages/motoman/motoman_gp4_support/meshes/gp4)

Tutorials and documentation for ROS1 from Yaskawa available here: [Motoman Wiki](http://wiki.ros.org/motoman)

#### **GP4 MoveIt Configuration**
The repo consists moveit configuration files for GP4 in the `motoman_gp4_moveit_config` package. It depends on `motoman_gp4_support` package.

- Execute RViz simulation: `roslaunch motoman_gp4_moveit_config moveit_planning_execution sim:=true`
- Execute MoveIt for real robot: `roslaunch motoman_gp4_moveit_config moveit_planning_execution sim:=false robot_ip:=192.168.1.7`

  *`motoman_gp4_support` package [here](https://github.com/cmu-mfi/motoman_ros1/tree/master/depend-packages/motoman/motoman_gp4_support) was modified to not pass `controller` argument. [Line added](https://github.com/cmu-mfi/motoman_ros1/blob/741ad854da63d73dff111be450eabcccc8984c65/depend-packages/motoman/motoman_gp4_support/launch/robot_interface_streaming_gp4.launch#L14) to `robot_interface_streaming_gp4.launch`*

- Execute MoveIt in a namespace: Launch files in `lego_moveit` can be used as examples. [`lego_moveit_A.launch`](https://github.com/cmu-mfi/motoman_ros1/blob/master/lego_moveit/launch/lego_moveit_A.launch) [`lego_moveit_sim.launch`](https://github.com/cmu-mfi/motoman_ros1/blob/master/lego_moveit/launch/lego_moveit_sim.launch)


## Pre-requisites

- **mongo c++ drivers**
  - Follow instructions from https://github.com/ros-planning/warehouse_ros_mongo
  - You might have to change python3 to python2 to use scons
  - edit line 1 of scons. use `which scons` to locate it
```
First get the driver:

git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git

Then compile using scons:

sudo apt-get install scons
cd mongo-cxx-driver
sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors
```

- **eigenpy**
    - dependency for moveit 
    - `sudo apt-get install ros-noetic-eigenpy`
    
- **moveit**
  - Follow instruction from https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html
```
wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials  # this is cloned in the next section
wstool update -t .
```

## Installation

- Use `catkin build` to build the package


[**Go to MFI Main Page**](https://github.com/cmu-mfi/)
