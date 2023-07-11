# motoman_ros1

## Pre-requisites

- **mongo c++ drivers**
  - Follow instructions from https://github.com/ros-planning/warehouse_ros_mongo
  - You might have to change python3 to python2 to use scons
    - edit line 1 of scons. use `which scons` to locate it
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
