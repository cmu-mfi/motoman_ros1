# motoman_ros1

## Installation

- Use `catkin build` to build the package
- If `warehouse-ros-mongo`gives an error, install mongo c++ drivers
  - https://github.com/ros-planning/warehouse_ros_mongo
  - You might have to change python3 to python2 to use scons
    - edit line 1 of scons. use `which scons` to locate it
