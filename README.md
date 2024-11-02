
## Pre-requisites

1. A Yaskawa Robot
2. Motoman ROS server. Follow instructions here: http://wiki.ros.org/motoman_driver/Tutorials/indigo/InstallServer

> Note: The repository is setup and tested for GP4 robot. Other Yaskawa robots will require necessary support and moveit_config packages, which can be pulled from here: https://github.com/ros-industrial/motoman.

## Installation Option 1 - Dockerfile

**Pre-requisite**

```shell
# Check if docker is installed
$ docker version
Client: Docker Engine - Community
 Version:           20.10.7
...

# Check if docker is running
$ systemctl is-active docker
active
```

* If installation is needed, install docker enginer from here: \
[https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)
* If docker is not running, followin troubleshooting steps: [https://docs.docker.com/engine/daemon/troubleshoot/](https://docs.docker.com/engine/daemon/troubleshoot/)

**Build and Run**

```shell
docker compose up --build
```

The above command will build and instantiate the built image as a container `motoros-container`

Use following in another terminal to get run ROS commands in the container. 

```shell
xhost +
docker exec -it motoros-container bash
```

> Note: Please change ROS_MASTER_URI in `compose.yml`. If not running roscore on host, comment out the line.

> Note: Use `xhost +` only if you intend to run RViz from the docker. 

## Installation Option 2 - Build from Source

### Pre-requisites

- **mongo c++ drivers**
  - Follow instructions from https://github.com/ros-planning/warehouse_ros_mongo
  - You might have to change python3 to python2 to use scons
  - edit line 1 of scons. use `which scons` to locate it
```shell
# First get the driver:

git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git

# Then compile using scons:

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

### Installation

- Use `catkin build` to build the package

## ROS Launch Command

`moveit.launch`:  Connect with a robot. Also starts moveit interface and yk_tasks ros service interface. More info on yk_tasks available [here](https://github.com/cmu-mfi/motoman_ros1/blob/master/yk_tasks/README.md)
```
roslaunch yk_launch moveit.launch namespace:=<namespace> sim:=<true/false>
```
* `sim`: Default value is `false`. If want to run moveit simulation, specify `true`.
* `namespace`: As the name suggests, it launches a robot nodes on specified namespace. If *sim:=false*, then namespace value need to be defined in *yk_launch/launch/moveit.launch*. Definition includes robot ip address and corresponding controller file in the directory *yk_launch/config*

> Note: When running with a real robot, ensure that you change the ip addresses in `yk_launch/launch/moveit.launch` file.

## Important Assets

### **Support package from motoman**
ROS support packages are [provided by motoman](https://github.com/ros-industrial/motoman) for each robot. Below are the links for GP4

- [URDF files](https://github.com/cmu-mfi/motoman_ros1/tree/master/depend-packages/motoman/motoman_gp4_support/urdf)
- [Mesh files](https://github.com/cmu-mfi/motoman_ros1/tree/master/depend-packages/motoman/motoman_gp4_support/meshes/gp4)

Tutorials and documentation for ROS1 from Yaskawa available here: [Motoman Wiki](http://wiki.ros.org/motoman)

### **GP4 MoveIt Configuration**
The repo consists moveit configuration files for GP4 in the `motoman_gp4_moveit_config` package. It depends on `motoman_gp4_support` package.

- Execute RViz simulation
```shell
roslaunch motoman_gp4_moveit_config moveit_planning_execution sim:=true
```
- Execute MoveIt for real robot (replace the IP with your robot IP):
```shell
roslaunch motoman_gp4_moveit_config moveit_planning_execution sim:=false robot_ip:=192.168.1.7
```
Then in another terminal execute below to be able to send motion commands.
```shell
rosservice call /robot_enable
```
>  *`motoman_gp4_support` package [here](https://github.com/cmu-mfi/motoman_ros1/tree/master/depend-packages/motoman/motoman_gp4_support) was modified to not pass `controller` argument. [Line added](https://github.com/cmu-mfi/motoman_ros1/blob/741ad854da63d73dff111be450eabcccc8984c65/depend-packages/motoman/motoman_gp4_support/launch/robot_interface_streaming_gp4.launch#L14) to `robot_interface_streaming_gp4.launch`*


