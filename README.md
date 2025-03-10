# ROS Gremsy Gimbal Interface
A ROS interface to control Gremsy gimbals. Based on the [gSDK](https://github.com/Gremsy/gSDK) interface and the MavLink protocol.

Disclaimer: This software package is not officially developed by or related to Gremsy.

![Build CI](https://github.com/Flova/ros_gremsy/workflows/Build%20CI/badge.svg)

## Description
This package includes a ROS Node which warps the gSDK for the Gremsy Gimbals which are mainly used for physical image stabilization.

The gimbal is connected via UART with a Linux host device running this node.
Devices such as the Raspberry Pi feature a build-in UART interface others like most PCs or Laptops need a cheap USB Adapter.
The used serial device, as well as many other gimbal specific parameters, can be configured in the `config.yaml` file.
The node publishes the gimbals encoder positions, imu measurements, and the camera mount orientation.

## Setup
Run the following commands to clone this repository and update all submodules (needed for the external gSDK repository).

We use `gitman` to handle submodules of this repository.
The list of submodules is given in `.gitman.yml` file.

```
git clone https://github.com/fly4future/ros1_gremsy.git
cd ros1_gremsy
git pull && gitman update
```
to update all the submodules (default: will update all repositories to its predefined branches).

Now you need to install all dependencies using rosdep. To execute this command make sure that the correct catkin workspace is sourced and the repository you just cloned is (linked) inside the `src` directory.
```
rosdep install --from-paths . --ignore-src -r -y
```

After installing the dependencies you should be able to build the package using:
```
catkin build
```

## Launching
Type the following command to run the node. Make sure that the gimbal is connected properly, the Linux permissions regarding the serial interface are correct (this depends on your distro) and the config features the correct device and baudrate (default setting should be fine as far as I know).
Node:
```
roslaunch ros1_gremsy gimbal.launch
```
Nodelet:
```
roslaunch ros1_gremsy gremsy_nodelet.launch
```
## ROS Message API
The node publishes:
- `/ros1_gremsy/encoder` with a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the encode values around the x (roll), y (pitch) and z (yaw) axis.
- `/ros1_gremsy/gimbal_attitude_quaternion` with a [geometry_msgs/Quaternion](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html) message representing the camera mount orientation in Quaternion representation, in the global frame except for the yaw axis which is provided relative to the gimbals mount on the vehicle or robot.
- `/ros1_gremsy/gimbal_attitude_euler` with a [geometry_msgs/Vector3Stamped
](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message representing the camera mount orientation using Euler Angles,  in the global frame except for the yaw axis which is provided relative to the gimbals mount on the vehicle or robot.
- `/ros1_gremsy/set_gimbal_attitude` expects a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the desired angles for each axis. 
The node receives:
- `/ros1_gremsy/goals` expects a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the desired angles for each axis. The frame for each axis (local or global), as well as the stabilization mode, can be configured in the `config.yaml` file.

