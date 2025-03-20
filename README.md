# ROS Gremsy Gimbal Interface
A ROS interface to control Gremsy gimbals. Based on the [gSDK_V3_alpha](https://github.com/Gremsy/gSDK/tree/gSDK_V3_alpha) interface and the MavLink protocol.

> Disclaimer: This software package is not officially developed by or related to Gremsy.


## Description
This package includes a ROS Node which warps the gSDK for the Gremsy Gimbals which are mainly used for physical image stabilization.

The gimbal is connected via UART with a Linux host device running this node.
Devices such as the Raspberry Pi feature a built-in UART interface others like most PCs or Laptops need a cheap USB Adapter.
The `config.yaml` file allows you to configure the serial device used and many other gimbal-specific parameters.
The node publishes the gimbal's encoder positions, imu measurements, and the camera mount orientation.

## Setup
Run the following commands to clone this repository and update all submodules (needed for the external gSDK repository).

We use `gitman` to handle the submodules of this repository.
The list of submodules is given in the `.gitman.yaml` file.

```sh
git clone https://github.com/fly4future/ros1_gremsy.git
cd ros1_gremsy
git pull && gitman update
```
to update all the submodules (default: will update all repositories to their predefined branches).

Now you need to install all dependencies using rosdep. To execute this command make sure that the correct catkin workspace is sourced and the repository you just cloned is (linked) inside the `src` directory.
```sh
rosdep install --from-paths . --ignore-src -r -y
```

After installing the dependencies you should be able to build the package using:
```sh
catkin build
```

## Launching
Type the following command to run the node. Make sure that the gimbal is connected properly, the Linux permissions regarding the serial interface are correct (this depends on your distro) and the config features the correct device and baudrate.

Nodelet:
```sh
roslaunch ros1_gremsy gremsy_nodelet.launch
```
## ROS API
## Coordinate System

### Earth Frame
The earth frame convention used is in ENU coordinates. (X: East, Y: North Z: Up).

### Body Frame
The body frame has the **X-axis** pointing forward the UAV heading, the **Y-axis** pointing to the left, and the **Z-axis** pointing up.

### Work Modes
When using Angle Mode, the gimbal moves to the target attitude in **earth frame** if operating in Lock mode. Otherwise, the gimbal moves to the target attitude in the Body frame if operating in Follow mode.

<p align="center">
    <img src="https://github.com/user-attachments/assets/2af6ffc1-1f3a-4776-8f89-32aeaa99a41c" width="400px">
</p>

### Messages:
The node publishes:
- `/diagnostics`:
  - **setpoint**: with a [geometry_msgs/Vector3](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html) message with current gimbal goal.
  - **encoder_values**: with a [geometry_msgs/Vector3](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html) message containing the encode values around the x (roll), y (pitch) and z (yaw) axis.
  - **attitude_quaternion**:  with a [geometry_msgs/Quaternion](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Quaternion.html) message representing the camera mount orientation in Quaternion representation, in the global frame except for the yaw axis which is provided relative to the gimbals mount on the vehicle or robot.
  - **attitude_euler**:  with a [geometry_msgs/Vector3
](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3.html) message representing the camera mount orientation using Euler Angles,  in the global frame except for the yaw axis which is provided relative to the gimbals mount on the vehicle or robot.
- `Transformations`:
  - **gimbal_link**: Gimbal link relative to `fcu_frame`. Static TF configured within the `gremsy_nodelet.launch` file.
  All the following transformations need to be configured in the `config.yaml` under the section "frames".
  
  Dynamic TF's based on gimbal attitude:
  - **gimbal/yaw**: Dynamic TF relative to the `gimbal_link` frame.
  - **gimbal/roll**: Dynamic TF relative to the `gimbal/yaw` frame.
  - **gimbal/pitch**: Dynamic TF relative to the `gimbal/roll` frame.
### Services:
- `/set_gimbal_attitude` expects a message containing the desired angles for each axis: roll, pitch, yaw (float64).
- `/goals` expects a [geometry_msgs/Vector3Stamped](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html) message containing the desired angles for each axis. The frame for each axis (local or global), as well as the stabilization mode, can be configured in the `config.yaml` file.
- `/set_gimbal_mode` expects a string message for the desired operation mode (follow/lock).




