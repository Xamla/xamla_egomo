# Description xamla_egomo_node#

## General Information ##

This node is supposed to be executed on the Raspberry Pi and offers a ros-service, actions for sending commands to the gripper. Furthermore different topics are offered in order to get information about the gripper, force torque (FT) and inertial measurement unit (IMU). Here an overview is given about:

 *  [`XamlaGripper`](#XamlaGripper): Gripper message
 *  [`XamlaGripperJointState`](#XamlaGripperJointState): Joint state of the gripper
 *  [`XamlaForceTorque`](#XamlaForceTorque): Force Torque message
 *  [`XamlaIOIMU`](#XamlaIOIMU): Message IMU
 *  [`SendCommand`](#SendCommand): Send commands to IO-Board
 *  [`EgomoGripperActivate`](#EgomoGripperActivate): Action for gripper activation
 *  [`EgomoGripperPos`](#EgomoGripperPos): Action for setting gripper position
 *  [`Troubleshooting`](#Troubleshooting): Troubleshooting

<a name="XamlaGripper"></a>
### Topic XamlaGripper ###

Gives information about the gripper. For publishing data an own message was created ([XamlaGripper.msg](https://github.com/Xamla/xamla_egomo/tree/master/egomo_msgs/msg)). Subscribe by typing the following in your terminal:

    rostopic echo /XamlaEgomo/XamlaGripper

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
grip_force | float32
left_finger_force | float32
right_finger_force | float32
pos_fb | float32
uint8 | state
bool | object_gripped

<a name="XamlaGripperJointState"></a>
### Topic XamlaGripperJointState ###

Gives information about the gripper meeting the joint state message specification ([JointState](http://docs.ros.org/jade/api/sensor_msgs/html/msg/JointState.html)). Subscribe by typing the following in your terminal:

    rostopic echo /joint_states

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
name     | string[]
position | float64[]
velocity | float64[]
effort | float64[]

<a name="XamlaForceTorque"></a>
### Topic XamlaForceTorque ###

Gives information about the force torque sensor using [WrenchStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/WrenchStamped.html). Subscribe by typing the following in your terminal: 

    rostopic echo /XamlaEgomo/XamlaForceTorque

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
wrench   | [geometry_msgs/Wrench](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Wrench.html)

<a name="XamlaIOIMU"></a>
### Topic XamlaIOIMU ###

Gives information about the inertial measurement unit (IMU) using [AccelStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/AccelStamped.html). Subscribe by typing the following in your terminal: 

    rostopic echo /XamlaEgomo/XamlaIOIMU

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
accel   | [geometry_msgs/Accel](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Accel.html)

<a name="SendCommand"></a>
### Service SendCommand ###

In order to send commands to the gripper this service was created ([SendCommand.srv](https://github.com/Xamla/xamla_egomo/tree/master/egomo_msgs/srv)). The following commands are valid:

Command | value range | description
---------|-----------|--------------
reset   | 0 or 1 | Resets the gripper when value is set from 1 -> 0 (falling edge)
max_speed | 0-255 | Controls the speed of the gripper
max_force | 0-255 | Controls the force of the gripper
pos_cmd | 0.0-0.087 | Moves the gripper to the position (in m)
out0 | 0 or 1 | Turns on/off device at this board output (e.g. led, ir-led ord laser)
out1 | 0 or 1 | Turns on/off device at this board output (e.g. led, ir-led ord laser)
out2 | 0 or 1 | Turns on/off device at this board output (e.g. led, ir-led ord laser)

Notice that you should reset the gripper before use. While initializing the gripper moves. Here are two examples of valid calls of the service from the terminal.

    rosservice call /XamlaEgomo/SendCommand "reset" 0
    rosservice call /XamlaEgomo/SendCommand "pos_cmd" 0.05
    rosservice call /XamlaEgomo/SendCommand "out0" 0

<a name="EgomoGripperActivate"></a>
### Action EgomoGripperActivate ###

This action can be used to activate/deactivate the gripper ([EgomoGripperActivate.action](https://github.com/Xamla/xamla_egomo/blob/master/egomo_msgs/action/EgomoGripperActivate.action)). It gives feedback about whether the gripper has been activated.

#### Goal ####

name | type | description
---------|-----------|---------------
activate   | bool | true to activate gripper - false to deactivate gripper

#### Feedback ####

name | type | description
---------|-----------|---------------
is_activated   | bool | true if gripper is activated - false if gripper is deactivated

#### Result ####

name | type | description
---------|-----------|---------------
is_activated   | bool | true if gripper is activated - false if gripper is deactivated

<a name="EgomoGripperPos"></a>
### Action EgomoGripperPos ###

This action can be used to send the gripper position commands ([EgomoGripperPos.action](https://github.com/Xamla/xamla_egomo/blob/master/egomo_msgs/action/EgomoGripperPos.action)). Furthermore the speed and the force can be set directly if needed. The feedback and the result contain the current position of the gripper and whether an object has been gripped.

#### Goal ####

name | type | description
---------|-----------|---------------
goal_pos   | float32 | The desired position in m (e.g. 0.0 for close or 0.087 for open)
set_speed_and_force | bool | If true the given speed and the force are set directly with the position command.
speed | uint8 | Speed of the gripper (0 is lowest, 255 is highest speed)
force | uint8 | Force of the gripper (0 is lowest, 255 is highest force)

#### Feedback ####

name | type | description
---------|-----------|---------------
pos_fb   | float32 | The current position of the gripper
object_gripped   | bool | True if an object has been gripped and false otherwise

#### Result ####

type | name | description
---------|-----------|---------------
pos_fb   | float32 | The current position of the gripper
object_gripped   | bool | True if an object has been gripped and false otherwise

<a name="Troubleshooting"></a>
## Troubleshooting FAQ

 - Q1: I get the error "No device could be found!!!".
 - A1: The IO-Board connected the the Raspberry Pi could not be found. Make sure that it is connected properly. Usually the IO-Board should be recognized by the Raspberry Pi as a device /dev/ttyACM0 or any other number.
