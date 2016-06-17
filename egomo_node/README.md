# Description xamla_egomo_node#

## General Information ##

This node is supposed to be executed on the Raspberry Pi and offers a ros-service for sending commands to the gripper. Furthermore different topics are offered in order to get information about the gripper, force torque (FT) and inertial measurement unit (IMU).

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

### Topic XamlaForceTorque ###

Gives information about the force torque sensor using [WrenchStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/WrenchStamped.html). Subscribe by typing the following in your terminal: 

    rostopic echo /XamlaEgomo/XamlaForceTorque

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
wrench   | [geometry_msgs/Wrench](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Wrench.html)

### Topic XamlaIOIMU ###

Gives information about the inertial measurement unit (IMU) using [AccelStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/AccelStamped.html). Subscribe by typing the following in your terminal: 

    rostopic echo /XamlaEgomo/XamlaIOIMU

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
accel   | [geometry_msgs/Accel](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Accel.html)

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
    
### Action EgomoGripperActivate ###

This action can be used to activate/deactivate the gripper ([EgomoGripperActivate.action](https://github.com/Xamla/xamla_egomo/blob/master/egomo_msgs/action/EgomoGripperActivate.action)). It gives feedback about whether the gripper has been activated.
    
## Troubleshooting FAQ

 - Q1: I get the error "No device could be found!!!".
 - A1: The IO-Board connected the the Raspberry Pi could not be found. Make sure that it is connected properly. Usually the IO-Board should be recognized by the Raspberry Pi as a device /dev/ttyACM0 or any other number.
