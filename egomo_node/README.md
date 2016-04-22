# Description xamla_egomo_node#

## General Information ##

This node is supposed to be executed on the Raspberry PI and offers a ros-service for sending commands to the gripper. Furthermore different topics are offered in order to get information about the gripper, force torque (FT) and inertial measurement unit (IMU).

### Topic XamlaGripper ###

Gives information about the gripper. For publishing data an own message was created ([XamlaGripper.msg](https://github.com/Xamla/xamla_egomo/tree/master/egomo_msgs/msg)). Subscribe by typing the following in your terminal:

    rostopic echo /XamlaGripper

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
grip_force | float32
left_finger_force | float32
right_finger_force | float32
pos_fb | float32

### Topic XamlaForceTorque ###

Gives information about the force torque sensor using [WrenchStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/WrenchStamped.html). Subscribe by typing the following in your terminal: 

    rostopic echo /XamlaForceTorque

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
wrench   | [geometry_msgs/Wrench](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Wrench.html)

### Topic XamlaIOIMU ###

Gives information about the inertial measurement unit (IMU) using [AccelStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/AccelStamped.html). Subscribe by typing the following in your terminal: 

    rostopic echo /XamlaIOIMU

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
accel   | [geometry_msgs/Accel](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Accel.html)

### Service SendGripperSetCommand ###

In order to send commands to the gripper this service was created ([SendGripperSetCommand.srv](https://github.com/Xamla/xamla_egomo/tree/master/egomo_msgs/srv)). The following commands are valid:

Command | value range | description
---------|-----------|--------------
reset   | 0 or 1 | Resets the gripper when value is set from 1 -> 0 (falling edge)
max_speed | 0-255 | Controls the speed of the gripper
max_force | 0-255 | Controls the force of the gripper
pos_cmd | 0-255 | Moves the gripper to the position

Notice that you should reset the gripper before use. While initializing the gripper moves. Here are two examples of valid calls of the service from the terminal.

    rosservice call /egomo_msgs/SendGripperSetCommand "reset" 0
    rosservice call /egomo_msgs/SendGripperSetCommand "pos_cmd" 127
