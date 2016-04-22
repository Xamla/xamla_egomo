# Description xamla_egomo_node#

## General Information ##

This node is supposed to be executed on the Rasperri PI and offers a ros-service for sending commands to the gripper. Furtheremore different topics are offered in order to get information about the gripper, force torque (FT) and inertial measurement unit (IMU).

### Topic XamlaGripper ###

Gives information about the gripper. For example subscribe with "rostopic echo /XamlaGripper" typing into your terminal.
For publishing data an own message was created ([XamlaGripper.msg](https://github.com/Xamla/xamla_egomo/tree/master/egomo_msgs/msg))

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
grip_force | float32
left_finger_force | float32
right_finger_force | float32
pos_fb | float32

### Topic XamlaForceTorque ###

Gives information about the force torque sensor using [WrenchStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/WrenchStamped.html). For example subscribe with "rostopic echo /XamlaForceTorque" typing into your terminal.

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
wrench   | [geometry_msgs/Wrench](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Wrench.html)

### Topic XamlaIOIMU ###

Gives information about the inertial measurement unit (IMU) using [AccelStamped](http://docs.ros.org/jade/api/geometry_msgs/html/msg/AccelStamped.html). For example subscribe with "rostopic echo /XamlaIOIMU" typing into your terminal.

Published data | type
---------|-----------
header   | [std_msgs/header](http://docs.ros.org/jade/api/std_msgs/html/msg/Header.html)
accel   | [geometry_msgs/Accel](http://docs.ros.org/jade/api/geometry_msgs/html/msg/Accel.html)

### Service SendGripperSetCommand ###
