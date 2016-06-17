# xamla_egomo

### What is Xamla Egomo

Egomo is an open-source research sensor head developed at [Xamla](http://www.xamla.com/) for adaptive robotics research on a Universal Robot UR5 equipped with Robotiq end-of-arm technologie. It acts as a low-cost, 3D-printable reference smart-sensor device for our [ROSvita](http://www.xamla.com/rosvita/) IDE (to be released in 2017).
For more information and product pictures visit the [egomo website](http://xamla.com/egomo).

### Unique Features of Xamla Egomo

* 2D+3D: full featured sensor head for RGB-D scans (RGB+Depth side cams) and visual servoing tasks (front-cam)
* NO CABELS: wireless connection of sensors and gripper!
* ROS: all sensors + actuators exposed as ROS nodes
* EASY: Lua client library to move robot and capture images + examples
* sensor-board with additional IO ports for laser, high-power IR/white LEDs or simple grippers or tactive sensors
* two side- and front view Logitech C920 cameras in combinaton with a structure IO depth sensor
* option to directy connect a Robotiq 2-Finger-Gripper-85 + Robotiq FT-300 sensor 

### Xamla Egomo Sensorhead ROS Package. ###

The Egomo sensorhead is a wireless based head for adaptive robotics. It is equipped with a [Logitech C920 HD](http://www.logitech.com/en-us/product/hd-pro-webcam-c920) Camera, a [Structure IO](http://structure.io/) Depth-Camera and an IR-Line-Laser. In combination with an [UR5](http://www.universal-robots.com/products/ur5-robot/) Robot equipped wih a [Robotiq 2-finger-85](http://robotiq.com/products/adaptive-robot-gripper/universal-robots-bundle/) gripper and an optionally [Robotiq FT-300](http://robotiq.com/force-torque-sensor-kit-universal-robots/) force-torque sensor you are able to solve robotic tasks with free path planning and no cables. 

This package is based on [LUA](https://www.lua.org/) with [Torch](https://github.com/torch) and the wrappers
[torch-ros](https://github.com/Xamla/torch-ros),  [torch-pcl](https://github.com/Xamla/torch-pcl)  and [torch-opencv](https://github.com/VisionLabs/torch-opencv).

This packages contains the following:

  * [xamla_egomo](https://github.com/Xamla/xamla_egomo/tree/master/xamla_egomo) Metapackages
  * [egomo_msgs](https://github.com/Xamla/xamla_egomo/tree/master/egomo_msgs): Messages, Services, Actions in order to talk to Gripper, Force-Torque Sensor, RGB-Camera, Depthcamera, Laser, LED
  * [egomo_node](https://github.com/Xamla/xamla_egomo/tree/master/egomo_node): LUA node which is responsible for the communication with any parts connected to the IO-Board (e.g. Gripper, Force-Torque, LED)
  * [egomo_ur5](https://github.com/Xamla/xamla_egomo/tree/master/egomo_ur5): Contains URDF Robot description of the UR5 equipped with Egomo sensorhead as well as launches for e.g. gazebo.
  * [egomo_webcam](https://github.com/Xamla/xamla_egomo/egomo_webcam): Contains ros-node for the Logitech C920 Webcam.
  * [egomo_depthcam](https://github.com/Xamla/xamla_egomo/egomo_depthcam): Contains ros-node for the Strucure-IO sensor.
  * [egomo_3d_printed_parts](https://github.com/Xamla/xamla_egomo/tree/master/egomo_3d_printed_parts): CAD files for printing the case of the Egomo sensorhead.
  * [egomo_demos](https://github.com/Xamla/xamla_egomo/egomo_demos): Demos based on LUA (like pick and place objects).
  * [egomo_tools](https://github.com/Xamla/xamla_egomo/egomo_tools): Libraries offering easy to use interfaces for e.g. Gripper control.
  * [egomo_pi_image](https://github.com/Xamla/xamla_egomo/egomo_pi_image): Link and description of the Raspberry PI containing everything to need with an read-only file system.

### Calibration ###

We do offer an implemented calibration routine on [request](http://xamla.com/en/contact/index.html) as well as CAD-Files when you would like to adapt the sensor head for another robot. In 2017 we plan to offer you [ROSVITA](http://xamla.com/en/rosvita/index.html) which brings a lot of featues such a professional calibration.

## Get an Egomo device the easy way

You like the idea of open-source hardware but the process of buying parts, 3D printing, soldering etc. is too cumbersome for you and you would like to get a tested/working sensor head quickly? No problem for EUR 1499 + shipping we send you pre-assembled version of the Egomo sensorhead with all parts included. If you are interested in this option please contact us via E-mail. 

### Contact ###

If you have any troubles please let us know by creating an Issue. Do not hesitate to [contact](http://xamla.com/en/contact/index.html) us for questions, feedback and
suggestions. We look forward to hear from you!
