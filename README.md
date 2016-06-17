# xamla_egomo
[Xamla Egomo](http://xamla.com/en/egomo/index.html) Sensorhead ROS Package. This package is based on [LUA](https://www.lua.org/) with [Torch](https://github.com/torch) and the wrappers
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
  * [egomo_pi_image](https://github.com/Xamla/xamla_egomo/egomo_pi_image): Image for the Raspberry PI containing everything to need with an read-only file system.

We do offer an implemented calibration routine on [request](http://xamla.com/en/contact/index.html) as well as CAD-Files when you would like to adapt the sensor head for another robot. In 2017 we plan to offer you [ROSVITA](http://xamla.com/en/rosvita/index.html) which brings a lot of featues such a professional calibration.

If you have any troubles please let us know by creating an Issue. Do not hesitate to [contact](http://xamla.com/en/contact/index.html) us for questions, feedback and
suggestions. We look forward to hear from you!
