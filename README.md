# xamla_egomo

### What is Xamla Egomo?

Egomo is an open-source research vision sensor developed at [Xamla](http://www.xamla.com/) for adaptive robotics research on a [Universal Robot UR5](http://www.universal-robots.com/) equipped with [Robotiq](http://robotiq.com/) end-of-arm technology. It integrates consumer cameras like the Logitech C920 and Structure IO and allows you to mount them on a robotic arm. It acts as a low-cost, 3D-printable reference smart-sensor device for our [ROSVITA](http://www.xamla.com/rosvita/) robot programming IDE which is currently under development (to be released in 2017).
For more information and product pictures visit the [egomo website](http://xamla.com/egomo).  

Video: Watch our UR5 robot [stacking duplos](https://www.youtube.com/watch?v=uo04vuXcwlw) using the Egomo sensor (source code of the demo can be found [here](/egomo_demos/duplo_stacking)).

![sensorhead1](egomo_3d_printed_parts/egomo-1/images/egomo_left.jpg "Egomo sensor head side view, showing that the webcam can be mounten on the side as well as in the front.")

![sensorhead2](egomo_3d_printed_parts/egomo-1/images/egomo_front.jpg "Egomo sensor head front view")

### Unique Features of Xamla Egomo

* 2D+3D: two side- and front view Logitech C920 cameras in combinaton with a structure IO depth sensor, ideal for RGB-D scans and visual servoing experiments
* NO CABLES: wireless connection of all sensors and and the gripper (no cabled device offers more available joint-configuration space)!
* ROS: the functionalty of all sensors + actuators is exposed as ROS nodes
* EASY: A Lua client library is included which allows to capture images and move the robot to capturing poses + demos are provided
* The IO-board allows to directly connect a Robotiq 2-Finger-Gripper and a Robotiq FT-300 force torque sensor to Egomo. It also has IO ports for laser, high-power white/IR LEDs, simple grippers or tactile sensors (e.g. force-sensitive resistors).
* Egomo seamlessly integrates with the [Torch](http://torch.ch/) machine-learning framework (e.g. for deep learning based image segmentation, object detection or advanced RL-experiments).

### Xamla Egomo sensor head ROS Package ###

This package is based on [LUA](https://www.lua.org/) with [Torch](http://torch.ch/) and the wrappers
[torch-ros](https://github.com/Xamla/torch-ros),  [torch-pcl](https://github.com/Xamla/torch-pcl)  and [torch-opencv](https://github.com/VisionLabs/torch-opencv).

This packages comprises (in alphabetical order):

  * [egomo_3d_printed_parts](/egomo_3d_printed_parts): STL files for printing the mounting parts and case of the Egomo sensor.
  * [egomo_demos](/egomo_demos): Demos showing the sensor in action (like pick and place objects).
  * [egomo_depthcam](/egomo_depthcam): Contains a ros-node for the Strucure-IO depth sensor.
  * [egomo_ioboard](/egomo_ioboard): Contains the specs and pinout of the Xamla IO board.
  * [egomo_msgs](/egomo_msgs): ROS Messages, Services, Actions in order to talk to Gripper, Force-Torque Sensor, RGB-Camera, Depth-Camera, Laser, LED
  * [egomo_node](/egomo_node): A ros-node which is responsible for the communication to the IO-Board (e.g. Gripper, Force-Torque, LEDs, Laser).
  * [egomo_pi_image](/egomo_pi_image): Describes the steps required to download and setup our fast booting Raspberry PI linux image.
  * [egomo_tools](/egomo_tools): A client library offering an easy to use interfaces to gripper, structure IO, webcam and the robot itself.
  * [egomo_ur5](/egomo_ur5): Contains URDF Robot description of the UR5 equipped with Egomo sensor head as well as launch files for e.g. gazebo.
  * [egomo_webcam](/egomo_webcam): Contains ros-node for the Logitech C920 webcam.
  * [xamla_egomo](/xamla_egomo) Metapackage

### Calibration ###

We offer an implemented calibration routine [on request](http://xamla.com/en/contact/index.html) as well as CAD-Files when you would like to adapt the sensor head for another robot. In 2017 we will offer [ROSVITA](http://xamla.com/en/rosvita/index.html) which brings a lot of features such as a convenient calibration routine (including full robot calibration).

### Get a pre-assembled Egomo device

You like the idea of open-source hardware but the process of buying parts, 3D printing, soldering, building etc. is too cumbersome and time-intensive for you and you would like to get a tested/working sensor head quickly?
As a convenient solution we offer a pre-assembled version of the Egomo sensor head with all parts installed and tested (good quality 3D printed parts made from ABS Ultrat, Logitech C920 Web-Cam (2x), Structure IO sensor, white LEDs (3x), IR LED, Raspberry Pi 3, Xamla IO-Board, shortened USB cables, a customized RPi operating system on a 16 GB class 10 microSD card and a 20mW 830nm IR line laser (depending on your country's import restrictions)) for EUR 1499 + shipping. If you are interested in this offer please contact us via [E-mail](mailto:egomo@xamla.com).

### Support / Contact ###

If you have any trouble please let us know by [creating an issue](https://github.com/Xamla/xamla_egomo/issues/new). Do not hesitate to contact us via [E-mail](mailto:egomo@xamla.com) for questions, feedback and suggestions. We look forward to hear from you!

To stay always up-to-date about our latest developments you can follow [@xamla on Twitter](https://twitter.com/xamla) or [subscribe](http://xamla.com/en/egomo) to our newsletter.
