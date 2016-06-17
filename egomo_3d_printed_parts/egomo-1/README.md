# STL files for 3D printing of Egomo-1

## Product Pictures

We took some photos to give you an impression how the finished sensor head looks when it is mounted on a UR5.

![Egomo left view](images/egomo_left.jpg)
![Egomo front view](images/egomo_front.jpg)
![Egomo right view](images/egomo_right.jpg)
![Egomo back view](images/egomo_back.jpg)
![Egomo joint cap](images/egomo_joint_cap.jpg)

## Multi-use Options

There are many ways to use the Egomo 3D models for your robotic research experiments:

Here are some ideas:

1. Obviously you can build the fully featured Egomo-1 sensor, optionally with or without enclosure for the Raspberry Pi or front-camarea mount.
2. Just use the coupling + base plate as a camera mount (conventionally cabled).
3. Use the RPi+IO-Board enclosure stand-alone as a remote ROS sensor-node or smart camera (e.g. to monitor the roboter scene from a fixed RGB-D camera). 

## Parts

The 3D printing file set of the Egomo-1 sensor consinsts of the following parts:

- coupling: [xamla_mount_flange.stl](xamla_mount_flange.stl)
- base-plate: [xamla_mount_base.stl](xamla_mount_base.stl)
- custom joint cap: [xamla_cap_hole.stl](xamla_cap_hole.stl)
- C920 side mount: [xamla_mount_c920_left.stl](xamla_mount_c920_left.stl), [xamla_mount_c920_right.stl](xamla_mount_c920_right.stl)
- C920 front mount: [xamla_mount_c920_front_0_left.stl](xamla_mount_c920_front_0_left.stl), [xamla_mount_c920_front_0_right.stl](xamla_mount_c920_front_0_right.stl) 
- Line-Laser mount: [xamla_mount_laser_bottom.stl](xamla_mount_laser_bottom.stl), [xamla_mount_laser_top.stl](xamla_mount_laser_top.stl)
- Raspberry Pi + IO Board enclosure: [xamla_mount_rpi_bottom.stl](xamla_mount_rpi_bottom.stl), [xamla_mount_rpi_middle.stl](xamla_mount_rpi_middle.stl),  [xamla_mount_rpi_top.stl](xamla_mount_rpi_top.stl)

Additionally we provide the file [xamla_cap.stl](xamla_cap.stl) which is a simple version of our modified joint cap without a hole for the power cable. It might be useful if you just want to fix a kinematic problem (collision) of the original UR5 joint cap that prevents the robot from freely rotating in all situations.

## Adaption to other Robots

If you are a robotics researcher and you want to customize the Egomo 3D parts to your special requirements or adapt it to a different robot we can provide you the original CAD files on request. Please get in touch with us via [E-Mail](mailto:egomo@xamla.com).


