*

This package provides an ROS interface to the Logitech C920 web camera. It uses video4linux for communication and therefore should work with other cameras supported by video4linux, too.


** Advertised Services

*** get\_new\_image
Grabs a new image from the camera. It is guaranteed that the image is recorded after this service is called. The image is provided in the camera raw format (uncompressed YUYV image in case of C920) and returned as a ROS [sensor_msgs/Image.msg](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html) message.
