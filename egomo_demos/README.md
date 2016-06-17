# Demos for egomo sensor
This directory contains two demos that uses the egomo sensor head.
- Clear Table
- Adaptive Duplo Stacking

### Clear Table
The robot moves above the table, makes a 3d image using the depth sensor makes a segmentation to identify objects located on the table, grabs them and throws them into a box (https://youtu.be/jR7Gnce9xus)

### Adaptive Duplo Stacking
The robot detects Duplo bricks on a table and stacks them. (https://youtu.be/uo04vuXcwlw)

## Calibration
These demos require geometric information about the cameras internal parameters (known as intrinsic parameters) and their mounting position with respect to the Tool Center Point (TCP). To have an accurate operating system these parameters have to be calibrated for each camera / egomo sensor head individually. To run the demo without this complex calibration step, we added default values that fit for our egomo sensor. With these parameters you should be able to see the basic behavior but not as precise as it is shown in the demo videos. 

A fully autonomous calibration procedure is in development. If you are interested to get the status or a preview version of the calibration that makes your UR5 + Egomo really accurate, please contact us

