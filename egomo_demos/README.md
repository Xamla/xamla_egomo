# Demos for egomo sensor
This directory contains two demos that use the egomo sensor head.
- Clear Table
- Adaptive Duplo Stacking

### Clear Table
The robot identifies objects located on a table which it automatically collects and drops into a box ([Video](https://youtu.be/jR7Gnce9xus)).

### Adaptive Duplo Stacking
The robot detects freely positioned Duplo bricks on a table and stacks them on a visually measured target position ([Video](https://youtu.be/uo04vuXcwlw)).

## Calibration
These demos require very precise information about the geometric properties of the cameras ([intrinsic parameters](https://en.wikipedia.org/wiki/Camera_resectioning#Intrinsic_parameters)) and their mounting positions+orientations with respect to the tool center point (TCP) (hand-to-eye matrices). For an accurately working system these parameters have to be calibrated individually for each camera / egomo sensor. To run the demo without this complex calibration step, we added default values. Without proper calibration you should be able to see the basic behavior but not as precise as it is shown in the demo videos. 

A fully autonomous calibration procedure is in development. If you are interested to get the status or a preview version of the calibration that makes your UR5 + Egomo really accurate, please [contact us](mailto:egomo@xamla.com).

