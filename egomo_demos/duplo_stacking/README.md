# Duplo Stacking Demo

## What is done?
This demo shows how to stack Duplo bricks that are placed randomly on a table. (A video of this demo is available on youtube https://youtu.be/uo04vuXcwlw ). The robot searchs in a predefined area for a target brick that has to be mounted on a Duplo base plate. If the target has been found, the robot moves to a predefined overview pose, acquires a depth image, makes a simple segmentation (object that is located above a plane), moves to the duplo brick, acquires two rgb images and triangualtes the center position of the brick. Then the robot grips the brick and moves to the target position and stacks the brick.

## Assumptions
- The tables normal where bricks are located is (0,0,-1) (in robot base coordinates)
- Bricks are standard duplo bricks (2x4 height 19mm)
- hand-eye of depth sensor and rgb camera and their intrinsics are known
- The setup is: one brick on a base-plate as target position at a roughly known position
- bricks to be stacked are located on a flat surface 

## Package Requirements
- torch-ros
- torch-opencv
- torch-pcl
- egomo_tools

## Code
The code consists of three parts
- robot movement and gripping operations (adaptiveStackingDuplo.lua)
- detection of duplo bricks (duploDetection.lua)
- segmentation of depth image (ObjectsOnPlaneSegmentation.lua)


### duploDetection.lua
Idea: The knobs of the Duplo are detected in each image individually by a opencv Hough Circle detection, then circles are matched using the known fundamental matrix between the images, and triangulated. To avoid matching errors an initial guess of the z-Value (in robot coordinates) of the top edge is provided by the user. Finally, the orientation and the center of the duplo is calculated

### adaptiveStackingDuplo.lua
Idea: (Workflow can be found in main function)
- Robot is moved to a predefined target position (positions are defined in CreatePoses() ), 
- the target is scanned (ScanTargetPosition) 
- The robot moves to an overview pose and grabs an image
- Then objects are segmented laying on the table (segmentation:process(....))
- Robot moves wo the Duplo such that the camera is located above the Duplo (ScanDuplo)
- If the Duplo is found the robot moves to a pre-pick position
- Then it gripps it
- Moves to target position (pre-Place position and then place postion)
- Opens the gripper
- Then the whole process is repeated





