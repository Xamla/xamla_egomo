require 'torch'
require 'sys'
local ros = require 'ros'
local tf = ros.tf
local pcl = require 'pcl'
local egomoTools = require 'egomo-tools'
local cv = require 'cv'
require 'cv.highgui'
require 'cv.imgproc'

require 'ObjectsOnPlaneSegmentation'
local duplo = require 'duploDetection'


local velocity_scaling = 1.0

-- Roboter and gripper control + initialisation
local roboControl = egomoTools.robot:new("duploDetect", velocity_scaling)
local gripperControl = egomoTools.gripper:new()
gripperControl:Connect()
sys.sleep(0.6)
gripperControl:ResetGripper()
gripperControl:OpenGripper()
gripperControl:SetGripForce(100)

-- StructureIO and Webcam control + initialisation
local camIntrinsicsIR=torch.DoubleTensor({
   {  563,    0,  322 },
   {    0,  562,  240 },
   {    0,    0,    1 }})
local heyeDepthcam = torch.DoubleTensor ({
      {  0.0001, -0.7759,  0.6308,  0.0458 },
      {  0.0005,  0.6308,  0.7759,  0.0580 },
      { -1.0000,  0.0002,  0.0005,  0.0806 },
      {  0.0000,  0.0000,  0.0000,  1.0000 }})


local heyeWebcam = duplo.handEye:clone()  -- select active handEye matrix..
heyeWebcam[{{1,3},{4}}] = heyeWebcam[{{1,3},{4}}] / 1000.0


local depthcam=egomoTools.structureio:new(camIntrinsicsIR)
depthcam:Connect()
local webcam = egomoTools.webcam:new("web_cam")
webcam:ConnectDefault()


local ps = roboControl:GetPlanningSceneInterface()
local rosMoveGroup = roboControl.rosMoveGroup --roboControl:GetMoveGroup()

local GRIPPER_LENGTH = 0.231


function moveCartPathPlanner(pose,p,stepsize)
   local currentStartPose=rosMoveGroup:getCurrentPose()

  local startEndorientation={}
  local startEndposition={}
   startEndorientation[1] = currentStartPose:getRotation()
   startEndorientation[2] = pose:getRotation()
   startEndposition[1] = currentStartPose:getOrigin()
   startEndposition[2] = pose:getOrigin()

   return rosMoveGroup:computeCartesianPath_Tensor(startEndposition, startEndorientation, stepsize, 10.0, false)
end


function move_to(pose)
  rosMoveGroup:getCurrentState()
  rosMoveGroup:setStartStateToCurrentState()
  rosMoveGroup:setPoseTarget(pose:toTensor())

  local s, p = rosMoveGroup:plan()
  --f, p = moveCartPathPlanner(pose,p)

  if s == 0 then
    return false
  end

  rosMoveGroup:execute(p)
  rosMoveGroup:clearPathConstraints()
  --  rosMoveGroup:move()
  return true
end


function move_to_cart(pose)
  rosMoveGroup:getCurrentState()
  rosMoveGroup:setStartStateToCurrentState()
  rosMoveGroup:setPoseTarget(pose)

  local s, p = rosMoveGroup:plan()
  i=0;
  local mystepsize= 0.01
  while i<3 do
    f, p = moveCartPathPlanner(pose,p,mystepsize)
    if f == 1.0 then
      break;
    else

      mystepsize = mystepsize/10
    end
    i= i+1;
  end

  if s == 0 and f < 1.0 then
    rosMoveGroup:clearPathConstraints()
    return false
  end

  rosMoveGroup:execute(p)
  rosMoveGroup:clearPathConstraints()
  --  rosMoveGroup:move()
  return true
end


function ScanDuplo(targetPoint, expectedDuploZ)
  local distance=0.15 --to target point

  imageCounter = 0
  robotPosesImages = {}

  local polarAngle = {}
  local azimuthalAngle = {}
  local upInverse = {}
  local targetPoints = {}
  local robotPoseLog={}
  
  -- Two positions that are close together with a small baseline are required to 
  -- do stereo. Here we decided to move the second camera by 1 cm from the first
  polarAngle[1]=-45
  azimuthalAngle[1]=0.1
  upInverse[1]=false
  targetPoints[1] = targetPoint:clone()

  polarAngle[2]=-45
  azimuthalAngle[2]=-0.1
  upInverse[2]=true
  targetPoints[2] = targetPoint:clone()
  targetPoints[2][2] = targetPoints[2][2] + 0.015 -- 1cm offset from first camera

  -- reset the duplo detection module
   duplo.reset()

   -- because we are so close to the brick, we have to set the focus of the camera
   webcam:SetFocusValue(30)
   local lastImage
   -- Lets take now two images of the brick
   for count=1, 2 do
    movePose = roboControl:WebCamLookAt(targetPoints[count], distance, math.rad(polarAngle[count]), math.rad(azimuthalAngle[count]),
					heyeWebcam, upInverse[count])
    movePose[3][4]=movePose[3][4]+0.002*count -- add additional 2 mm to each consecutive pose to avoid coplanar camera poses
    print("Move to Pose")
    print(movePose)

    roboControl:MoveRobotTo(movePose)
    sys.sleep(0.2)  -- wait for controller convergence
    local img=webcam:GrabGrayscaleImgROS() -- Take image
    if img == nil then
      print("Image not captured!")      
    end

    --cv.imshow{"webcam capture result",img}
    --cv.waitKey{-1}
 
    robotPoseLog[count] = roboControl:ReadRobotPose() -- Save the robot pose
    imageCounter = imageCounter+1

    found = duplo.addImage(img, robotPoseLog[count]) -- Add the image to the duplo detection module
    if not found and count == 2 then
      print('duplo not found! :(')
      return nil
    end
  end

  -- Get the orientation and position from the duplo detection modul
  local success, orientation, center = duplo.getDuploOrientation(expectedDuploZ, 4)
  if not success then
    print(orientation)
    print('orientation fail!')
    return nil
  else
    print("Found Orientation and Center!")
    print("Orientation")
    print(orientation)
    print("Center")
    print(center)
  end

  orientation = orientation * -1 -- HACK HACK HACK (TODO replace this with correct rotation matrix mult)  
  local pose = torch.eye(4,4)
  pose[{{1,3},{1,3}}] = orientation
  pose[{{1,3},{4}}] = center:view(3,1) / 1000.0 -- we need m instead of mm as unit to move the robot
  return pose, center:view(3,1) / 1000.0
end


local segmentation = ObjectsOnPlaneSegmentation{
  distance_threshold = 0.02,
  crop_radius = 1,
  cluster_tolerance = 0.02,  -- 2cm
  min_cluster_size = 300,
  max_cluster_size = 25000
}


---
-- Transforms a point measured in the depth cams coordinate system to the robot base coordinates
-- @param points a 3xn torch.Tensor of 3d points
-- @param robotPose 4x4 torch.Tensor
-- @return 3xn torch.Tensor with transformed 3d points
local function DepthCamToWorld(points, robotPose)
  return (robotPose * heyeDepthcam * points:t()):t()
end


--- 
-- Create a list pre-defined (hardcoded) positions
-- This list comprises the pose where an overview (depth)image is acquired, the pose where the brick
-- has to be places (place) and a pose that has a small z offset w.r.t the place position. We call that 
-- pre-place position
-- @return a table with three poses (overview, place, prePlace)
function CreatePoses()
   local poseList = {}
   local poseTmp = tf.Transform()
   local rotTmp = poseTmp:getRotation()

   -- pose to view the whole scene
   --poseList.overview = roboControl:WebCamLookAt(torch.DoubleTensor({0.1, 0.50, 0}), 0.55, math.rad(-45), math.rad(0.5), heyeDepthcam)
   poseList.overview = roboControl:WebCamLookAt(torch.DoubleTensor({0.1, 0.2, 0.0}), 0.55, math.rad(-45), math.rad(20), heyeWebcam)

   -- base pose before moving down to stack the duplo
   poseTmp:setOrigin(torch.Tensor({-0.2545,0.508, 0.231}))
   poseTmp:setRotation(rotTmp:setRPY(0, math.rad(180), math.rad(-45)))
   poseList.prePlace = poseTmp:toTensor()

   -- base pose when stacking the duplo
   poseTmp:setOrigin(torch.Tensor({-0.2545,0.508, 0.186}))
   poseTmp:setRotation(rotTmp:setRPY(0, math.rad(180), math.rad(-45)))
   poseList.place = poseTmp:toTensor()

   return poseList
end


---
-- This function calculates the target position where the brick has to be placed
-- @param originalTarget rough pose where we want to search for the target brick
-- @param z value of the bricks upper edge in robot coordinates
function ScanTargetPosition(originalTarget, estimatedDuploHeightZm)

  assert(estimatedDuploHeightZm > -0.1 and estimatedDuploHeightZm < 1)
  local scanTargetPosition = originalTarget[{{1,3},4}]:clone()
  -- our destination where we want to look
  scanTargetPosition[3] = estimatedDuploHeightZm
  -- We now scan the target
  local orientation, center = ScanDuplo(scanTargetPosition, estimatedDuploHeightZm*1000) --*1000 means meter to mm conversion
  if orientation ~= nil then
    -- we make a 4x4 matrix given orientation and center
    local tmp = orientation:clone()
    tmp[3][4] = originalTarget[3][4]
    return tmp
  end
  return nil
end


function main()

  local upper_z_target = 0.003 -- The z value of the duplo that is located at the target position
  local upper_z_normal_brick = -0.008 --The z value of the duplo that is located at the target position


  local poseList = CreatePoses()

  local stackHeight = 2

  gripperControl:SetMoveSpeed(20)

  local targetPos
  while not targetPos do
    targetPos = ScanTargetPosition(poseList.place:clone(), upper_z_target)
    if targetPos ~= nil then
      poseList.place = targetPos:clone()
      poseList.prePlace = poseList.place:clone()
      poseList.prePlace[3][4] = poseList.prePlace[3][4] + 0.05
    else
      print("Duplo Target Pose not found - retrying")
    end
   end

   print("Z: "..poseList.place[3][4])
   local targetPos = ScanTargetPosition(poseList.place:clone(), upper_z_target)

   --local viewer = pcl.CloudViewer()
   for ii=1, 20 do
      print("Overview pose:")
      print(poseList.overview)
      roboControl:MoveRobotTo(poseList.overview)
      local cloud = depthcam:GrabPointCloud()
      --cloud:savePCDFile("cloud.pcd")

  --viewer:showCloud(cloud)
    print("segmenting")
      local segmentationResult, objectPoints, objectIndices = segmentation:process(cloud)
      if segmentationResult and segmentationResult.meanOfEachCluster:nElement() > 0 then

     segmentationResult.meanOfEachCluster = DepthCamToWorld(segmentationResult.meanOfEachCluster, rosMoveGroup:getCurrentPose():toTensor())
     print("segmenting done")
     print(segmentationResult.meanOfEachCluster)

      local scanPosition
      -- Search for a duplo candidate in the list of segmented objects.
      -- The duplo detection itself is very primitive
      for i = 1, segmentationResult.meanOfEachCluster:size()[1] do
        local point = segmentationResult.meanOfEachCluster[{i, {1,3}}]
        if (point[3] < upper_z_normal_brick + 0.02 and point[3] > upper_z_normal_brick - 0.02) then --typically center of a duplo is below the table
          scanPosition = point
          break
        end
      end

      local duploPose
      if scanPosition == nil or torch.norm(scanPosition) == 0 then
        print("No detailed scan position found!")
      else
        scanPosition[3] = 0.0
        local counter=1
        while duploPose == nil and counter< 5 do
          duploPose=ScanDuplo(scanPosition,  upper_z_normal_brick * 1000.0) ---8 indicates that the top of the duplo is located at z = -8 mm
          counter = counter+1
          print("counter: "..counter)
        end
      end

      local posePrePick = duploPose:clone()
      posePrePick[3][4]=0.25 -- set z position to 25cm

      local posePick = duploPose:clone()
      posePick[3][4]=0.195

      local posePrePlace = poseList.prePlace:clone()
      posePrePlace[3][4] = posePrePlace[3][4] + (stackHeight * 0.019)  -- each duplo increases the place pose by 1.9 cm

      local posePlace = poseList.place:clone()
      posePlace[3][4] = posePlace[3][4] + (stackHeight * 0.019)  -- each duplo increases the place pose by 1.9 cm


      if roboControl:MoveRobotTo(posePrePick) then
      print("PosePick")
      print(posePick)
      print("posePrePlace")
      print(posePrePlace)
      print("posePlace")
      print(posePlace)

      function tryplacebrick()
        if not roboControl:MoveRobotTo(posePick) then return false end
        gripperControl:CloseGripper()
        sys.sleep(1.7)
        if not roboControl:MoveRobotTo(posePrePick) then return false end
        if not roboControl:MoveRobotTo(posePrePlace) then return false end
        sys.sleep(0.1)
--        roboControl.rosMoveGroup:setMaxVelocityScalingFactor(velocity_scaling*0.1)
        roboControl.rosMoveGroup:getCurrentState()

        local p2 = ros.tf.Transform()
        p2:fromTensor(posePlace)
        if not move_to_cart(p2) then return false end
        --if not roboControl:MoveRobotTo(posePlace) then return false end
        gripperControl:SetMoveSpeed(20)
        gripperControl:OpenGripper()
        sys.sleep(1.0)
        roboControl.rosMoveGroup:setMaxVelocityScalingFactor(velocity_scaling)
        roboControl.rosMoveGroup:getCurrentState()
        roboControl:MoveRobotTo(posePrePlace)
        return true
      end

      if tryplacebrick() then
        local newPlacePosition = ScanTargetPosition(posePlace, upper_z_target + (stackHeight-1)* 0.019)
        if newPlacePosition then

          local z = poseList.place[3][4]
	  poseList.place = newPlacePosition:clone()
          poseList.place[3][4] = z -- height of our stackes bricks
          poseList.prePlace = poseList.place:clone()
          poseList.prePlace[3][4] = poseList.prePlace[3][4]+0.05

	end
        stackHeight = stackHeight + 1

      else
        gripperControl:OpenGripper()
      end
     end
    end
   end
end

main()

ros.shutdown()
