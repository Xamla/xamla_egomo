require 'torch'

local ros = require 'ros'
local pcl = require 'pcl'

local cv = require 'cv'
require 'cv.highgui'
require 'cv.imgproc'

ct = require 'cloud-tools'
require 'cloud-tools.objects_on_plane_segmentation'

local egomoTools = require 'egomo-tools'
local roboControl = egomoTools.robot:new("cleanTable", 1.0)
local ps = roboControl:GetPlanningSceneInterface()


local camIntrinsicsIR=torch.FloatTensor{
   {584.8203397188, 0, 322.13417604596},	
   {0, 587.09551694351, 242.17211588298},	
   {0, 0, 1}}	 -- 2016-05-04


local heye = torch.DoubleTensor({
{-0.0026, -0.7745,  0.6325,  0.0318},
{-0.0282,  0.6324,  0.7742,  0.0442},
{-0.9996, -0.0158, -0.0235,  0.0949},
{ 0.0000,  0.0000,  0.0000,  1.0000}})  -- 2016-06-13


local depthcam=egomoTools.structureio:new(camIntrinsicsIR, "egomo_depthcam")
depthcam:Connect()

local gripperControl = egomoTools.gripper:new()
gripperControl:Connect()
sys.sleep(0.2)
gripperControl:ResetGripper()
gripperControl:OpenGripper()
gripperControl:SetGripForce(50)

local tf = ros.tf

-- object segmentation
local segmentation = ObjectsOnPlaneSegmentation{
  cloudType = 'XYZ',
  distanceThreshold = 0.015,
  cropRadius = 1,
  clusterTolerance = 0.02,  -- 2cm
  minClusterSize = 300,
  maxClusterSize = 25000
}

local GRIPPER_LENGTH = 0.22
local APPROACH_DISTANCE = 0.08

local poses = {}

local fsmState="initializing"

--local cloudVis = pcl.PCLVisualizer('demo', true)
local currObj=nil  -- internal variable, used by VisualizeCloud



local function FindObjectsVirtual ()
  local N = 3
  local result = {}
  
  for i=1,N do
    -- generate a random position on table
    table.insert(result, {
      center = torch.Tensor({ 0.1 + math.random()*0.5, 0.1 + math.random() * 0.5, math.random() * 0.05, 1 }),
      main_axis = torch.Tensor({ math.random(), math.random(), math.random() })
    })
  end

  return result
end

--- 
-- Heuristic to find grippable objects
-- @param meanOfEachObject center point of the object found by the segmentation algorithm
-- @param firstEigenvectorOfEachObject the corrspnding largest eigenvector of the object, i.e. main axis
local function FindObjects(meanOfEachObject, firstEigenvectorOfEachObject)
  local result = {}
  local origin = torch.Tensor({0,0,0,1})
  local N = meanOfEachObject:size(1)
  for i=1,N do
    local mean = meanOfEachObject[i]
    local mainAxis = firstEigenvectorOfEachObject[i]
    if mean[3] >= -0.04 and mean[3] <= 0.05 and (mean - origin):norm() > 0.3 then
      table.insert(result, {
        center = mean,
        mainAxis = mainAxis,
	cloudIndex=i
      })
    end
  end
  
  return result
end


--- 
-- Creates the grip pose
-- @param target position of the object and the orientation of the object to grip
-- @param zdistance height above z plane where to grip
local function CreateApproachPose(target, zdistance)
  target.center[3]=0
  print("Target main axis:")
  print(target.mainAxis)
  print("Target center:")
  print(target.center)
  local mainAxis = target.mainAxis:clone()

  print('main axis:')
  print(mainAxis)
  
  local tcpPosition=target.center[{{1,3}}]:clone()
  tcpPosition[3] = tcpPosition[3]+zdistance+GRIPPER_LENGTH

  local pose=roboControl:PointAtPose(tcpPosition, target.center[{{1,3}}], torch.DoubleTensor({0,1,0}))
  local alpha = math.atan2(mainAxis[1], mainAxis[2])
  -- alpha = alpha+math.rad(90) -- no additional +math.rad(90) here because PointAtPose() provides an already rotated pose
  local rotPose = roboControl:RotateGripper(pose, alpha)
  print("pose for picking")
  print(rotPose)
  return rotPose 
end


--- 
-- Creates the grip pose
-- @param target position of the object and the orientation of the object to grip
-- @param zdistance height above z plane where to grip
local function CreateGripPose(target, zdistance)
   return CreateApproachPose(target, zdistance)
end

--- 
-- Sets the position for dropping the objects
-- @param orientation 4x4 torch.Tensor the orientation of the TCP when dropping positions
local function CreateDropPose(orientation)
   local dropPoseManual = orientation:clone()
   dropPoseManual[1][4]=0.55
   dropPoseManual[2][4]=0.05
   dropPoseManual[3][4]=0.35
   return dropPoseManual
end

--- 
-- The picking action
-- Robot moves to approach position, moves down and closes the gripper
local function PickObject()
   print("picking object, approachPose: ")
  print(poses.approache)

  roboControl:MoveRobotTo(poses.approache)
  
  -- descent 
  print("gripPose")
  print(poses.grip)
  roboControl:MoveRobotTo(poses.grip)
  
  gripperControl:CloseGripper()
  sys.sleep(0.15) 

  fsmState="pickDone"
end

--- 
-- Dropping the object
local function DropObject()
   print("moving and dropping object")

  -- return to approach position
  roboControl:MoveRobotTo(poses.approache)
  roboControl:MoveRobotTo(poses.drop)
   
  --drop
  gripperControl.OpenGripper(gripperControl)
  sys.sleep(0.05)
  fsmState="dropDone"
end

---
-- Moves the robot to the overview pose.
-- An additional colision object is added to prevent that the robot
-- moves one of its joints into the view frustrum of the camera
local function MoveToOverview()
   print("moveto overview")
   print(poses.overview)   
   local spherePose = tf.Transform()
   spherePose:setOrigin({0.2, 0.6, 0.1})
   spherePose:setRotation(tf.Quaternion({1, 0, 0}, math.pi))
   ps:addSphere('working_area', 0.3, spherePose)
   print("Pose Overview:")
   print(overviewPose)
   roboControl:MoveRobotTo(poses.overview)
   ps:removeCollisionObjects('working_area') 
   
   print("move overview done")
   fsmState="overviewPose"
end


---
-- Transforms a 3d point measured in depth-camera coordinate system to robot base
-- @param torch.Tensor 3xN points in camera coordinate system
-- @param torch.Tensor 4x4 robot pose 
local function CamToWorld(points, robotPose)
  return (robotPose * heye * points:t()):t()
end


local function main(N)
   local timer = torch.Timer()

   poses.overview = roboControl:WebCamLookAt(torch.DoubleTensor({0.25, 0.58, 0}), 0.65, math.rad(-45), math.rad(0.5), heye)
   
   MoveToOverview()
   fsmState="overviewPose"

   local emptyTableTimer = torch.Timer()

   while ros.ok() do
      if(fsmState == "overviewPose") then
	 print("Grabbing point cloud and RGB image")
	 timer:reset()
	 timer:resume()
	 local cloud = depthcam:GrabPointCloud()
	 timer:stop()
	 print(string.format("PointCloud finished, took %.1f ms", timer:time().real*1000))
	 local capturePose = roboControl:GetPose()
	 timer:reset()
	 timer:resume()
	 timer:stop()
	 
	 timer:reset()
	 timer:resume()
	 segmentation:process(cloud)
	 timer:stop()
	 print(string.format("ObjectSegmentation finished, took %.1f ms", timer:time().real*1000))
	 
	 if (segmentation.objMeans:dim(1) > 0) and (segmentation.objMeans:size(1) > 0) then
	    timer:reset()
	    timer:resume()
	    
	    -- transform from camera into world coordinates
	    local meanOfEachObject = CamToWorld(segmentation.objMeans, capturePose)
	    local e = torch.Tensor(segmentation.objEigenVectors:size(1), 4)
	    e[{{},{1,3}}] = segmentation.objEigenVectors
	    e[{{},4}] = 0 -- only rotate, no movement
	    local firstEigenvectorOfEachObject = CamToWorld(e, capturePose)[{{},{1,3}}]:clone()
	    local targets = FindObjects(meanOfEachObject, firstEigenvectorOfEachObject)
	    timer:stop()
	    print(string.format("ObjectLocalisation finished, took %.1f ms", timer:time().real*1000))
	    
	    if #targets > 0 then
               local target = targets[1]
	       poses.approache=CreateApproachPose(target, APPROACH_DISTANCE)
	       poses.grip=CreateGripPose(target, -0.027)
	       poses.drop=CreateDropPose(poses.approache)

	       fsmState = "picking"
	       PickObject()
	    else
	       fsmState="emptyTable"
	    end
	    
	 else
	    fsmState="emptyTable"
	 end
      end
      
      if(fsmState == "pickDone") then
	 fsmState = "dropping"
	 DropObject()
      end
      
      if(fsmState == "dropDone") then
	 fsmState = "moveToOverview"
	 MoveToOverview()
      end
      
      if(fsmState == "emptyTable") then
	 if (emptyTableTimer:time().real*1000 > 5000) then
	    fsmState = "overviewPose"
	    emptyTableTimer:reset()
	 end
      end
     
   end
end


main(9)
ros.shutdown()
