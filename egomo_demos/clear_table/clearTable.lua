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

-- Required to colorize cloud, not for actual operation
local matrixRGB=torch.FloatTensor{
   {873.59043129674, 0, 480.87504519504, 0}, 
   {0, 875.86880791514, 357.51156061521, 0}, 
   {0, 0, 1, 0}}     
local matrixCamTrafo=torch.FloatTensor{
   {-0.9998948829071, -0.001522834188929, -0.014418880408065, 0.027557160671132},    
   {0.0018089833246721, -0.99980126830416, -0.019853248518212, 0.031334218255476},   
   {-0.014385781713905, -0.019877245116661, 0.99969892688302, -0.03391795945098},    
   {0, 0, 0, 1}}     



local depthcam=egomoTools.structureio:new(camIntrinsicsIR)
depthcam:Connect()

local webcam = egomoTools.webcam:new("web_cam")
webcam:ConnectDefault()
webcam:SetFocusValue(5)

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


local function CreateGripPose(target, zdistance)
   return CreateApproachPose(target, zdistance)
end


local function CreateDropPose(orientation)
   local dropPoseManual = orientation:clone()
   dropPoseManual[1][4]=0.55
   dropPoseManual[2][4]=0.05
   dropPoseManual[3][4]=0.35
   return dropPoseManual
end


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



local function CamToWorld(points, robotPose)
  return (robotPose * heye * points:t()):t()
end


local function ArrangeCloudViewWindows(winA, winB)
   if winA ~= nil then
      local camPos = {}
      --[[camPos.x    =-0.11
      camPos.y    =0.2
      camPos.z    =-0.80
      camPos.viewX=-0.06
      camPos.viewY=0.05
      camPos.viewZ=0.52
      camPos.upX  =-0.008
      camPos.upY  =-0.994
      camPos.upZ  =-0.11 ]]

      camPos.x    = -0.0539242
      camPos.y    = 0.310586
      camPos.z    = -0.7
      camPos.viewX= -0.0514919
      camPos.viewY= 0.15113
      camPos.viewZ= 0.236273
      camPos.upX  = 0.014
      camPos.upY  = -0.91
      camPos.upZ  = -0.41


      winA:setCameraPosition(camPos.x, camPos.y, camPos.z,
				  camPos.viewX, camPos.viewY, camPos.viewZ,
				  camPos.upX, camPos.upY, camPos.upZ)
      winA:setCameraClipDistances(0.003, 3.0)
      winA:setSize(1280, 720)
      winA:setPosition(60, 70)



   end

   if winB ~= nil then
      print("TODO")
   end
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
