require 'torch'

local ros = require 'ros'
local pcl = require 'pcl'

local cv = require 'cv'
require 'cv.highgui'
require 'cv.imgproc'

package.path = "../core/?.lua;" .. package.path
require 'ObjectsOnPlaneSegmentation'

local egomo_tools = require 'egomo-tools'

-- Constants which you might have to customize for your own environment
local ROBOT_SPEED = 1.0
local GRIPPER_LENGTH = 0.22
local APPROACH_DISTANCE = 0.08

local VISUALIZE = true

local CAM_INTRINSICS_IR = torch.FloatTensor{
  {584.8203397188, 0, 322.13417604596},
  {0, 587.09551694351, 242.17211588298},
  {0, 0, 1}}   -- 2016-05-04

local HAND_EYE = torch.DoubleTensor({
  {-0.0026, -0.7745,  0.6325,  0.0318},
  {-0.0282,  0.6324,  0.7742,  0.0442},
  {-0.9996, -0.0158, -0.0235,  0.0949},
  { 0.0000,  0.0000,  0.0000,  1.0000}})  -- 2016-06-13

local OVERVIEW_POSE_POINT_TO_LOOK = torch.DoubleTensor({0.25, 0.58, 0})
local OVERVIEW_POSE_Z_DISTANCE = 0.65

local GRIP_POSE_Z_DISTANCE = -0.027

local DROP_POSITION = torch.DoubleTensor({0.213, 0.690, 0.470})

local GRIPPER_SPEED = 255
local GRIPPER_FORCE = 255

-- Constants which remain unchanged
local STATE_INIT = 0
local STATE_PICKING = 1
local STATE_PICK_DONE = 2
local STATE_DROPPING = 3
local STATE_DROP_DONE = 4
local STATE_MOVE_TO_OVERVIEW = 5
local STATE_OVERVIEW_POSE = 6
local STATE_EMPTY_TABLE = 7

local ACTION_SUCCEEDED = 7

local robot_control
local planning_scene_interface
local depthcam
local gripper_control
local tf
local poses = {}
local fsm_state

-- object segmentation
local segmentation = ObjectsOnPlaneSegmentation{
  cloudType = 'XYZ',
  distanceThreshold = 0.015,
  cropRadius = 1,
  clusterTolerance = 0.02,  -- 2cm
  minClusterSize = 300,
  maxClusterSize = 25000
}

local function initialze()
  robot_control = egomo_tools.robot:new("cleanTable", ROBOT_SPEED)
  planning_scene_interface = robot_control:GetPlanningSceneInterface()
  tf = ros.tf

  depthcam = egomo_tools.structureio:new(CAM_INTRINSICS_IR, "egomo_depthcam")
  depthcam:Connect()

  gripper_control = egomo_tools.gripper:new()
  gripper_control:Connect()

  sys.sleep(0.2)
  gripper_control:ResetGripper()
  gripper_control:OpenGripper()
  gripper_control:SetGripForce(5)

  --[[
  local gripper_init_result = gripper_control:resetGripperViaAction()

  if gripper_init_result == ACTION_SUCCEEDED then
    print("Gripper has been initialized successfully!")
    gripper_control:openGripperViaAction()
  else
    print("Gripper initialization has failed!")
  end

  if gripper_init_result == ACTION_SUCCEEDED then
    gripper_control:openGripperViaAction(15, 5, true, GRIPPER_SPEED, GRIPPER_FORCE)
  end
  --]]
  fsm_state = STATE_INIT
end

--- 
-- Utility function to translate color IDs to actual color values
-- @param id color ID
function getColor(id)
   local cloudColors = {}
   cloudColors[1] = { 0.5, 0.7, 1.0} -- lightskyblue
   cloudColors[2] = {1, 0.843137, 0 } -- gold
   cloudColors[3] = { 0.133333, 0.545098, 0.133333} -- forrestgreen
   cloudColors[4] = { 190/255.0, 34/255.0, 34/255.0} -- firebrick
   cloudColors[5] = { 0.780392, 0.0823529, 0.521569} -- mediumvioletred
   cloudColors[6] = { 0.980392, 0.921569, 0.843137} -- antiquewhite
   cloudColors[7] = { 1, 0.7, 0} -- orange
   cloudColors[8] = { 0, 0, 0.803922} -- mediumblue
   cloudColors[9] = { 0.729412, 0.333333, 0.827451} -- mediumorchid
   cloudColors[10] = { 0.956863, 0.643137, 0.376471} --sandybrown
   cloudColors[11] = {1, 0.54902, 0 } -- darkorange

   cloudColors[12] = { 0, 1, 0.5} -- Spring Green
   cloudColors[13] = { 0.2, 0.2, 1} -- blue
   cloudColors[14] = { 160/255.0, 32/255.0, 240/255.0} -- Purple
   cloudColors[15] = { 230/255.0, 230/255.0, 250/255.0} -- Lavender

   cloudColors[16] = { 0.3, 0.3, 0.3} -- dark gray
   cloudColors[17] = { 0.86, 0.86, 0.86} -- light gray

   if(id<=17) then
      return cloudColors[id]
   else
      return {1, 1, 1}
   end
end

---
-- Heuristic to find grippable objects
-- @param mean_of_each_object center point of the object found by the segmentation algorithm
-- @param first_eigenvector_of_each_object the corrspnding largest eigenvector of the object, i.e. main axis
local function findObjects(mean_of_each_object, first_eigenvector_of_each_object)
  local result = {}
  local origin = torch.Tensor({0,0,0,1})
  local N = mean_of_each_object:size(1)
  for i=1,N do
    local mean = mean_of_each_object[i]
    local main_axis = first_eigenvector_of_each_object[i]
    if mean[3] >= -0.04 and mean[3] <= 0.05 and (mean - origin):norm() > 0.3 then
      table.insert(result, {
        center = mean,
        main_axis = main_axis,
        cloud_index = i
      })
    end
  end
  return result
end

---
-- Creates the grip pose
-- @param target position of the object and the orientation of the object to grip
-- @param z_distance height above z plane where to grip
local function createApproachPose(target, z_distance)
  target.center[3]=0
  print("Target main axis:")
  print(target.main_axis)
  print("Target center:")
  print(target.center)
  local main_axis = target.main_axis:clone()

  print('main axis:')
  print(main_axis)

  local tcpPosition = target.center[{{1,3}}]:clone()
  tcpPosition[3] = tcpPosition[3] + z_distance + GRIPPER_LENGTH

  local pose = robot_control:PointAtPose(tcpPosition, target.center[{{1,3}}], torch.DoubleTensor({0,1,0}))
  local alpha = math.atan2(main_axis[1], main_axis[2])
  -- alpha = alpha+math.rad(90) -- no additional +math.rad(90) here because PointAtPose() provides an already rotated pose
  local rot_pose = robot_control:RotateGripper(pose, alpha)
  print("pose for picking")
  print(rot_pose)
  return rot_pose
end

---
-- Creates the grip pose
-- @param target position of the object and the orientation of the object to grip
-- @param z_distance height above z plane where to grip
local function createGripPose(target, z_distance)
  return createApproachPose(target, z_distance)
end

---
-- Sets the position for dropping the objects
-- @param orientation 4x4 torch.Tensor the orientation of the TCP when dropping positions
local function createDropPose(orientation)
  local drop_pose_manual = orientation:clone()
  drop_pose_manual[{{1,3}, {4}}] = DROP_POSITION
  return drop_pose_manual
end

---
-- The picking action
-- Robot moves to approach position, moves down and closes the gripper
local function pickObject()
  print("picking object, approachPose: ")
  print(poses.approache)

  robot_control:MoveRobotTo(poses.approache)

  -- descent
  print("gripPose")
  print(poses.grip)
  robot_control:MoveRobotTo(poses.grip)

  gripper_control:CloseGripper()
  sys.sleep(0.6) 
--[[
  while gripper_control:closeGripperViaAction() ~= 7 do
    print("Closing gripper failed. Trying again!")
  end
--]]

  fsm_state = STATE_PICK_DONE
end

---
-- Dropping the object
local function dropObject()
  print("moving and dropping object")

  -- return to approach position
  robot_control:MoveRobotTo(poses.approache)
  robot_control:MoveRobotTo(poses.drop)

  --drop
  gripper_control:OpenGripper()
  sys.sleep(0.1)
  --[[
  while gripper_control:openGripperViaAction() ~= 7 do
    print("Opening gripper failed. Trying again!")
  end
  --]]
  fsm_state = STATE_DROP_DONE
end

---
-- Moves the robot to the overview pose.
-- An additional colision object is added to prevent that the robot
-- moves one of its joints into the view frustrum of the camera
local function moveToOverview()
  print("moveto overview")
  print(poses.overview)
  local sphere_pose = tf.Transform()
  sphere_pose:setOrigin({-0.23, 0.35, 0.1})
  sphere_pose:setRotation(tf.Quaternion({1, 0, 0}, math.pi))
  planning_scene_interface:addSphere('working_area', 0.2, sphere_pose)
  print("Pose Overview:")
  robot_control:MoveRobotTo(poses.overview)
  planning_scene_interface:removeCollisionObjects('working_area')

  print("move overview done")
  fsm_state = STATE_OVERVIEW_POSE
end


---
-- Transforms a 3d point measured in depth-camera coordinate system to robot base
-- @param torch.Tensor 3xN points in camera coordinate system
-- @param torch.Tensor 4x4 robot pose
local function camToWorld(points, robot_pose)
  return (robot_pose * HAND_EYE * points:t()):t()
end


local function main(N)
  local timer = torch.Timer()

  --poses.overview = robot_control:WebCamLookAt(OVERVIEW_POSE_POINT_TO_LOOK, OVERVIEW_POSE_Z_DISTANCE, math.rad(-45), math.rad(0.5), HAND_EYE)
  poses.overview = torch.DoubleTensor{
     { 0.5165, -0.4161, -0.7484, -0.2357},
     { 0.5838, -0.4683,  0.6633,  0.3522},
     {-0.6264, -0.7795,  0.0010,  0.6227},
     { 0.0000,  0.0000,  0.0000,  1.0000}}
  
  moveToOverview()
  fsm_state = STATE_OVERVIEW_POSE

  local empty_table_timer = torch.Timer()

  local viewer
  if (VISUALIZE) then
    viewer = pcl.PCLVisualizer('Viewer', true)
  end


  while ros.ok() do
    if(fsm_state == STATE_OVERVIEW_POSE) then
      print("Grabbing point cloud and RGB image")
      local cloud = depthcam:GrabPointCloud()
      local capture_pose = robot_control:GetPose()
      segmentation:process(cloud)

      print("Plane parameters: ")
      print(segmentation.planeParameters)
      print("Number of found objects: " .. segmentation.objIndices:size())

      if (VISUALIZE) then
	viewer:removeAllPointClouds()
        viewer:addCoordinateSystem(0.05)  -- world coordinate system

        viewer:addPointCloud(segmentation.planeCloud, 'plane')
        viewer:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 2, 'plane')
        local cloudColor = getColor(1)
        viewer:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 
                                                 cloudColor[1], cloudColor[2], cloudColor[3], 'plane')

        viewer:addPointCloud(segmentation.notPlaneCloud, 'notPlane')
        viewer:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 2, 'notPlane')
        local cloudColor = getColor(2)
        viewer:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 
                                                 cloudColor[1], cloudColor[2], cloudColor[3], 'notPlane')

	local numberOfClusters = segmentation.objIndices:size()
        for i=1, numberOfClusters do
          local currObj = pcl.filter.extractIndices(segmentation.objCloud, segmentation.objIndices[i], nil, false)
          viewer:addPointCloud(currObj, 'obj'..i)
          viewer:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 2, 'obj'..i)
          local cloudColor = getColor(2+i)
          viewer:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 
                                                   cloudColor[1], cloudColor[2], cloudColor[3], 'obj'..i)
        end

        viewer:spin()
      end

      if (segmentation.objMeans:dim(1) > 0) and (segmentation.objMeans:size(1) > 0) then

        -- transform from camera into world coordinates
        local mean_of_each_object = camToWorld(segmentation.objMeans, capture_pose)
        local e = torch.Tensor(segmentation.objEigenVectors:size(1), 4)
        e[{{},{1,3}}] = segmentation.objEigenVectors
        e[{{},4}] = 0 -- only rotate, no movement
        local first_eigenvector_of_each_object = camToWorld(e, capture_pose)[{{},{1,3}}]:clone()
        local targets = findObjects(mean_of_each_object, first_eigenvector_of_each_object)

        if #targets > 0 then
          local target = targets[1]
          poses.approache = createApproachPose(target, APPROACH_DISTANCE)
          poses.grip = createGripPose(target, GRIP_POSE_Z_DISTANCE)
          poses.drop = createDropPose(poses.approache)

          if (VISUALIZE) then
            local pickObj = pcl.filter.extractIndices(segmentation.objCloud, segmentation.objIndices[target.cloud_index], nil, false)
            viewer:addPointCloud(pickObj, 'pickobj')
            viewer:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 3, 'pickobj')
            -- set the point cloud of the next object to pick to red
            viewer:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 1.0, 0.2, 0.2, 'pickobj')
            viewer:spinOnce()
          end

          fsm_state = STATE_PICKING
          pickObject()
        else
          fsm_state = STATE_EMPTY_TABLE
        end

      else
        fsm_state = STATE_EMPTY_TABLE
      end
    end

    if(fsm_state == STATE_PICK_DONE) then
      fsm_state = STATE_DROPPING
      dropObject()
    end

    if(fsm_state == STATE_DROP_DONE) then
      fsm_state = STATE_MOVE_TO_OVERVIEW
      moveToOverview()
    end

    if(fsm_state == STATE_EMPTY_TABLE) then
      if (empty_table_timer:time().real*1000 > 5000) then
        fsm_state = STATE_OVERVIEW_POSE
        empty_table_timer:reset()
      end
    end

  end
end

initialze()
main(9)
ros.shutdown()
