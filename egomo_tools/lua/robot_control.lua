--- A module for moving the robot with ROS and MoveIT
-- @classmod RobotControl

local function checkModuleInstalled(name)
  if package.loaded[name] then  -- package already loaded, so it exists
    return true
  else
    for _, searcher in ipairs(package.searchers or package.loaders) do
      local loader = searcher(name)
      if type(loader) == 'function' then
        package.preload[name] = loader
        return true
      end
    end
    return false
  end
end

-- Import section
local ros = require 'ros'
local moveit
if checkModuleInstalled('moveit') then
   moveit = require 'moveit'
else
   moveit = nil
end

local tf = ros.tf
local sleep = sys.sleep
local math = math

local sock = require 'socket'
local ffi =require 'ffi'

-- no more external access after this point
_ENV = nil

RobotControl = {
   robotAvailable,
   rosMoveGroup,
   planningScene,
   rosSpinner,
}


--- Constructor of the robot class.
-- It initializes ROS, adds collision objects to the scene (right now they fit to the Xamla lab
-- and have to be changed if the robot is positioned in a different environment).
-- It also initializes the movegroups for MoveIT.
-- @param idString string that identifies the robots node in ROS
-- @param robotSpeed maximum speed of the robot. The value may be beteen zero and one
function RobotControl:new(idString, robotSpeed)
   o = {}
   setmetatable(o, self)
   self.__index = self

   if moveit == nil then
      print("moveit not found")
      robotAvailable=false

      ros.init(idString)  -- short initialisation, just for ROS but no moment/scene related things
      ros.Time.init()

      rosSpinner = ros.AsyncSpinner()  -- background job
      rosSpinner:start()
   else
      -- full initialisation
      robotSpeed = robotSpeed or 0.1
      if robotSpeed > 1 then
         print("Invalid velocity scaling factor: "..robotSpeed". Valid range: 0 to 1")
         print("Scaling set to 0.4")
         robotSpeed=0.4
      end

      print("Robot speed scaling set to "..robotSpeed)

      ros.init(idString)
      ros.Time.init()

      rosSpinner = ros.AsyncSpinner()  -- background job
      rosSpinner:start()

      self.planningScene = moveit.PlanningSceneInterface()
      self.planningScene:addPlane('ground plane', 0, 0, 1, -0.001)

      self.rosMoveGroup = moveit.MoveGroup('manipulator')
      self.rosMoveGroup:setPlanningTime(10.0);
      self.rosMoveGroup:setMaxVelocityScalingFactor(robotSpeed)
      self.rosMoveGroup:setGoalTolerance(1E-4)
      -- self.rosMoveGroup:setPlannerId("BKPIECEkConfigDefault");
      self.rosMoveGroup:getCurrentState()
      self.rosMoveGroup:setStartStateToCurrentState()

      print('Current end effector link:')
      print(self.rosMoveGroup:getEndEffectorLink())

      -- Collision box for table
      box_pose = tf.Transform()
      box_pose:setOrigin({0.05, 0.45, -0.431})
      rot = box_pose:getRotation()
      rot:setRPY(0,0,-45,1)
      box_pose:setRotation(rot)
      self.planningScene:addBox('ground', 0.8, 0.90, 0.8, box_pose)

      -- Collision box for wall
      boxWall_pose = tf.Transform()
      boxWall_pose:setOrigin({-0.0, -0.45, 0.0})
      rot = boxWall_pose:getRotation()
      rot:setRPY(0,0,0,1)
      boxWall_pose:setRotation(rot)
      self.planningScene:addBox('WallA', 10.1, 0.1, 10.8, boxWall_pose)

      -- Collision box for wall
      boxWallB_pose = tf.Transform()
      boxWallB_pose:setOrigin({-0.8, -0.0, 0.0})
      rot = boxWallB_pose:getRotation()
      rot:setRPY(0,0,-90,1)
      boxWallB_pose:setRotation(rot)
      self.planningScene:addBox('WallB', 10.1, 0.1, 10.8, boxWallB_pose)
   end
   return o
end


--- Returns the moveIT move groups which is required for planning
-- @return a moveGroup object
function RobotControl:GetMoveGroup()
  return self.rosMoveGroup
end


--- Returns the planning scene interface
-- @return Planning scene interface
function RobotControl:GetPlanningSceneInterface()
  return self.planningScene
end


--- Creates a pose that can be used to move the robot by providing a position and a rotation
-- @param pos torch.Tensor 3x1 position in robot base coordinates
-- @param rot torch.Tensor 3x3 rotation matrix
function RobotControl:CreatePose(pos, rot)
  local pose = tf.Transform()
  pose:setOrigin(pos)
  pose:setRotation(rot)
  return pose:toTensor()
end


--- Move robot to pose "pose", using move group "rosMoveGroup"
-- @param pose target robot pose, type torch.Tensor(4,4)
-- @treturn bool true if robot has been moved successfully (planning was successfull)
function RobotControl:MoveRobotTo(pose)
   if robotAvailable == false then  -- without robot, just ignore the move command
      print("Robot would move to: ")
      print(pose)
      sys.sleep(1.0)
      return true
   end

   if (pose == nil) then
      print("RobotControl:MoveRobotTo: Target pose is nil!!")
      return false
   end

   self.rosMoveGroup:getCurrentState()
   self.rosMoveGroup:setStartStateToCurrentState()
   self.rosMoveGroup:setPoseTarget(pose)
   print("----- Start Planning ------")
   local s, p = self.rosMoveGroup:plan()
   local currentStartPose= self.rosMoveGroup:getCurrentPose()
   print("----- Planning Finished ------")
   if s == 0 then
      return false
   end
   print("-------- Moving ... -----------")
   self.rosMoveGroup:execute(p)
   print("-------- Move Done ---------")

   return true
end


function RobotControl:Normalize(v)
   -- Private
   -- Utility function for PointAtPose
   return v / torch.norm(v)
end


--- Calculates the TCP pose required to point the TCP z axis to point at with TCP at position eye
-- eye becomes origin, 'at' lies on x-axis
-- @param eye torch.Tensor(3,1); TCP position in x, y, z
-- @param at torch.Tensor(3,1); Point to look at
-- @param up torch.Tensor({0, 0, 1}); up direction of the camera
-- @return torch.Tensor(4,4), robot pose
function RobotControl:PointAtPose(eye, at, up, handEye)
  local zaxis = self:Normalize(at - eye)
  local xaxis = self:Normalize(torch.cross(zaxis, up))
  local yaxis = torch.cross(zaxis, xaxis)

  local basis = torch.Tensor(3,3)
  basis[{{},{1}}] = xaxis
  basis[{{},{2}}] = yaxis
  basis[{{},{3}}] = zaxis

  local t = tf.Transform()
  t:setBasis(basis)
  t:setOrigin(eye)
  local tcpLookat=t:toTensor():clone()

  if handEye ~= nil then
    tcpLookat = tcpLookat*torch.inverse(handEye)
  end

  return tcpLookat
end


function RobotControl:RotateGripper(currPose, alpha)
  local t = tf.Transform()
  t:fromTensor(currPose)
  local newRot = t:getRotation() * tf.Quaternion({0,0,1}, alpha)
  t:setRotation(newRot)

  return t:toTensor()
end


--- Calculates the tool center pose for a given camera view
-- Calculates the tool center point pose which corresponds to a camera located at distance from target point looking at target point
-- with angle polarAngle and azimuthalAngle (in radians)
-- @param targetPoint torch.Tensor(3,1); point to look at with the camera
-- @param distance distance in meter between targetPoint and camera
-- @param polarAngle polar angle in radians
-- @param azimuthalAngle azimuthal angle in radians
-- @param handEye torch.Tensor(4,4); hand-eye transformation matrix between TCP and camera
-- @param upInverse
-- @return torch.Tensor(4,4); robot pose
function RobotControl:WebCamLookAt(targetPoint, distance, polarAngle, azimuthalAngle, handEye, upInverse)
   local r=distance
   local theta=azimuthalAngle
   local phi=polarAngle
   local upInverse=upInverse or false

   local upVector
   if upInverse then
      upVector = torch.DoubleTensor({0,0,-1})
   else
      upVector = torch.DoubleTensor({0,0,1})
   end

   local offset = torch.DoubleTensor({math.sin(phi)*(r*math.sin(theta)), math.cos(phi)*(r*(-1)*math.sin(theta)), r*math.cos(theta)})
   local eyePoint = targetPoint + offset
   local eyePose = self:PointAtPose(eyePoint, targetPoint, upVector)

   local poseForTCP = eyePose*torch.inverse(handEye)
   return poseForTCP
end


---
--
function RobotControl:LookDown(targetPoint, rotZangle, handEye)
   local eyePose = CreatePose(targetPoint, tf.Quaternion({0,0,1}, rotZangle))
   local poseTCP = eyePose * torch.inverse(handEye)
   return poseTCP
end


--- Reads the current pose of the robot usung the associated move group.
-- The function returns a table containing the pose in different formats:
-- pose.full: 4x4 torch.Tensor pose of the TCP
-- pose.x, pose.y pose.z position of the TCP in robot base coordinates
-- pose.roll pose.pitch pose.yaw orientation of the TCP as Roll, Pitch, Yaw
-- @param printData prints the data on the command line
-- @return table with fields full, x, y, z, roll, pitch, yaw
function RobotControl:ReadRobotPose(printData)
   if robotAvailable == false then  -- without robot, return an empty position table
      return {}
   end

   printData = printData or false
   local pose = self.rosMoveGroup:getCurrentPose_StampedTransform()
   local origin = pose:getOrigin()
   local rpy = pose:getRotation():getRPY()

   local readBackPose = {}
   readBackPose["full"]=pose:toTensor()
   readBackPose["x"]=origin[1]
   readBackPose["y"]=origin[2]
   readBackPose["z"]=origin[3]
   readBackPose["roll"]=rpy[1]
   readBackPose["pitch"]=rpy[2]
   readBackPose["yaw"]=rpy[3]

   if printData then
      print(string.format('robot position: x=%.4f, y=%.4f, z=%.4f; roll=%.4f, pitch=%.4f, yaw=%.4f', readBackPose["x"], readBackPose["y"], readBackPose["z"], readBackPose["roll"], readBackPose["pitch"], readBackPose["yaw"]))
   end

   return readBackPose
end


---
-- Returns the robots pose (TCP) as 4x4 torch.Tensor
-- @return torch.Tensor pose of the TCP
function RobotControl:GetPose()
   if robotAvailable == false then
      return torch.DoubleTensor(4,4):zero()
   else
      return self.rosMoveGroup:getCurrentPose_StampedTransform():toTensor()
   end
end

-- Direct read of the robot pose via socket connection
-- *** START ***

function RobotControl:interpret_as_double(x)
  local u64_buf = ffi.typeof('uint64_t[1]')()
  local double_ptr_ctype = ffi.typeof('double*')
  u64_buf[0] = x
  return ffi.cast(double_ptr_ctype, u64_buf)[0]
end


function RobotControl:unpackVector(r, D)
  local v = torch.DoubleTensor(D)
  for i = 1,D do
    v[i] = self:interpret_as_double(bit.bswap(r:readUInt64()))
  end
  return v
end


function RobotControl:decode(block)
  local s = torch.ByteStorage():string(block)
  local r = ros.StorageReader(s)

  local len = bit.bswap(r:readInt32())
  if len == #block then

    return {
      time = bit.bswap(r:readInt64()),
      q_target = self:unpackVector(r, 6),     -- Target joint positions
      qd_target = self:unpackVector(r, 6),    -- Target joint velocities
      qdd_target = self:unpackVector(r, 6),   -- Target joint accelerations
      i_target = self:unpackVector(r, 6),     -- Target joint currents
      m_target = self:unpackVector(r, 6),     -- Target joint moments (torques)
      q_actual = self:unpackVector(r, 6),     -- Actual joint positions
      qd_actual = self:unpackVector(r, 6),    -- Actual joint velocities
      i_actual = self:unpackVector(r, 6),     -- Actual joint currents
      i_contro_ = self:unpackVector(r, 6),    -- Joint control currents
      tool_vector_actual = self:unpackVector(r, 6),  -- Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
      tcp_speed_actual = self:unpackVector(r, 6),    -- Actual speed of the tool given in Cartesian coordinates
      tcp_force = self:unpackVector(r, 6),           -- Generalised forces in the TCP
      tool_vector_target = self:unpackVector(r, 6),  -- Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
      tcp_speed_target = self:unpackVector(r, 6),    -- Target speed of the tool given in Cartesian coordinates
      digital_input_bits = bit.bswap(r:readInt64()),  -- Current state of the digital inputs. NOTE: these are bits encoded as int64_t, e.g. a value of 5 corresponds to bit 0 and bit 2 set high
      motor_temp = self:unpackVector(r, 6),                -- Temperature of each joint in degrees celsius
      controller_time = bit.bswap(r:readInt64())      -- Controller realtime thread execution time
    }

  end
end


--- Reads the internal state of the robot ny calling the UR5 interface directly (no MoveIT).
-- @return table of all readable internal parameters of the UR5
function RobotControl:ReadUR5data()
   if robotAvailable == false then
      return
   end

  local client = socket.tcp()

  client:connect('ur5', 30003)
  client:settimeout(0.01, 'b')

  for i=1,1000 do
    local r, e, p = client:receive(2048)
    if e == 'timeout' then
      r = p
    end
    if not r then
      error(e)
    end

    if #r > 0 then
      local y = self:decode(r)
      if y then
        client:close()
        return y
      end
    end
  end
  client:close()
  error('no valid data block received')
end

--- Extracts the joint coordinates from the UR5 data
-- @param data the raw data read by ReadUR5data
-- @param printData a bool value that indicates if the data should be printed on the screen
-- @return ???
function RobotControl:DecodeUR5actualJointState(data, printData)
  local printData = printData or false

  local jointPos = data.q_actual
  local jointVel = data.qd_actual
  local jointCurrent = data.i_actual

  if printData then
    local names = self:GetUR5JointNames()
    for i=1, 6 do
      print(names[i].." = "..math.deg(jointPos[i]))
    end
  end

  return jointPos, jointVel, jointCurrent
end


--- Function to create for each joint a human readable name
-- @return table with joint names [1] = "Base"... [6] = "Wrist3"
function RobotControl:GetUR5JointNames()
  local names = {}
  names[1]="Base"
  names[2]="Shoulder"
  names[3]="Elbow"
  names[4]="Wrist1"
  names[5]="Wrist2"
  names[6]="Wrist3"

  return names
end


function RobotControl:rotateRPY(roll, pitch, yaw) -- X, Y', Z'' ; roll-pitch-yaw = roll-nick-gier
  local psi, theta, phi = yaw, pitch, roll
  local cos, sin = math.cos, math.sin
  local R = torch.Tensor({
    { cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi), 0 },
    { cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), 0 },
    { -sin(theta),         sin(phi)*cos(theta),                            cos(phi)*cos(theta),                            0 },
    { 0, 0, 0, 1 },
  })
  return R
end


--- Extracts the TCP Pose from the UR5 data without MoveIT.
-- The function returns a table containing the pose in different formats:
-- pose.full: 4x4 torch.Tensor pose of the TCP
-- pose.x, pose.y pose.z position of the TCP in robot base coordinates
-- pose.roll pose.pitch pose.yaw orientation of the TCP as Roll, Pitch, Yaw
-- @param data the raw data read by ReadUR5data
-- @param printData prints the data on the command line
function RobotControl:DecodeUR5TcpPose(data, printData)

  local ur_tcp = data.tool_vector_actual

  local axis = ur_tcp[{{4,6}}]
  local phi = torch.norm(ur_tcp[{{4,6}}])

  local pose = ros.tf.Transform()
  pose:setOrigin(ur_tcp[{{1,3}}])
  pose:setRotation(ros.tf.Quaternion(axis, phi))

  local origin = pose:getOrigin()
  local rpy = pose:getRotation():getRPY()

  local readBackPose = {}
  readBackPose["full"]=pose:toTensor()
  readBackPose["x"]=origin[1]
  readBackPose["y"]=origin[2]
  readBackPose["z"]=origin[3]
  readBackPose["roll"]=rpy[1]
  readBackPose["pitch"]=rpy[2]
  readBackPose["yaw"]=rpy[3]

  if printData then
    print(string.format('robot position via UR5: x=%.4f, y=%.4f, z=%.4f; roll=%.4f, pitch=%.4f, yaw=%.4f', readBackPose["x"], readBackPose["y"], readBackPose["z"], readBackPose["roll"], readBackPose["pitch"], readBackPose["yaw"]))
  end

  return readBackPose
end

-- *** END *** direct read of robot pose via socket


return RobotControl
