-- module gripper_interface

-- Import section
local ros = require 'ros'
require 'ros.actionlib.SimpleActionClient'
local actionlib = ros.actionlib

-- no more external access after this point
_ENV = nil

GripperInterface = {
  nodeID,
  serviceGripperCtl,
  actionGripperActivation,
  actionGripperSetPosition
}

function GripperInterface:new (nodeIDstring)
  o = {}
  setmetatable(o, self)
  self.__index = self

  self.nodeID=nodeIDstring or "XamlaEgomo"
  return o
end

function GripperInterface:Connect ()
  local nodehandle = ros.NodeHandle() -- generic node handle
  print("/"..self.nodeID.."/SendCommand")
  self.serviceGripperCtl = nodehandle:serviceClient("/"..self.nodeID.."/SendCommand", "egomo_msgs/SendCommand")
  self.actionGripperActivation = actionlib.SimpleActionClient('egomo_msgs/EgomoGripperActivate', "/" .. self.nodeID .. "/gripper_activation_action", nodehandle)
  self.actionGripperSetPosition = actionlib.SimpleActionClient('egomo_msgs/EgomoGripperPos', "/" .. self.nodeID .. "/gripper_pos_action", nodehandle)
  print(self.serviceGripperCtl.spec)
end

--reset (reset is executed only on the transition 1 -> 0 (falling edge))
function GripperInterface:ResetGripper ()
  self:SetMoveSpeed(255)
  self:SentGripperMessage("reset", 1)
  sys.sleep(0.5)
  self:SentGripperMessage("reset", 0)
  self:SetMoveSpeed(255)  -- set defaults for movement speed
  self:SetGripForce(100)  -- and gripping force
end

function GripperInterface:resetGripperViaAction(execute_timeout, preempt_timeout)
  local goal = self.actionGripperActivation:createGoal()
  goal.activate = true
  return self.actionGripperActivation:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end

function GripperInterface:OpenGripper ()
  return self:MoveGripper(0.087)
end

function GripperInterface:openGripperViaAction(execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  local goal = self.actionGripperSetPosition:createGoal()
  goal.goal_pos = 0.087
  goal.set_speed_and_force = set_speed_and_force or false
  if set_speed_and_force then
    goal.speed = speed
    goal.force = force
  end
  return self.actionGripperSetPosition:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end

function GripperInterface:CloseGripper ()
  return self:MoveGripper(0.0)
end

function GripperInterface:closeGripperViaAction(execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  local goal = self.actionGripperSetPosition:createGoal()
  goal.goal_pos = 0.0
  goal.set_speed_and_force = set_speed_and_force or false
  if set_speed_and_force then
    goal.speed = speed
    goal.force = force
  end
  return self.actionGripperSetPosition:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end

function GripperInterface:MoveGripper (value)
  return self:SentGripperMessage ("pos_cmd", value)
end

function GripperInterface:moveGripperViaAction(pos_goal, execute_timeout, preempt_timeout, set_speed_and_force, speed, force)
  local goal = self.actionGripperSetPosition:createGoal()
  goal.goal_pos = pos_goal
  goal.set_speed_and_force = set_speed_and_force or false
  if set_speed_and_force then
    goal.speed = speed
    goal.force = force
  end
  return self.actionGripperSetPosition:sendGoalAndWait(goal, execute_timeout or 15, preempt_timeout or 5)
end

function GripperInterface:SetMoveSpeed(value)
  value = value or 255
  return self:SentGripperMessage ("max_speed", value)
end

function GripperInterface:SetGripForce(value)
  value = value or 190
  --max_force (0-255)
  return self:SentGripperMessage ("max_force", value)
end

function GripperInterface:SentGripperMessage (name, value)
  local m=ros.Message(self.serviceGripperCtl.spec.request_spec) -- get a message template of that service
  m.command_name=name
  m.value=value

  -- send the message
  local msgGripper = self.serviceGripperCtl:call(m)
  if msgGripper == nil then
    print("Command failed")
    return nil
  end

  return msgGripper
end

return GripperInterface
