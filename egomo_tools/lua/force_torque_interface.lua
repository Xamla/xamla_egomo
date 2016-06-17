-- module force_torque_interface

-- Import section
local ros = require 'ros'
local smooth = require 'egomo-tools.exponential_smoothing'

_ENV = nil

ForceTorqueInterface = {
  nodeID,
  forceTorqueSubscriber,
  rosSpinner,
  filter_active = false,
  filter,
}

function ForceTorqueInterface:new(idString, nodeIDstring)
  o = {}
  setmetatable(o,self)
  self.__index = self

  self.nodeID=nodeIDstring or "geometry_msgs"

  ros.init(idString)
  ros.Time.init()
  self.rosSpinner = ros.AsyncSpinner()
  self.rosSpinner:start()
  return o
end

function ForceTorqueInterface:connect(protocol, queueSize)
  local nodehandle = ros.NodeHandle()
  self.forceTorqueSubscriber = nodehandle:subscribe("XamlaForceTorque", "geometry_msgs/WrenchStamped", queueSize, protocol)
end

function ForceTorqueInterface:getData()
  local data = {}
  ros.spinOnce()
  if self.forceTorqueSubscriber:hasMessage() then
    local msg = self.forceTorqueSubscriber:read()
    data["seq"] = msg.header.seq
    data["timestamp"] = msg.header.stamp
    data["frame_id"] = msg.header.frame_id
    data["force_x"] = msg.wrench.force.x
    data["force_y"] = msg.wrench.force.y
    data["force_z"] = msg.wrench.force.z
    data["torque_roll"] = msg.wrench.torque.x
    data["torque_pitch"] = msg.wrench.torque.y
    data["torque_yaw"] = msg.wrench.torque.z
    if self.filter_active then
      local valuesForUpdate =  {
        ["smooth_force_x"] = data["force_x"],
        ["smooth_force_y"] = data["force_y"],
        ["smooth_force_z"] = data["force_z"],
        ["smooth_torque_roll"] = data["torque_roll"],
        ["smooth_torque_pitch"] = data["torque_pitch"],
        ["smooth_torque_yaw"] =  data["torque_yaw"]
      }
      self.filter:updateValues(valuesForUpdate)
      for index, value in pairs(self.filter.results) do
        data[index] = value
      end
    end
  end
  return data
end


local function createInitPreviousTable(method, firstData, secondData, alpha)
  if method == "exp2" then
    return {
      ["smooth_force_x"] = firstData["force_x"],
      ["smooth_force_y"] = firstData["force_y"],
      ["smooth_force_z"] = firstData["force_z"],
      ["smooth_torque_roll"] = firstData["torque_roll"],
      ["smooth_torque_pitch"] = firstData["torque_pitch"],
      ["smooth_torque_yaw"] = firstData["torque_yaw"]
    }
  elseif method == "brown" then
    return {
      ["smooth_force_x"] = alpha * (secondData["force_x"] - firstData["force_x"]),
      ["smooth_force_y"] = alpha * (secondData["force_y"] - firstData["force_y"]),
      ["smooth_force_z"] = alpha * (secondData["force_z"] - firstData["force_z"]),
      ["smooth_torque_roll"] = alpha * (secondData["torque_roll"] - firstData["torque_roll"]),
      ["smooth_torque_pitch"] = alpha * (secondData["torque_pitch"] - firstData["torque_pitch"]),
      ["smooth_torque_yaw"] = alpha * (secondData["torque_yaw"] - firstData["torque_yaw"])
    }
  elseif method == "exp-holt-winters" then
    return {
      ["smooth_force_x"] = secondData["force_x"] - firstData["force_x"],
      ["smooth_force_y"] = secondData["force_y"] - firstData["force_y"],
      ["smooth_force_z"] = secondData["force_z"] - firstData["force_z"],
      ["smooth_torque_roll"] = secondData["torque_roll"] - firstData["torque_roll"],
      ["smooth_torque_pitch"] = secondData["torque_pitch"] - firstData["torque_pitch"],
      ["smooth_torque_yaw"] = secondData["torque_yaw"] - firstData["torque_yaw"]
    }
  else
    return nil
  end
end

-- This function sets and activates a filter for the data. The raw data will still be returned
--
-- Input:
--  method (string) - defines which filter is used. Available filters are "exp1", "exp2", "brow" and "exp-holt-winters"
--  parameter (table) - defines the parameter for the filter. For "exp1", "exp2", "brow" only the parameter alpha is needed (e.g. {["alpha"] = 0.6}).
--                      For exp-holt-winters two parameter alpha and beta are needed (e.g. {["alpha"] = 0.6, ["beta"] = 0.6}).
--                      Notice that the values for the parameters have to be between 0-1. A higher value leads to past values have a lesser influence on the result.
function ForceTorqueInterface:setFilter(method, parameter)

  local firstData = nil
  local secondData = nil
  local waitForFirstData = true
  local waitForSecondData = true

  while waitForFirstData or waitForSecondData do
    if method == "exp1" then
      firstData = self:getData()
      for i, v in pairs (firstData) do
        waitForFirstData = false
        waitForSecondData = false
        break
      end
    else
      if waitForFirstData then
        firstData = self:getData()
      end
      for i, v in pairs (firstData) do
        waitForFirstData = false
        break
      end
      if waitForSecondData then
        secondData = self:getData()
      end
      for i, v in pairs (secondData) do
        waitForSecondData = false
        break
      end
    end
  end

  local currentData = nil

  if method == "exp1" then
    currentData = {
      ["smooth_force_x"] = firstData["force_x"],
      ["smooth_force_y"] = firstData["force_y"],
      ["smooth_force_z"] = firstData["force_z"],
      ["smooth_torque_roll"] = firstData["torque_roll"],
      ["smooth_torque_pitch"] = firstData["torque_pitch"],
      ["smooth_torque_yaw"] = firstData["torque_yaw"]
    }
  else
    currentData = {
      ["smooth_force_x"] = secondData["force_x"],
      ["smooth_force_y"] = secondData["force_y"],
      ["smooth_force_z"] = secondData["force_z"],
      ["smooth_torque_roll"] = secondData["torque_roll"],
      ["smooth_torque_pitch"] = secondData["torque_pitch"],
      ["smooth_torque_yaw"] = secondData["torque_yaw"]
    }
  end


  if method == "exp1" or method == "exp2" or method == "brown" or method == "exp-holt-winters" then
    local pastData = createInitPreviousTable(method,firstData,secondData, parameter["alpha"])
    self.filter = smooth:new(method, parameter, 6, currentData, pastData)
    ros.INFO("FT_INTERFACE: " .. method .. " has been set as filter")
  else
    local pastData = createInitPreviousTable(method,firstData,secondData, 0.5)
    self.filter = smooth:new("exp1", {["alpha"] = 0.5}, 6, currentData)
    ros.INFO("FT_INTERFACE: " .. "Received method is unknown, exp1 has been set as filter instead")
  end
  self.filter_active = true
end

return ForceTorqueInterface
