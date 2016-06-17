local egomoTools = require 'egomo-tools.env'

-- list of the provided sub-modules
local validInterfaces = {
   gripper="gripper_interface",
   robot="robot_control",
   structureio="structureio_interface",
   webcam="webcam_interface",
   force_torque="force_torque_interface",
   compressionLZ4="compressionLZ4_interface"
}


-- load the sub-modules on first access
local mt = {}
function mt:__index(key)
   local value=rawget(self, key)
   print("value is ")
   print(value)
   if value == nil and validInterfaces[key] ~= nil then
      print("Load interface "..key)
      value = require ('egomo-tools.'..validInterfaces[key])
      rawset(egomoTools, key, value)
   end
   return value
end
   
setmetatable( egomoTools, mt )

return egomoTools
