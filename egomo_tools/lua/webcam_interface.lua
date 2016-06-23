-- module webcam_interface

-- Import section
local ros = require 'ros'
local imageHandler = require 'image'
local imgConv = require 'cv.imgproc'


-- no more external access after this point
_ENV = nil

--- 
-- This interface allows easy access to acquire images from egomo
-- It allows you to capture single images, to set the focus value of the camera
-- and to connect to a low-quality but high-framerate JPEG stream
--@classmod WebcamInterface 
local WebcamInterface = {
   nodeID=nil,
   topicImgCompressed=nil,
   serviceGetNewImg=nil,
   serviceSetFocus=nil,
}

---
-- Creates a new WebcamInterface object.
-- This function does not make any connections to ROS topics 
-- @param nodeIDstring the underlying ROS node name
-- @return the created Webcam interface object
function WebcamInterface:new (nodeIDstring)
   o = {}
   setmetatable(o, self)
   self.__index = self

   self.nodeID=nodeIDstring or "webcam"
   print("service base name set to "..self.nodeID)
   return o
end


---
-- Registers the interface to the JPEG stream on egomo sensor. If you want to capture the low quality livestream images 
-- you first have to connect to the jpeg stream. Later, you can call GrabJPEGstreamROS() to get an image 
-- @return false if jpeg stream could not be found, otherwise true
function WebcamInterface:ConnectToJpgStream()
   local nodehandle = ros.NodeHandle() -- generic node handle
   self.topicImgCompressed = nodehandle:subscribe("/"..self.nodeID.."/stream/compressed", 'sensor_msgs/CompressedImage', 1)
   print("Subscribed to "..self.topicImgCompressed:getTopic())
   if (self.topicImgCompressed == nil) then
      print(string.format("Topic %s/stream/compressed not found", self.nodeID))
      return false
   end
   -- print(self.topicImgCompressed.msg_spec)
   return true
end


---
-- Registers the interface to the service that allows you to capture single still images. Calling this function is a
-- requirement for grabbing still images by calling GrabGrayscaleImgROS () or GrabImgROS () 
-- @return false if the single image service is not found
function WebcamInterface:ConnectToSingleImage ()
   local nodehandle = ros.NodeHandle() -- generic node handle
   print(self.nodeID)
   self.serviceGetNewImg = nodehandle:serviceClient("/"..self.nodeID.."/get_new_image", "web_cam/GetNewImage")
   if (self.serviceGetNewImg == nil) then
      print(string.format("Service %s/get_new_image not found", self.nodeID))
      return false
   end
   -- print(self.serviceGetNewImg.spec)  -- Print the service specification (just for showing/debugging)
   return true
end

---
-- Registers the interface to the focus settings service. Calling this function is a requirement to call
-- SetFocusValue (focus)
-- @return false if the focus service is not found otherwise true
function WebcamInterface:ConnectToServiceFocus ()
   local nodehandle = ros.NodeHandle() -- generic node handle
   self.serviceSetFocus = nodehandle:serviceClient("/"..self.nodeID.."/set_focus", "web_cam/SetCameraFocus")
   if (self.serviceSetFocus == nil) then
      print(string.format("Service %s/set_focus not found", self.nodeID))
      return false
   end
   -- print(self.serviceSetFocus.spec)  -- Print the service specification (just for showing/debugging)
   return true
end


--- 
-- Connects the interface to the single image service and the focus setting service. Its the same as
-- calling ConnectToSingleImage() and ConnectToServiceFocus()
function WebcamInterface:ConnectDefault ()
   self:ConnectToSingleImage()
   self:ConnectToServiceFocus()
   -- do not connect to JPG stream by default because that stream uses quite some bandwidth on the PI WLAN
end


--- 
-- Sets a focus value to the camera
-- @param focus focus setting between 0 and 255
function WebcamInterface:SetFocusValue (focus)

  -- get a message template of that service, response is available via respense_spec
  local m=ros.Message(self.serviceSetFocus.spec.request_spec) 
  m.autofocus=false -- disable autofocus
  m.focusValue=focus   -- set a new focus value (=lense position)

  -- send the message
  local msgFocus=self.serviceSetFocus:call(m)
  if not (msgFocus == nil) then
    print("Camera focus set to " .. msgFocus.result)
  else
    print("Set camera focus failed")
    return false
  end

  return true
end


---
-- Activates the autofocus of the camera
function WebcamInterface:EnableAutoFocus ()
   local m=ros.Message(self.serviceSetFocus.spec.request_spec) -- get a message template of that service
  m.autofocus=true -- enable autofocus
  m.focusValue=0   -- arbitrary number, will be ignored

-- send the message
  local msgFocus=self.serviceSetFocus:call(m)
  if not (msgFocus == nil) then
    print("Autofocus enabled")
  else
    print("Enabling autofocus failed")
    return false
  end

  return true
end


---
-- Grab an via a service call and convert it from YUYV to Gray (take only Y channel). ConnectToSingleImage () has to be called once before
-- @return the image as torch.ByteTensor
function WebcamInterface:GrabGrayscaleImgROS ()

  -- set a message to the given service, in this case just an integer 1 to request a new image
  local msgWebCam=self.serviceGetNewImg:call(1) 
  if not (msgWebCam == nil) then
    -- image conversion: yuv to grayscale (only luminance (y) channel
    local input = msgWebCam.img.data:clone()
    local imgWidth = msgWebCam.img.width
    local imgHeight = msgWebCam.img.height
    
    local x = input:view(imgHeight, imgWidth, 2)
    local resultImg = x[{{},{},1}]:clone() -- fill Y part
    return resultImg
  else
    return nil
  end
end


---
-- Grab an via a service call and convert it from YUYV to RGB. ConnectToSingleImage () has to be called once before
-- @return the image as torch.Byte tensor with dimension (rows,cols,3)
function WebcamInterface:GrabRGBImgROS ()
   -- Grab an via a service call and convert it from YUYV to RGB

  -- set a message to the given service, in this case just an integer 1 to request a new image
  local msgWebCam=self.serviceGetNewImg:call(1) 

  if not (msgWebCam == nil) then
    -- image conversion: yuyv to bgr
    local u = msgWebCam.img.data:clone()
    local x = u:reshape(msgWebCam.img.height, msgWebCam.img.width,2)
    local imageRGB = imgConv.cvtColor{x, nil, imgConv.COLOR_YUV2BGR_YUYV}

    return imageRGB
  else
    return nil
  end
end


---
-- Grabs an image from JPEG stream. ConnectToJpgStream() has to be called before.
-- @return the image as torch.ByteTensor
function WebcamInterface:GrabJPEGstreamROS ()
   local img
   if self.topicImgCompressed:hasMessage() then
      local msg = self.topicImgCompressed:read()
      local imgDecomp = imageHandler.decompressJPG(msg.data)  -- msg.data contains the actual jpeg coded image
      local imgColSwap = imgDecomp:clone()
      imgColSwap[{1, {}, {}}] = imgDecomp[{3, {}, {}}]  -- swap red and blue channel 
      imgColSwap[{3, {}, {}}] = imgDecomp[{1, {}, {}}]
      img=imgColSwap:permute(2,3,1):clone()
      img = (img*255):type('torch.ByteTensor')
      -- TODO: convert from RGB to BGR for openCV
   end
   return img
end


return WebcamInterface
