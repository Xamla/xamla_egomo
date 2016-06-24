-- module structureio_interface

local ros = require 'ros'
local ffi = require 'ffi'
local imageHandler = require 'image'

local pcl = require 'pcl'


local cv = require 'cv'
require 'cv.highgui'
require 'cv.imgproc'

-- no more external access after this point
_ENV = nil


local StructureIOInterface = {
   nodeID=nil,
   topicDepthCompressed=nil,
   serviceGetNewImg=nil,
   serviceSetResolution=nil,
   camIntrinsicsIR=nil,
}


function StructureIOInterface:new (camIntrinsicMatrix, nodeIDstring)
   local o = {}
   setmetatable(o, self)
   self.__index = self

   -- intrinsic matrix is only required for the point cloud calculation, so it's an optional parameter
   if not (camIntrinsicMatrix == nil) then
      self.camIntrinsicsIR = camIntrinsicMatrix:type("torch.FloatTensor"):clone()
   end

   self.nodeID=nodeIDstring or "openni2_cam"
   return o
end


function StructureIOInterface:Connect()
   local nodehandle = ros.NodeHandle()
   self.serviceGetNewImg = nodehandle:serviceClient("/"..self.nodeID.."/get_new_image", "egomo_depthcam/GetNewImage")
   self.serviceSetResolution = nodehandle:serviceClient("/"..self.nodeID.."/set_parameters", "egomo_depthcam/SetParameters")
end


function StructureIOInterface:ConnectDepthStream()
   local nodehandle = ros.NodeHandle() -- generic node handle
   self.topicDepthCompressed = nodehandle:subscribe("/"..self.nodeID.."/depthstream/compressed", 'sensor_msgs/CompressedImage', 1)
   print("Subscribed to "..self.topicDepthCompressed:getTopic())
   if (self.topicDepthCompressed == nil) then
      print(string.format("Topic %s/depthstream/compressed not found", self.nodeID))
      return false
   end
   -- print(self.topicDepthCompressed.msg_spec)
   return true
end


function StructureIOInterface:SetIntrinsicMatrix(camIntrinsicMatrix)
   self.camIntrinsicsIR = camIntrinsicMatrix:clone()
end


function StructureIOInterface:GrabDepthStreamData()
   local img
   if self.topicDepthCompressed:hasMessage() then
      local msg = self.topicDepthCompressed:read()
      local imgDecomp = imageHandler.decompressPNG(msg.data)  -- msg.data contains the actual jpeg coded image
      img=imgDecomp:permute(2,3,1):clone()
   end
   return img
end


-- GrabIRNoSpeckleViaROS () and GrabIRDefaultImageViaROS ()
-- are and will ever be exactly the same functions. The only
-- difference is the first line. However, if one tries to merge
-- the two function into one, msgIRCam becomes nil all the time
-- Reason: Unkown?

function StructureIOInterface:GrabIRNoSpeckleViaROS ()
   -- set a message to the given service, in this case just an integer 1 to request a new image
   print("This is GrabIRNoSpeckleViaROS")
   local msgIRCam = self.serviceGetNewImg:call(2)

   if not (msgIRCam == nil) then
    -- image conversion: 2*8 bit to 16 bit image
    local imgWidth = msgIRCam.img.width
    local imgHeight = msgIRCam.img.height
    
     local imageIRfull = torch.ShortTensor(imgHeight, imgWidth, 1)
    ffi.copy(imageIRfull:storage():data(), msgIRCam.img.data:storage():data(), imgHeight*imgWidth*2)

    -- example png decompression
    --local imageIRfull = imageHandler.decompressPNG(msgIRCam.img.data)
    --imageIRfull=imageIRfull:permute(2,3,1):clone()

    local imageIR8bit = imageIRfull:type('torch.FloatTensor')
    local a=imageIR8bit:max()
    imageIR8bit:mul(255.0/a)
    imageIR8bit:clamp(0,255)
    imageIR8bit = imageIR8bit:type('torch.ByteTensor')
    return imageIR8bit, imageIRfull
  else
    return nil
  end

  return nil
end


function StructureIOInterface:GrabIRDefaultImageViaROS ()
   local msgIRCam=self.serviceGetNewImg:call(0)

   if not (msgIRCam == nil) then
    -- image conversion: 2*8 bit to 16 bit image
    local imgWidth = msgIRCam.img.width
    local imgHeight = msgIRCam.img.height

    local imageIRfull = torch.ShortTensor(imgHeight, imgWidth, 1)
    ffi.copy(imageIRfull:storage():data(), msgIRCam.img.data:storage():data(), imgHeight*imgWidth*2)

    local imageIR8bit = imageIRfull:type('torch.FloatTensor')
    local a=imageIR8bit:max()
    imageIR8bit:mul(255.0/a)
    imageIR8bit:clamp(0,255)
    imageIR8bit = imageIR8bit:type('torch.ByteTensor')
    return imageIR8bit, imageIRfull
  else
    return nil
  end
end


function StructureIOInterface:SetProjectorStatus(status)
   -- get a message template of that service, response is available via respense_spec
  local m=ros.Message(self.serviceSetResolution.spec.request_spec)
  m.camStream = -1 -- do not change the resolution of any of the streams
  m.imgWidth=0
  m.imgHeight=0
  if(status == true) then
     m.irProjector=1
  else
     m.irProjector=0
  end

  -- send the message
  local msgResolution=self.serviceSetResolution:call(m)
  if not (msgResolution == nil) then
    print(string.format("IR projector status set to %s", tostring(status)))
  else
    print("Set projector status failed.")
    return false
  end

  return true
end


function StructureIOInterface:GrabDepthImageViaROS ()
local timer = torch.Timer()
  local msgDepthCam=self.serviceGetNewImg:call(5) -- set a message to the given service, in this case just an integer 1 to request a new image
  timer:stop()
  --print(string.format("Grabbing depth image took %.1f ms", timer:time().real*1000))

  timer:reset()
  timer:resume()
  if not (msgDepthCam == nil) then
    -- image conversion: 2*8 bit to 16 bit image
    local imgWidth = msgDepthCam.img.width
    local imgHeight = msgDepthCam.img.height
    local imageDepth = torch.ShortTensor(imgHeight, imgWidth, 1)
    ffi.copy(imageDepth:storage():data(), msgDepthCam.img.data:storage():data(), imgHeight*imgWidth*2)
    imageDepth = imageDepth:type('torch.FloatTensor')
    imageDepth:mul(1.0/1000) -- convert from mm to meter
    timer:stop()
    --print(string.format("Scaling depth image took %.1f ms", timer:time().real*1000))
    return imageDepth
  else
    print("Grabbing depth image failed!")
    return nil
  end
end


function StructureIOInterface:BuildCloudVersB(depthImg)
  local camIntrinsicsIRinverse = torch.inverse(self.camIntrinsicsIR)
  local xResolution=depthImg:size(2)
  local yResolution=depthImg:size(1)

  --print(xResolution.."x"..yResolution)

  --print("center pos: "..depthImg[240][320][1])

  local z=torch.FloatTensor(yResolution, xResolution, 3)
  z[{{}, {}, 3}]=1
  z[{{}, {}, 1}]=torch.linspace(0.5, xResolution-0.5, xResolution):view(1,xResolution,1):expand(yResolution, xResolution,1)
  z[{{}, {}, 2}]=torch.linspace(0.5, yResolution-0.5, yResolution):view(yResolution,1,1):expand(yResolution, xResolution,1)

--  print(z[{{1,10}, {1,10}, {1,3}}])

  --local result=camIntrinsicsIRinverse * z:view(xResolution*yResolution, 3):t()
  local result= z:view(xResolution*yResolution, 3) * camIntrinsicsIRinverse:t()
  --result=result:t():clone()

  local mask = result[{{}, 3}]:le(0):view(result:size(1),1):expand(result:size())
  --result[mask]=0/0

  local depthExpanded=depthImg:view(result:size(1),1):expand(result:size())
  result:cmul(depthExpanded)

  local newCloud = torch.FloatTensor(yResolution, xResolution, 4)
  newCloud[{{}, {}, 4}]=1
  newCloud[{{}, {}, {1,3}}] = result:view(yResolution,  xResolution, 3)

  return pcl.PointCloud('xyz', newCloud)
end


function StructureIOInterface:BuildCloudVersA(depthImg)
  local camIntrinsicsIRinverse = torch.inverse(self.camIntrinsicsIR)
  local xResolution=depthImg:size()[2]
  local yResolution=depthImg:size()[1]

  --print(xResolution.."x"..yResolution)

  --print("center pos: "..depthImg[240][320][1])

  local newCloud = torch.FloatTensor(yResolution, xResolution, 4)
  local depth
  local newPos=torch.FloatTensor(3)
  local nanVector = torch.FloatTensor({0/0, 0/0, 0/0})

  newCloud[{{}, {}, 4}]=1
  for x=1, xResolution do
    for y=1, yResolution do
      newPos = torch.mv(camIntrinsicsIRinverse, torch.FloatTensor{x, y, 1})
      newPos = torch.mul(newPos, depthImg[y][x][1])
      if newPos[3]<=0 then
       newPos=nanVector
      end
      newCloud[{y, x, {1,3}}]=newPos
    end
  end

  return pcl.PointCloud('xyz', newCloud)
end


function StructureIOInterface:GrabPointCloud ()
local timer = torch.Timer()
   local depthImg = self:GrabDepthImageViaROS()
    timer:stop()
    --print(string.format("Total time depth image %.1f ms", timer:time().real*1000))
    timer:reset()
    timer:resume()
   local cloud=self:BuildCloudVersB(depthImg)
   timer:stop()
    --print(string.format("Converting depth to cloud: %.1f ms", timer:time().real*1000))
   return cloud
end


function StructureIOInterface:SetResolution(camStream, imgWidth, imgHeight)
  -- get a message template of that service, response is available via respense_spec
  local m=ros.Message(self.serviceSetResolution.spec.request_spec)
  m.camStream = camStream
  m.imgWidth=imgWidth
  m.imgHeight=imgHeight
  m.irProjector=-1  -- do not change the IR projector status

  -- send the message
  local msgResolution=self.serviceSetResolution:call(m)
  if not (msgResolution == nil) then
    print("Resolution set to "..m.imgWidth.."x"..m.imgHeight)
  else
    print("Set resolution failed.")
    return false
  end

  return true
end


function StructureIOInterface:SetIRresolution(imgWidth, imgHeight)
   return self:SetResolution(1, imgWidth, imgHeight)
end

function StructureIOInterface:SetDepthResolution(imgWidth, imgHeight)
   return self:SetResolution(3, imgWidth, imgHeight)
end


return StructureIOInterface
