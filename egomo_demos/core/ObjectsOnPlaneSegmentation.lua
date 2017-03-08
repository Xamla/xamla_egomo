local torch = require 'torch'
local pcl = require 'pcl'

local ObjectsOnPlaneSegmentation = torch.class('ObjectsOnPlaneSegmentation')

ObjectsOnPlaneSegmentation.defaultParameters = {
  cloudType = 'XYZ', -- type of the point cloud to be processed, typically 'XYZ' or 'XYZRGBA'
  cropRadius = 1,              -- remove all elements from input with distance > 1m from origin (camera)
  distanceThreshold = 0.015, 
  clusterTolerance = 0.02,     -- points closer than 2cm will be considered to belong to the same cluster
  minClusterSize = 300,       -- minimum number of points for a cluster to be considered
  maxClusterSize = 25000,      -- maximum number of points for a cluster to be considered
  minPointDistanceToPlane = 0.005,  -- points closer to calculated plane then this value will be considered plane points (in meter)
  maxPointDistanceToPlane = 0.5  -- points further away from the calculated plane then this value will be removed (in meter)

}

function ObjectsOnPlaneSegmentation:__init(parameters)
  -- fill in missing default values
  parameters = parameters or ObjectsOnPlaneSegmentation.defaultParameters
  for k,v in pairs(ObjectsOnPlaneSegmentation.defaultParameters) do
    if not parameters[k] then
      parameters[k] = v
    end
  end
  self.p = parameters

  self.cloudCleaned=nil      -- input cloud wiht NaN points removed
  self.planeCloud=nil        -- point cloud containing all points belonging to the detected plane
  self.planeRotAxis=nil      -- axis and angle to rotate the plane so it is perpendicular ...
  self.planeRotAngle=nil     --    ... to the z axis
  self.planeParameters=nil
  self.notPlaneCloud=nil     -- all points of the point cloud not assigend to the plane
  self.objCloud=nil          -- point cloud containing all the object clusters
  self.objIndices=nil        -- Array of indices of the individual objects in objCloud
  self.objMeans=nil          -- Spacial center of each object cluster
  self.objEigenVectors=nil   -- First Eigenvector of each object cluster
  self.objEigenVectorsMiddle=nil   -- Second Eigenvector of each object cluster
  self.objEigenVectorsMinor=nil   -- Third Eigenvector of each object cluster
end

-- internal processing functions (private)

local function preprocess(self, cloud)
  if self.p.cropRadius > 0 then
     print("Cropping cloud with radius "..self.p.cropRadius)
     self.cloudCleaned = pcl.filter.cropSphere(cloud, {0,0,0,1}, self.p.cropRadius)
     self.cloudCleaned:removeNaN()
  else
     self.cloudCleaned = cloud:clone()
     self.cloudCleaned:removeNaN()
  end
end


function ObjectsOnPlaneSegmentation:axisAngleToRotMatrix(axis, angle)
   -- formula taken from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
   --[[
      print("axisAngleToRotMatrix axis")
      print(axis)
      print("angle")
      print(angle)
   --]]
   local axisNorm = axis:clone()
   axisNorm:mul(math.sqrt(1.0/(axis[1]*axis[1]+axis[2]*axis[2]+axis[3]*axis[3])))
   local c = math.cos(angle)
   local s = math.sin(angle)
   local t = 1-c
   local x, y, z = axisNorm[1], axisNorm[2], axisNorm[3]
--[[
   print("axisNorm")
   print(axisNorm)
--]]

   local rotMatrix = torch.FloatTensor(4,4):zero()
   rotMatrix[4][4]=1

   rotMatrix[1][1]=t*x*x + c
   rotMatrix[1][2]=t*x*y - z*s
   rotMatrix[1][3]=t*x*z + y*s

   rotMatrix[2][1]=t*x*y + z*s
   rotMatrix[2][2]=t*y*y + c
   rotMatrix[2][3]=t*y*z - x*s

   rotMatrix[3][1]=t*x*z - y*s
   rotMatrix[3][2]=t*y*z + x*s
   rotMatrix[3][3]=t*z*z + c

   return rotMatrix
end


local function detectPlane(self)
   
  -- detect the plane of the table
  local segmentation = pcl.SACSegmentation(self.p.cloudType)
  segmentation:setOptimizeCoefficients(true)
  segmentation:setModelType(pcl.SACMODEL.PLANE)
  segmentation:setMethodType(pcl.SAC.RANSAC)
  segmentation:setDistanceThreshold(self.p.distanceThreshold); -- Parameter sensitiv wegen der Unebenheiten im Tisch
  segmentation:setInputCloud(self.cloudCleaned)

  -- extract the plane coefficients in ax + by + cz + d = 0 form.
  local inliers = pcl.Indices()
  local coefficients = torch.FloatTensor()
  segmentation:segment(inliers, coefficients)

  -- store all plane cloud points and all non-plane cloud points in two new, separate point clouds
  self.planeCloud = pcl.filter.extractIndices(self.cloudCleaned, inliers, nil, false)
  self.notPlaneCloud = pcl.filter.extractIndices(self.cloudCleaned, inliers, nil, true)

  -- rotate the object point cloud so that the calculated plane is exactly perpendicular to the z axis
  local up=torch.FloatTensor({0, 0, 1})
  local planeNormal = torch.FloatTensor({coefficients[1], coefficients[2], coefficients[3]})
  self.planeRotAxis = planeNormal:cross(up)
  self.planeRotAngle = math.acos((planeNormal:dot(up))/planeNormal:norm())
  local rotMatrix = self:axisAngleToRotMatrix(self.planeRotAxis, self.planeRotAngle)
  -- self.notPlaneCloud:transform(rotMatrix)

  -- remove all points which are below or less then 5 mm above the calculated plane

  print("cutting from "..((-1)*coefficients[4])-self.p.minPointDistanceToPlane.." to "..((-1)*coefficients[4])-self.p.maxPointDistanceToPlane)
  self.objCloud = pcl.filter.passThrough(self.notPlaneCloud, 'z', ((-1)*coefficients[4])-self.p.maxPointDistanceToPlane, ((-1)*coefficients[4])-self.p.minPointDistanceToPlane)

  -- rotate both clouds back
  rotMatrix = self:axisAngleToRotMatrix(self.planeRotAxis, self.planeRotAngle*(-1)) 
  -- self.notPlaneCloud:transform(rotMatrix)
  -- self.objCloud:transform(rotMatrix)

  self.planeParameters=coefficients

--[[
  print("plane cloud size: "..self.planeCloud:size())
  print("object cloud size (before cut): "..self.notPlaneCloud:size())
  print("object cloud size (after cut): "..self.objCloud:size())
  print("plane coefficients: ")
  print(coefficients)
  print("plane normal: ")
  print(planeNormal)
  print("rot axis")
  print(self.planeRotAxis)
  print("rot angle")
  print(self.planeRotAngle)
  
--]]
end


local function findObjectClusters(self)
  local kdTree = pcl.KdTree(self.p.cloudType)
  kdTree:setInputCloud(self.objCloud)
  local objectClustering = pcl.EuclideanClusterExtraction(self.p.cloudType)
  objectClustering:setClusterTolerance(self.p.clusterTolerance);
  objectClustering:setMinClusterSize(self.p.minClusterSize);
  objectClustering:setMaxClusterSize(self.p.maxClusterSize);
  objectClustering:setInputCloud(self.objCloud)
  self.objIndices = objectClustering:extract()
end


local function computeMeansAndEigenvectors(self)
  local pca = pcl.PCA(self.p.cloudType)
  local numberOfClusters = self.objIndices:size()

   self.objMeans = torch.Tensor(numberOfClusters, 4)
   self.objEigenVectors = torch.Tensor(numberOfClusters, 3)
   self.objEigenVectorsMiddle = torch.Tensor(numberOfClusters, 3)
   self.objEigenVectorsMinor = torch.Tensor(numberOfClusters, 3)

  for i=1,numberOfClusters do
    pca:setInputCloud(self.objCloud)
    pca:setIndices(self.objIndices[i])
    self.objMeans[{i,{}}] = pca:getMean()
    local eigenVectors = pca:getEigenVectors()
    self.objEigenVectors[{i,{}}] = eigenVectors[{{}, 1}]    -- eigenvectors are the columns
    self.objEigenVectorsMiddle[{i,{}}] = eigenVectors[{{}, 2}]    -- eigenvectors are the columns
    self.objEigenVectorsMinor[{i,{}}] = eigenVectors[{{}, 3}]    -- eigenvectors are the columns
  end
end


local function computeMeansAndEigenvectors2D(self)
  local pca = pcl.PCA(self.p.cloudType)
  local numberOfClusters = self.objIndices:size()

  self.objMeans = torch.Tensor(numberOfClusters, 4)
  self.objEigenVectors = torch.Tensor(numberOfClusters, 3)
  self.objEigenVectorsMiddle = torch.Tensor(numberOfClusters, 3)
  self.objEigenVectorsMinor = torch.Tensor(numberOfClusters, 3)

   local tmpCloud = self.objCloud:clone()
   tmpCloud:pointsXYZ()[{{}, {}, {3}}]=0

   cloudVis = pcl.PCLVisualizer('demo', true)
   cloudVis:addPointCloud(tmpCloud, "tmpCloud")
   cloudVis:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 0.6, 0.6, 0.6, cloudName)
   cloudVis:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 2, cloudName)
   cloudVis:spin()
   
  for i=1,numberOfClusters do
    pca:setInputCloud(self.objCloud)
    pca:setIndices(self.objIndices[i])
    self.objMeans[{i,{}}] = pca:getMean()
     
    pca:setInputCloud(tmpCloud)
    pca:setIndices(self.objIndices[i])
    local eigenVectors = pca:getEigenVectors()
    self.objEigenVectors[{i,{}}] = eigenVectors[{{}, 1}]    -- eigenvectors are the columns
    self.objEigenVectorsMiddle[{i,{}}] = eigenVectors[{{}, 2}]    -- eigenvectors are the columns
    --self.objEigenVectorsMinor[{i,{}}] = nil    -- eigenvectors are the columns
  end
end



-- main public processing method to find objects on a plane in a given point cloud
function ObjectsOnPlaneSegmentation:process(cloud, printTiming)
  self:resetCloudData()

  printTiming = printTiming or false
  local timer
  if printTiming then
     print("ObjectsOnPlaneSegmentation timing overview:")
     timer= torch.Timer()
     sys.tic()
  end
  preprocess(self, cloud)
  if printTiming then
     print(string.format("  preproces %.1f ms", sys.toc()*1000))
     sys.tic()
  end
  detectPlane(self)
  if printTiming then
     print(string.format("  detect plane %.1f ms", sys.toc()*1000))
     sys.tic()
  end
  findObjectClusters(self)
  if printTiming then
     print(string.format("  find objects %.1f ms", sys.toc()*1000))
     sys.tic()
  end

  local result = computeMeansAndEigenvectors(self)
  if printTiming then
     print(string.format("  computeMeansAndEigenvectors %.1f ms", sys.toc()*1000))
     timer:stop()
  end
  if printTiming then
     print(string.format("ObjectsOnPlaneSegmentation segmentation finished, took %.1f ms in total", timer:time().real*1000))
  end
end


-- Resets all variables containing data of the last cloud processed
function ObjectsOnPlaneSegmentation:resetCloudData() 
  self.cloudCleaned=nil
  self.planeCloud=nil
  self.planeRotAxis=nil
  self.planeRotAngle=nil
  self.notPlaneCloud=nil
  self.objCloud=nil
  self.objIndices=nil
  self.objMeans=nil
  self.objEigenVectors=nil
  self.objEigenVectorsMiddle=nil
  self.objEigenVectorsMinor=nil

end

return ObjectsOnPlaneSegmentation
