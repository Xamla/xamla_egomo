local torch = require 'torch'
local pcl = require 'pcl'

local ObjectsOnPlaneSegmentation = torch.class('ObjectsOnPlaneSegmentation')

ObjectsOnPlaneSegmentation.defaultParameters = {
  crop_radius = 1,              -- remove all elements from input with distance > 1m from origin (camera)
  distance_threshold = 0.015, 
  cluster_tolerance = 0.02,     -- points closer than 2cm will be considered to belong to the same cluster
  min_cluster_size = 300,       -- minimum number of points for a cluster to be considered
  max_cluster_size = 25000      -- maximum number of points for a cluster to be considered
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
end

-- internal processing functions (private)

local function preprocess(self, cloud)
  cloud:removeNaN()
  if self.p.crop_radius then
    cloud = pcl.filter.cropSphere(cloud, {0,0,0,1}, self.p.crop_radius)
  end
  return cloud
end


function axisAngleToRotMatrix(axis, angle)
   -- formula taken from http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToMatrix/
   axisNorm = axis:clone()
   axisNorm:mul(math.sqrt(1.0/(axis[1]*axis[1]+axis[2]*axis[2]+axis[3]*axis[3])))
   local c = math.cos(angle)
   local s = math.sin(angle)
   local t = 1-c
   local x, y, z = axisNorm[1], axisNorm[2], axisNorm[3]
--[[
   print("axis")
   print(axis)
   print("axisNorm")
   print(axisNorm)
   print("angle="..angle) 
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


local function detectPlane(self, cloud)
   
  -- detect the plane of the table
  local segmentation = pcl.SACSegmentation('XYZ')
  segmentation:setOptimizeCoefficients(true)
  segmentation:setModelType(pcl.SACMODEL.PLANE)
  segmentation:setMethodType(pcl.SAC.RANSAC)
  segmentation:setDistanceThreshold(self.p.distance_threshold); -- Parameter sensitiv wegen der Unebenheiten im Tisch
  segmentation:setInputCloud(cloud)

  -- extract the plane coefficients in ax + by + cz + d = 0 form.
  local inliers = pcl.Indices()
  local coefficients = torch.FloatTensor()
  segmentation:segment(inliers, coefficients)

  -- store all plane cloud points and all non-plane cloud points in two new, separate point clouds
  local plane = pcl.filter.extractIndices(cloud, inliers, nil, false)
  local objects = pcl.filter.extractIndices(cloud, inliers, nil, true)

  -- rotate the object point cloud so that the calculated plane is exactly perpendicular to the z axis
  local up=torch.FloatTensor({0, 0, 1})
  local planeNormal = torch.FloatTensor({coefficients[1], coefficients[2], coefficients[3]})
  local rotAxis = planeNormal:cross(up)
  local rotAngle = math.acos((planeNormal:dot(up))/planeNormal:norm())
  local rotMatrix = axisAngleToRotMatrix(rotAxis, rotAngle)
  objects:transform(rotMatrix)

  -- remove all points which are below or less then 5 mm above the calculated plane
  local objCleaned = pcl.filter.passThrough(objects, 'z', 0.1, ((-1)*coefficients[4])-0.005)
  rotMatrix = axisAngleToRotMatrix(rotAxis, rotAngle*(-1)) -- rotate clouds back
  objects:transform(rotMatrix)
  objCleaned:transform(rotMatrix)

--[[
  print("plane coefficients: ")
  print(coefficients)
  print("plane normal: ")
  print(planeNormal)
--]]

  return objCleaned, plane, objects
end


local function findObjectClusters(self, objects)
  local kdTree = pcl.KdTree()
  kdTree:setInputCloud(objects)
  local objectClustering = pcl.EuclideanClusterExtraction()
  objectClustering:setClusterTolerance(self.p.cluster_tolerance);
  objectClustering:setMinClusterSize(self.p.min_cluster_size);
  objectClustering:setMaxClusterSize(self.p.max_cluster_size);
  objectClustering:setInputCloud(objects)
  return objectClustering:extract()
end


local function computeMeansAndEigenvectors(self, object_points, cluster_indices)
  local pca = pcl.PCA()
  numberOfClusters = cluster_indices:size()
  -- result
  local r = {
    firstEigenvectorOfEachCluster = torch.Tensor(numberOfClusters, 3),
    meanOfEachCluster = torch.Tensor(numberOfClusters, 4)
  }
  for i=1,numberOfClusters do
    pca:setInputCloud(object_points)
    pca:setIndices(cluster_indices[i])
    r.meanOfEachCluster[{i,{}}] = pca:getMean()
    r.firstEigenvectorOfEachCluster[{i,{}}] = pca:getEigenVectors()[{{}, 1}]    -- eigenvectors are the columns
  end
  return r
end

-- main public processing method to find objects on a plane in a given point cloud
function ObjectsOnPlaneSegmentation:process(cloud, printTiming)
  printTiming = printTiming or false
  local timer
  if printTiming then
     print("ObjectsOnPlaneSegmentation timing overview:")
     timer= torch.Timer()
     sys.tic()
  end
  cloud = preprocess(self, cloud)
  if printTiming then
     print(string.format("  preproces %.1f ms", sys.toc()*1000))
     sys.tic()
  end
  local objectsCleaned, plane, objects = detectPlane(self, cloud)
  if printTiming then
     print(string.format("  detect plane %.1f ms", sys.toc()*1000))
     sys.tic()
  end
  local clusterIndices = findObjectClusters(self, objectsCleaned)
  if printTiming then
     print(string.format("  find objects %.1f ms", sys.toc()*1000))
     sys.tic()
  end
  local result = computeMeansAndEigenvectors(self, objectsCleaned, clusterIndices)
  if printTiming then
     print(string.format("  computeMeansAndEigenvectors %.1f ms", sys.toc()*1000))
     timer:stop()
  end
  if printTiming then
     print(string.format("ObjectsOnPlaneSegmentation segmentation finished, took %.1f ms in total", timer:time().real*1000))
  end
  return result, objectsCleaned, clusterIndices
end
