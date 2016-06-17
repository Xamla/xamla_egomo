local cv = require 'cv'
require '../core/xamla3d'
require 'cv.highgui'
require 'cv.videoio'
require 'cv.imgproc'
require 'cv.calib3d'
require 'cv.imgcodecs'
require 'cv.features2d'

local duplo = {}

duplo.directory = '/home/hoppe/data/duplo_accuracy_test/jitter/duplo1/'
duplo.rMin = 15 --radius min (pixel in image of brick nobs)
duplo.rMax = 40 -- radius max (pixel in image of brick nobs)
duplo.circleGradientThreshold = 30 -- Threshold for finding circles. If lower more circles are found
duplo.distCoeffs = torch.zeros(5,1) -- Distortion coefficients of rgb camera
duplo.expectedZ = 3 -- We expect that out duplo is located on the table at z = duplo.expectedZ (in mm)

duplo.intrinsics = torch.DoubleTensor({  -- Intrinsic camera matrix of rgb camera (C920)
 { 918.3122, 0.0000,  481.8074},
 { 0.0000,   917.5487,  359.0547},
 { 0.0000,   0.0000 ,   1.0000}
})

duplo.handEye = torch.DoubleTensor({        -- HandEye of rgb camera
  {  0.0025,  0.7642,  0.6450,  0.0152 },
  { -0.0007, -0.6456,  0.7637,  0.0699 },
  {  1.0000,  0.0010,  0.0018,  0.0560 },
  {  0.0000,  0.0000,  0.0000,  1.0000 }
})

duplo.handEye[{{1,3},4}] = duplo.handEye[{{1,3},4}] * 1000 --handEye is in mm

duplo.images = {}
duplo.robotPose = {}
duplo.files = {}
duplo.triangulationData = {}

function duplo.initOffline()
  duplo.robotPose = {}
  duplo.files = {}
  duplo.triangulationData = {}
  print(duplo.directory)
  local success, files = xamla3d.utils.readFileList(duplo.directory, 'web', 'png')
  if not success then
    print("Did not found files in directory ".. duplo.directory)
  end
  duplo.files = files
  duplo.robotPose = torch.load(duplo.directory.."/duplo.t7").MoveitPose

end

function duplo.compare(a,b)
  return a[1] < b[1]
end

function duplo.inTable(tbl, item)
    for key, value in pairs(tbl) do
        if value == item then return key end
    end
    return false
end


--- 
-- creates a 3x4 projection matrix given the robot pose, the handeye and the intrinsic parameters
-- @param robotPose 4x4 torch.DoubleTensor pose of the TCP 
-- @param handEye 4x4 torch.DoubleTensor hand-eye matrix of rgb camera w.r.t. TCP
-- @intrinsic 3x3 intrinsic matrix of rgb camera
function duplo.createProjectionMatrix3x4(robotPose, handEye, intrinsic)
  local cam_inv = torch.inverse(robotPose * handEye)
  local cam_pose3x4 = cam_inv[{{1,3},{}}]
  local P = intrinsic * cam_pose3x4
  return P
end


---
-- Generates the 3D positions (in mm) of the Duplos knobs
-- @param rows number of knob rows on the Duplo brick
-- @param cols number of knob cols on the Duplo brick
-- @param height height of the Duplo (in mm, typically 19mm)
function duplo.createDuploCenters(rows, cols, height)
  local duploCenters = torch.DoubleTensor(rows*cols, 3)

  for i = 1,rows do
    for j = 1, cols do
      local x = (j-1)*16
      local y = (i-1)*16
      duploCenters[{{(i-1)*cols + j},{}}] = torch.DoubleTensor({x,y,height})
    end
  end

  return duploCenters
end


function duplo.getOrderedCircleCenters(image)
  local image_gray
  if image:nDimension() == 3 then
    image_gray = cv.cvtColor{src = image, code = cv.COLOR_BGR2GRAY}
  else
    image_gray = image
  end

  --local circles = cv.HoughCircles{image = image_gray, method = cv.HOUGH_GRADIENT, dp=1, param1 = 40, param2 = 40, minDist = 20}
  local circles = cv.HoughCircles{image = image_gray, method = cv.HOUGH_GRADIENT, dp=1, param1 = duplo.circleGradientThreshold, param2 = duplo.circleGradientThreshold, minDist = duplo.rMax, minRadius = duplo.rMin, maxRadius = duplo.rMax}
  if circles:dim() > 2 then

    local orderedPts = {}

    --local idx = torch.randperm(circles:size()[2])

    local d = 0
    for j = 1,circles:size(2) do
      --if j ~= idx[1] then
        cv.circle{img = image, center ={x=circles[{1,j,1}], y=circles[{1,j,2}]}, radius = circles[{1,j,3}], color = {255,255,255} }
        cv.circle{img = image, center ={x=circles[{1,j,1}], y=circles[{1,j,2}]}, radius = 2, color = {0,255,0} }
        local x ={}
        x.x = circles[{1,j,1}]
        x.y = circles[{1,j,2}]
        d = d + circles[{1,j,3}]
        table.insert(orderedPts, x)
       -- end
    end

    -- AKo: added very simple ordering scheme
    local avg_diameter = d / circles:size(2)
    table.sort(orderedPts, function(a,b) return a.y < b.y and math.abs(a.y - b.y) > avg_diameter or math.abs(a.y - b.y) <= avg_diameter and a.x < b.x end)

    --orderedPts, orderedLines = duplo.createAllPotentialLines(circles)

    for i = 1,#orderedPts do
             cv.putText{image, text= i.." ", org={x=orderedPts[i].x+5, y=orderedPts[i].y}, fontFace=cv.FONT_HERSHEY_SIMPLEX,
                fontScale=0.4, color={255, 255, 0}, thickness=1}

    end
    return true, orderedPts

  end -- of circle dim > 2
  return false, nil
end


---
-- Resets all internal datastructures
function duplo.reset()
  duplo.robotPose = {}
  duplo.triangulationData = {}
  duplo.images = {}
  duplo.files = {}
end

--- 
-- Adds an image (containing the duplo bricks) to this module 
-- @param image the image of the brick
-- @param 4x4 torch.Tensor robot TCP
function duplo.addImage(image, robotPoseTC)

  local image_undist = cv.undistort{src = image, cameraMatrix = duplo.intrinsics, distCoeffs = duplo.distCoeffs}
  print('undistorted...')
  local found, detectedCircles = duplo.getOrderedCircleCenters(image_undist)
  if found then
    print('found circles')
    local tmp = robotPoseTC.full:clone()
    tmp[{{1,3}, 4}] =  tmp[{{1,3}, 4}] *1000
    local triangStructure = {}
    triangStructure.robotPose = tmp:clone()
    triangStructure.pts2d = detectedCircles
    table.insert(duplo.triangulationData, triangStructure)
    --cv.imshow{"DetectedCircles",image_undist}
    table.insert(duplo.images, image_undist:clone())
    return true
  else
    print("Circles not found :-( ");
    return false
  end
end


---
-- Makes a deepcopy from a table
-- @param orig table 
-- @return a deepcopy element from the original table 
local function deepcopy(orig)
    local orig_type = type(orig)
    local copy
    if orig_type == 'table' then
        copy = {}
        for orig_key, orig_value in next, orig, nil do
            copy[deepcopy(orig_key)] = deepcopy(orig_value)
        end
        setmetatable(copy, deepcopy(getmetatable(orig)))
    else -- number, string, boolean, etc
        copy = orig
    end
    return copy
end


---
-- Makes pairwise matching of detected circles using epiploar geometry and a prior-knowlege of Z position of the 3d points
-- @param expectedMinZ minimum z coordinate of a point in robot base coordinates (in mm)
-- @param expectedMaxZ maximum z coordinate of a point in robot base coordinates (in mm)
function duplo.matchByEpipolarGeometry(expectedMinZ, expectedMaxZ)

  assert(expectedMinZ ~= nil)
  assert(expectedMaxZ ~= nil)
  assert(expectedMaxZ >= expectedMinZ)


  local matches = {}
  local sum_sampson = 0

  for i = 1,#duplo.triangulationData do
    table.insert(matches, {})

    for j = 1,#duplo.triangulationData do
      table.insert(matches[i],{})

      if j > i then

          local robotP_i = duplo.triangulationData[i].robotPose
          local robotP_j = duplo.triangulationData[j].robotPose
          local pose_i = torch.inverse(robotP_i*duplo.handEye)
          local pose_j = torch.inverse(robotP_j*duplo.handEye)

          local ProjI = duplo.intrinsics * pose_i[{{1,3},{}}]
          local ProjJ = duplo.intrinsics * pose_j[{{1,3},{}}]

          local F = xamla3d.getFundamentalMatrixFromFullPose(pose_i, pose_j, duplo.intrinsics)

          for m_i = 1, #duplo.triangulationData[i].pts2d do

             local min_disparity = 10000000
             local min_dist_idx = 0

            for m_j = 1, #duplo.triangulationData[j].pts2d do

               local p1 = torch.DoubleTensor(3,1)
               local p2 = torch.DoubleTensor(3,1)

               p1[{{1,3},1}] = torch.DoubleTensor({duplo.triangulationData[i].pts2d[m_i].x, duplo.triangulationData[i].pts2d[m_i].y,1})
               p2[{{1,3},1}] = torch.DoubleTensor({duplo.triangulationData[j].pts2d[m_j].x, duplo.triangulationData[j].pts2d[m_j].y,1})
               -- Calculate the fundamental mat
               local d = math.abs(xamla3d.sampsonDistance(p1, p2, F))

               if (d < 200) then
                 local meas = {}
                 table.insert(meas, p1[{{1,2}, 1}]:view(1,2):clone())
                 table.insert(meas, p2[{{1,2}, 1}]:view(1,2):clone())

                 local Proj = {}
                 table.insert(Proj, ProjI)
                 table.insert(Proj, ProjJ)
                 local s, X = xamla3d.linearTriangulation(Proj, meas)
                 if ( X[3][1] > expectedMinZ and X[3][1] < expectedMaxZ) then
                    local m = {}
                    m.imageID1 = i
                    m.imageID2 = j
                    m.idx1 = m_i
                    m.idx2 = m_j
                    table.insert(matches[i][j], m)
                 end
               end
            end
          end
      end
    end
  end


  for j = 1,#matches[1][2] do
    local m = matches[1][2][j];
    local cx1 = duplo.triangulationData[m.imageID1].pts2d[m.idx1].x
    local cy1 = duplo.triangulationData[m.imageID1].pts2d[m.idx1].y
    local cx2 = duplo.triangulationData[m.imageID2].pts2d[m.idx2].x
    local cy2 = duplo.triangulationData[m.imageID2].pts2d[m.idx2].y

    local r = torch.rand(1)[1]*255
    local g = torch.rand(1)[1]*255
    local b = torch.rand(1)[1]*255

    local offset =(0.5-torch.rand(1)[1])*10

    cv.circle{img = duplo.images[m.imageID1], center ={x=cx1+offset, y=cy1+offset}, radius = 5, thickness= 10, color = {r,g,b} }
    cv.circle{img = duplo.images[m.imageID2], center ={x=cx2+offset, y=cy2+offset}, radius = 5, thickness= 10, color = {r,g,b} }
  end

  cv.imshow{"Matches 1", duplo.images[1]}
  cv.imshow{"Matches 2", duplo.images[2]}
  cv.waitKey{30}

  return matches
end


function duplo.isInLinkedMeasurementList(linkedMeasurements, image_id, measurement_id)

  for i = 1,#linkedMeasurements do
    for j = 1,#linkedMeasurements[i] do
      if linkedMeasurements[i][j].imageId == image_id and
         linkedMeasurements[i][j].measurementId == measurement_id then
         return true, i
       end
    end
  end
  return false, nil
end

function duplo.hasAlreadyMeasurementOfThisImage(linkedMeasurements, pointID, imageID)
  for i = 1,#linkedMeasurements[pointID] do
    local m = linkedMeasurements[pointID][i]
    if m.imageId == imageID then
      return true
    end
  end
  return false

end


--- 
-- Links pairwise matches of "features/correspondences" together
-- Example: Feature A has been matches in Imagepair  (1, 2) and in image pair (2,3)
-- The output is that Feature A is visible in image (1,2,3)
function duplo.linkMeasurements(matches)
  local linkedMeasurements = {}

  for i = 1,#matches do
    for j = 1,#matches[i] do
      local mi = matches[i][j]


      for k = 1,#mi do
         local m = mi[k]
         local inMeas1, id1 = duplo.isInLinkedMeasurementList(linkedMeasurements, m.imageID1, m.idx1)
         local inMeas2, id2 = duplo.isInLinkedMeasurementList(linkedMeasurements, m.imageID2, m.idx2)
         -- in case first measurement is already in and second is not, we add the second one
         if (inMeas1== true and inMeas2 == false) then
            if (duplo.hasAlreadyMeasurementOfThisImage(linkedMeasurements, id1, m.imageID2)) then
             -- we create for both measurements a new 3d point
              --print("Duplicate!! 1")
              local dummy = {}
              table.insert(linkedMeasurements, dummy)
              local new_id = #linkedMeasurements
              local t = {}
              t["imageId"] = m.imageID1
              t["measurementId"] = m.idx1
              table.insert(linkedMeasurements[new_id], t)
              local t2 = {}
              t2["imageId"] = m.imageID2
              t2["measurementId"] = m.idx2
              table.insert(linkedMeasurements[new_id], t2)
            else -- we just add the new measurement to the existing point
              --print("Increase 1")
              local t = {}
              t["imageId"]= m.imageID2
              t["measurementId"] = m.idx2
              table.insert(linkedMeasurements[id1], t)
            end

         end
         -- in case second is in, but not first one
         if (inMeas1 == false and inMeas2 == true) then
            if (duplo.hasAlreadyMeasurementOfThisImage(linkedMeasurements, id2, m.imageID1)) then
             -- we create for both measurements a new 3d point
              --print("Duplicate!! 2")
              local dummy = {}
              table.insert(linkedMeasurements, dummy)
              local new_id = #linkedMeasurements
              local t = {}
              t["imageId"] = m.imageID1
              t["measurementId"] = m.idx1
              table.insert(linkedMeasurements[new_id], t)
              local t2 = {}
              t2["imageId"] = m.imageID2
              t2["measurementId"] = m.idx2
              table.insert(linkedMeasurements[new_id], t2)
            else
              --print("Increase 2")
              local t = {}
              t["imageId"] = m.imageID1
              t["measurementId"] = m.idx1
              table.insert(linkedMeasurements[id2], t)
            end
         end

         -- none of them is in, we add both
         if (inMeas1 == false and inMeas2 == false) then
            --print("New!!")
            local dummy = {}
            table.insert(linkedMeasurements, dummy)
            local new_id = #linkedMeasurements
            local t = {}
            t["imageId"] = m.imageID1
            t["measurementId"] = m.idx1
            table.insert(linkedMeasurements[new_id], t)
            local t2 = {}
            t2["imageId"] = m.imageID2
            t2["measurementId"] = m.idx2
            table.insert(linkedMeasurements[new_id], t2)
         end
         if (inMeas1 == true and inMeas2 == true and id1 ~= id2) then
            --print("New-------------------_!")
            local dummy = {}
            table.insert(linkedMeasurements, dummy)
            local new_id = #linkedMeasurements
            local t = {}
            t["imageId"] = m.imageID1
            t["measurementId"] = m.idx1
            table.insert(linkedMeasurements[new_id], t)
            local t2 = {}
            t2["imageId"] = m.imageID2
            t2["measurementId"] = m.idx2
            table.insert(linkedMeasurements[new_id], t2)
         end
         ---
      end --k
    end --j
  end --i
  --duplo.printLinkedMeasurements(linkedMeasurements)
  return linkedMeasurements
end


--- 
-- Debugging the linked measurement list
function duplo.printLinkedMeasurements(linkedMeasurements)
  for i = 1,#linkedMeasurements do
  io.write("Point id "..i .."| ")
    for j = 1,#linkedMeasurements[i] do
      io.write(linkedMeasurements[i][j].imageId .."->"..linkedMeasurements[i][j].measurementId .." ")
    end
    io.write("\n")
  end
end

---
-- Main function for triangulation
-- @param minZ minimum Z coordinate of the triangulated point (a-priori knowledge)
-- @param maxZ maximum Z coordinate of the triangulated point (a-priori knowledge)
-- @param verbose if the function should output debugging information
function duplo.triangulatePoints(minZ, maxZ, verbose)
 if #duplo.triangulationData < 2 then
    return false, nil
 end

 -- find the matches (image pairwise) by assuming that the z coordinate of the upper edge of the brick is located within minZ and maxZ
 local matches = duplo.matchByEpipolarGeometry(minZ, maxZ) 
 -- linkes the pairwise matches together
 local linkedMatches = duplo.linkMeasurements(matches)

 
 print("Potential 3D Points: "..#linkedMatches)
 
 -- Triangulate the all image measurements using a linear triangulation method
 local allPts3d = {}
 for i = 1,#linkedMatches do
  local matches = linkedMatches[i]

  local P = {}
  local meas = {}
  ----- Stack all P matrices and measurements
  for j = 1,#matches do
    local imgId = matches[j].imageId
    local measId = matches[j].measurementId
    local robotP = duplo.triangulationData[imgId].robotPose
    local p2d = duplo.triangulationData[imgId].pts2d[measId]
    table.insert(P, duplo.createProjectionMatrix3x4(robotP, duplo.handEye, duplo.intrinsics))
    local m = torch.DoubleTensor(1,2)
    m[1][1] = p2d.x
    m[1][2] = p2d.y
    table.insert(meas, m)
  end

  local s, X, inlier = xamla3d.linearTriangulation(P, meas)
  table.insert(allPts3d,X)

  --Do reprojection!
  local re = 0
  for j = 1,#matches do
    local imgId = matches[j].imageId
    local measId = matches[j].measurementId
    local robotP = duplo.triangulationData[imgId].robotPose
    local p2d = duplo.triangulationData[imgId].pts2d[measId]

    local x = xamla3d.projectPoint(duplo.intrinsics, torch.inverse(robotP*duplo.handEye)[{{1,3},{}}], X)
    local r = math.sqrt((p2d.x - x[1])^2+(p2d.y - x[2])^2)
    re = re + r
  end

  if (verbose ~= nil and verbose == true) then
    print("RE of point " ..i.. " "..re / #matches)
  end
 end

 return true, allPts3d
end


---
-- Fits a line through the given 3d points
-- @param points3D table of 3x1 torch.Tensor containg points 3d coordinates
-- @return --line3d as it comes from opencvs cv.fitLine function
function duplo.fitLine(points3D)
  local X = torch.DoubleTensor(#points3D,3)
  for i = 1,#points3D do
    X[{i, {1,3}}] = points3D[i]:view(1,3)
  end
  local line = cv.fitLine{points = X, distType = cv.DIST_L2, param = 0, reps = 0.01, aeps = 0.01}
  return line
end

---line3d as it comes from linefit on opencv
-- @param P1 first point on the line
-- @param P2 second point on the  line 
-- @param P0 point that is not on the line
function duplo.getPointLineDistance3D(P1, P2, P0)
  -- See http://mathworld.wolfram.com/Point-LineDistance3-Dimensional.html
  return torch.norm((P0-P1):cross(P0-P2))/torch.norm(P2-P1)
end


---
-- Gets the orientation and position of the Duplo brick in robot base coordinates. 
-- This is the interface function that is called from outside!
-- @param expectedDuploZPose the expected upper edge z-value (in mm in robot coordinates) of the brick
-- @param circlesPerRow number of knobs the brick has (in the demo we uses 4x2 bricks, so default is 4)
-- @param image optional parameter If given, debug information is drawn on the image (also requires robotPose parameter)
-- @param robotPose if an image for debugging is provided then also the robot pose is required
function duplo.getDuploOrientation(expectedDuploZPos, circlesPerRow, image, robotPose)
  circlesPerRow = circlesPerRow or 4

  local expectedMinZ = expectedDuploZPos - 10
  local expectedMaxZ = expectedDuploZPos + 10

  local success, result = duplo.triangulatePoints(expectedMinZ, expectedMaxZ)

  -- We need at least the number of circles that our bricks as knobs
  if #result < circlesPerRow * 2 then
    return false
  end

  if (success) then
    -- The z component is often very noisy, so we set the z component to 0 to get a good line fit
    local augmentedPts3D = {}
    for i = 1,#result do
      local X = result[i]:clone()
      X[3][1] = 0
      table.insert(augmentedPts3D, X)
    end

    -- Find two lines that both contain circlePerRow circles
    local lines = duplo.fitNLines(augmentedPts3D, 2, circlesPerRow, 1000)

    if (#lines ~=2) then
      return false
    end

    local fittedLines = {}

    -- the center of the brick
    local center = torch.DoubleTensor(3,1):zero()

    -- find the geometric center of all circles
    if (#lines == 2) then
      local cnt = 0
      for i = 1,#lines do
        local l = duplo.fitLine(lines[i])
        table.insert(fittedLines, l)
        for j = 1,#lines[i] do
          center = center + lines[i][j]
          cnt = cnt+1
        end
      end
      center = center / cnt
      print("Center")
      print(center)

      -- An average line of the two detected ones
      local finalLine = (fittedLines[1] + fittedLines[2])/2

      -- calculate the short and long axis coordinates of the duplo which define our brick's coordinate system
      local longAxis = finalLine[{{1,3}, 1}]
      longAxis[3] = 0
      longAxis = longAxis * (1 / torch.norm(longAxis))
      local shortAxis = torch.DoubleTensor({-1 * longAxis[2], longAxis[1], 0})
      shortAxis = shortAxis * (1 / torch.norm(shortAxis))

      local matrix = torch.eye(3,3)
      matrix[{{1,3},1}] = shortAxis:view(3,1)
      matrix[{{1,3},2}] = longAxis:view(3,1)
     
     -- If image and robot pose is given we draw the bricks coordinate system into the image
     if (image ~= nil and robotPose ~=nil) then
       local x_c = xamla3d.projectPoint(duplo.intrinsics, torch.inverse(robotPose * duplo.handEye)[{{1,3},{}}], center)
       local xAxis =  xamla3d.projectPoint(duplo.intrinsics, torch.inverse(robotPose * duplo.handEye)[{{1,3},{}}], center+matrix[{{1,3},1}]*30 )
       local yAxis =  xamla3d.projectPoint(duplo.intrinsics, torch.inverse(robotPose * duplo.handEye)[{{1,3},{}}], center+matrix[{{1,3},2}]*20 )
       cv.circle{img = image, center ={x=x_c[1], y=x_c[2]}, radius = 5, color = {0,255,0} }
       cv.line{img = image, pt1 = {x = x_c[1], y = x_c[2]}, pt2 ={x = xAxis[1], y = xAxis[2]}, thickness = 2, color= {0,0,255} }
       cv.line{img = image, pt1 = {x = x_c[1], y = x_c[2]}, pt2 ={x = yAxis[1], y = yAxis[2]}, thickness = 2, color= {0,255,0} }

     end

     return true, matrix, center

    else
      return false
    end

  end

   return false, nil

end


---
-- Fit multipe lines through a set of 3D points (in a RANSAC fashion)
-- @param X table containg 3x1 torch.Tensor, set of 3d points 
-- @param threshold distance of a point w.r.t. the line (RANSAC threshold)
-- @param maxIterations number of RANSAC iterations
function duplo.fitNLines(X,threshold, nMinElementsPerLine, maxIterations)
 local lines = {}

 for i = 1, maxIterations do
      local idx = torch.randperm(#X)
      local inlier = 0
      for j = 1, #X do
        local d = duplo.getPointLineDistance3D(X[idx[1]],X[idx[2]],X[j])
        if (d < threshold) then inlier = inlier + 1 end
      end
      if (inlier >=nMinElementsPerLine) then
        table.insert(lines, {})
        local P1 = X[idx[1]]
        local P2 = X[idx[2]]

         for j = #X,1,-1 do
           local d = duplo.getPointLineDistance3D(P1, P2 ,X[j])
           if (d < threshold) then
            table.insert(lines[#lines], X[j])
            table.remove(X, j)

           end
        end
        if (#X < nMinElementsPerLine) then
           return lines
        end
      end
    end
    return lines
end


function duplo.main()

  local numberOfCircles = 4
  local allCenters = {}
  for t = 1,5 do
    duplo.reset()
    duplo.directory = '/home/hoppe/data/duplo_triple7/'..t.."/"
    duplo.initOffline()


    for i = 1,#duplo.files-1 do
      image = cv.imread{files[i]}
      image_gray = cv.cvtColor{src = image, code = cv.COLOR_BGR2GRAY}
      local timer = torch.Timer()
      duplo.addImage(image, duplo.robotPose[i])
      print("Timer for adding an image: ".. timer:time().real)
    end

    --duplo.fitDuplo()
    local s, matrix, center = duplo.getDuploOrientation(numberOfCircles)
    if  s  == true then
      print("Duplo Found!")
      table.insert(allCenters, center)
    else
      print("Duplo Not Found!")
      print("Searching for duplo with 2x"..numberOfCircles .. " points")
    end
    cv.waitKey{-1}

  end -- t

  print(allCenters)

  for i = 1,#allCenters do
	print(allCenters[i][1][1] .. " " .. allCenters[i][2][1].." "..allCenters[i][3][1])
  end
end

--duplo.main()

return duplo
