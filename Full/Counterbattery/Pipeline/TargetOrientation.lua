-- target/orientation
-- Estimate the target's body orientation from exact grid-constrained aimpoints.
-- This is an enrichment product: it does not replace target/rotation.
local TargetOrientation = {}

local EPSILON = 1e-9
local function atan2(y, x)
  if math.atan2 then return math.atan2(y, x) end
  return math.atan(y, x)
end

local AXES = {
  Vector3(1, 0, 0),
  Vector3(0, 1, 0),
  Vector3(0, 0, 1),
}

local function magnitude(vector)
  return vector and math.sqrt(vector.sqrMagnitude) or 0
end

local function normalized(vector)
  local length = magnitude(vector)
  if length <= EPSILON then return nil end
  return vector / length
end

local function validVector(vector)
  return vector and type(vector.x) == "number"
    and type(vector.y) == "number" and type(vector.z) == "number"
end

local function relativePosition(observation)
  if observation.relativePosition then return observation.relativePosition end
  if observation.position and observation.trueCenterOfMass then
    return observation.position - observation.trueCenterOfMass
  end
  return nil
end

local function nearestInteger(value)
  if value >= 0 then return math.floor(value + 0.5) end
  return math.ceil(value - 0.5)
end

local function gridError(point)
  local dx = point.x - nearestInteger(point.x)
  local dy = point.y - nearestInteger(point.y)
  local dz = point.z - nearestInteger(point.z)
  return dx * dx + dy * dy + dz * dz
end

local function quaternionSimilarity(first, second)
  if not first or not second then return 0 end
  return math.abs(first.x * second.x + first.y * second.y
    + first.z * second.z + first.w * second.w)
end

local function angularDistance(first, second)
  local similarity = math.max(-1, math.min(1, quaternionSimilarity(first, second)))
  return 2 * math.acos(similarity)
end

local function projectPerpendicular(vector, axis)
  return vector - axis * Vector3.Dot(vector, axis)
end

-- Build a rotation that maps one local Cartesian axis to an observed world
-- axis, retaining the prior orientation's roll as much as possible.
local function alignAxis(prior, localIndex, worldAxis, sign)
  local desired = normalized(worldAxis * sign)
  if not desired then return nil end
  local reference
  if localIndex == 2 then
    reference = projectPerpendicular(prior * Vector3(0, 0, 1), desired)
    reference = normalized(reference)
    if not reference then return nil end
    return Quaternion.LookRotation(reference, desired)
  elseif localIndex == 3 then
    reference = projectPerpendicular(prior * Vector3(0, 1, 0), desired)
    reference = normalized(reference)
    if not reference then return nil end
    return Quaternion.LookRotation(desired, reference)
  end

  local up = projectPerpendicular(prior * Vector3(0, 1, 0), desired)
  up = normalized(up)
  if not up then return nil end
  local forward = normalized(Vector3.Cross(desired, up))
  if not forward then return nil end
  return Quaternion.LookRotation(forward, up)
end

local function addCandidate(candidates, candidate, source, priority, limit)
  if not candidate or #candidates >= limit then return end
  candidates[#candidates + 1] = {
    rotation = candidate,
    source = source,
    priority = priority or 0,
  }
end

local function addYawGridCandidates(candidates, difference, config, limit)
  local horizontal = math.sqrt(difference.x * difference.x + difference.z * difference.z)
  if horizontal <= EPSILON then return end
  local y = nearestInteger(difference.y)
  if math.abs(difference.y - y) > config.gridCoordinateTolerance then return end

  local coordinateLimit = math.min(config.maxGridCoordinate,
    math.ceil(horizontal) + 1)
  local metricTolerance = config.gridMetricTolerance
  for x = -coordinateLimit, coordinateLimit do
    for z = -coordinateLimit, coordinateLimit do
      if x ~= 0 or z ~= 0 then
        local metricError = math.abs(x * x + z * z - horizontal * horizontal)
        if metricError <= metricTolerance then
            local observedAngle = atan2(difference.x, difference.z)
            local gridAngle = atan2(x, z)
          local yaw = math.deg(observedAngle - gridAngle)
          addCandidate(candidates, Quaternion.AngleAxis(yaw, Vector3(0, 1, 0)),
            "integer-grid", 1, limit)
        end
      end
      if #candidates >= limit then return end
    end
  end
end

local function subobjectSources(subobjects, targetId)
  local sources = {}
  local target = subobjects and subobjects.byId[targetId]
  for _, observation in ipairs((target and target.observations) or {}) do
    if observation.sourceId ~= nil then sources[observation.sourceId] = true end
  end
  return sources
end

function TargetOrientation.new(config)
  config = config or {}
  return setmetatable({
    maxGridCoordinate = config.maxGridCoordinate or 24,
    gridCoordinateTolerance = config.gridCoordinateTolerance or 0.2,
    gridMetricTolerance = config.gridMetricTolerance or 0.2,
    gridInlierThreshold = config.gridInlierThreshold or 0.15,
    continuityWeight = config.orientationContinuityWeight or 0.05,
    maxCandidates = config.orientationMaxCandidates or 96,
    assumeTargetYAligned = config.assumeTargetYAligned ~= false,
    worldHistory = {},
    previous = {},
  }, {__index = TargetOrientation})
end

function TargetOrientation:update(I, tracks, aimpoints, rotations, subobjects, stamp)
  local byId = {}
  for id, track in pairs(tracks.byId) do
    local aimpoint = aimpoints.byId[id]
    local rotationProduct = rotations.byId[id]
    local prior = self.previous[id]
      or (rotationProduct and rotationProduct.rotation)
      or Quaternion.identity
    local candidates = {}
    local sources = subobjectSources(subobjects, id)
    local histories = self.worldHistory[id] or {}
    self.worldHistory[id] = histories

    -- Estimate world-space axes from the points already classified as
    -- subobject aimpoints.  These candidates are deliberately inserted first.
    for _, observation in ipairs((aimpoint and aimpoint.observations) or {}) do
      local point = relativePosition(observation)
      if point and sources[observation.sourceId] then
        local history = histories[observation.sourceId] or {}
        histories[observation.sourceId] = history
        history[#history + 1] = point
        if #history > 3 then table.remove(history, 1) end
        if #history == 3 then
          local firstChord = history[2] - history[1]
          local secondChord = history[3] - history[2]
          local worldAxis = normalized(Vector3.Cross(firstChord, secondChord))
          if worldAxis and rotationProduct and rotationProduct.rotation then
            for axisIndex = 1, 3 do
              for sign = -1, 1, 2 do
                addCandidate(candidates,
                  alignAxis(rotationProduct.rotation, axisIndex, worldAxis, sign),
                  "subobject-axis", 0, self.maxCandidates)
              end
            end
          end
        end
      end
    end

    -- A normal spawn has local Y aligned to global Y, so a pairwise difference
    -- supplies a finite set of possible integer local vectors and hence yaw
    -- candidates.  This avoids enumerating arbitrary 3-D orientations.  The
    -- assumption is configurable for mothership-spawned targets.
    local observations = (aimpoint and aimpoint.observations) or {}
    if self.assumeTargetYAligned then
      for first = 1, #observations - 1 do
        local firstPoint = relativePosition(observations[first])
        if firstPoint then
          for second = first + 1, #observations do
            local secondPoint = relativePosition(observations[second])
            if secondPoint then
              addYawGridCandidates(candidates, firstPoint - secondPoint,
                self, self.maxCandidates)
            end
            if #candidates >= self.maxCandidates then break end
          end
        end
        if #candidates >= self.maxCandidates then break end
      end
    end
    addCandidate(candidates, rotationProduct and rotationProduct.rotation,
      "rotation-fallback", 2, self.maxCandidates)

    local best
    for _, candidate in ipairs(candidates) do
      local residual = 0
      local inliers = 0
      for _, observation in ipairs(observations) do
        local point = relativePosition(observation)
        if point then
          local localPoint = Quaternion.Inverse(candidate.rotation) * point
          local error = gridError(localPoint)
          residual = residual + math.min(error, 1)
          if error <= self.gridInlierThreshold then inliers = inliers + 1 end
        end
      end
      local continuity = angularDistance(candidate.rotation, prior)
      local score = residual + self.continuityWeight * continuity * continuity
        + candidate.priority * 1e-6
      if not best or score < best.score
          or (score == best.score and inliers > best.inliers) then
        best = {
          rotation = candidate.rotation,
          source = candidate.source,
          score = score,
          residual = residual,
          inliers = inliers,
          continuity = continuity,
        }
      end
    end

    local status = best and "estimated" or "unavailable"
    local product = CounterbatteryTypes.product("target/orientation", id, stamp, {
      frame = "world",
      orientation = best and best.rotation or nil,
      status = status,
      source = best and best.source or nil,
      score = best and best.score or nil,
      gridResidual = best and best.residual or nil,
      gridInliers = best and best.inliers or 0,
      continuity = best and best.continuity or nil,
      candidateCount = #candidates,
      targetYAligned = self.assumeTargetYAligned,
      subobjectAxesEvaluatedFirst = true,
      state = track.state,
    })
    byId[id] = product
    if best then self.previous[id] = best.rotation end
  end
  return CounterbatteryTypes.collection("target/orientation", stamp, byId,
    {frame = "world"})
end

_G.TargetOrientation = TargetOrientation
