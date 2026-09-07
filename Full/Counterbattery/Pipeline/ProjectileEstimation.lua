-- target/track-cluster/projectile
-- Projectile metadata enrichment for reconstructed origins.
--
-- The FtD Lua API gives us projectile position/velocity observations and
-- vehicle self-awareness dimensions, but not projectile caliber or a firing
-- event/cannon identifier.  Size is inferred from detection range through a
-- supplied calibration (or retained as an uncalibrated range proxy), never
-- read from a nonexistent API field.
local TargetProjectileEstimation = {}

local RANGE_EPSILON = 1e-9

local function magnitude(vector)
  if not vector then return nil end
  if vector.sqrMagnitude then return math.sqrt(vector.sqrMagnitude) end
  if vector.magnitude then return vector.magnitude end
  if vector.x and vector.y and vector.z then
    return math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
  end
  return nil
end

local function distance(first, second)
  if not first or not second then return nil end
  return magnitude(first - second)
end

local function dot(first, second)
  if not first or not second then return nil end
  if Vector3 and Vector3.Dot then return Vector3.Dot(first, second) end
  return first.x * second.x + first.y * second.y + first.z * second.z
end

local function readVector(I, methodName)
  local method = I and I[methodName]
  if type(method) ~= "function" then return nil end
  local ok, value = pcall(method, I)
  if ok then return value end
  return nil
end

local function finite(value)
  return type(value) == "number" and value == value and value < math.huge and value > -math.huge
end

local function componentwiseClampDistance(localPoint, minimum, maximum)
  local dx = 0
  local dy = 0
  local dz = 0
  if localPoint.x < minimum.x then dx = minimum.x - localPoint.x
  elseif localPoint.x > maximum.x then dx = localPoint.x - maximum.x end
  if localPoint.y < minimum.y then dy = minimum.y - localPoint.y
  elseif localPoint.y > maximum.y then dy = localPoint.y - maximum.y end
  if localPoint.z < minimum.z then dz = minimum.z - localPoint.z
  elseif localPoint.z > maximum.z then dz = localPoint.z - maximum.z end
  return math.sqrt(dx * dx + dy * dy + dz * dz)
end

local function directBounds(config)
  local bounds = config and config.vehicleBounds
  if not bounds or not bounds.minimum or not bounds.maximum then return nil end
  return {
    center = bounds.center,
    minimum = bounds.minimum,
    maximum = bounds.maximum,
    right = bounds.right,
    up = bounds.up,
    forward = bounds.forward,
  }
end

local function vehicleBounds(I, config)
  local configured = directBounds(config)
  if configured then return configured end

  local minimum = readVector(I, "GetConstructMinDimensions")
  local maximum = readVector(I, "GetConstructMaxDimensions")
  local center = readVector(I, "GetConstructPosition")
  if not minimum or not maximum or not center then return nil end
  return {
    center = center,
    minimum = minimum,
    maximum = maximum,
    right = readVector(I, "GetConstructRightVector"),
    up = readVector(I, "GetConstructUpVector"),
    forward = readVector(I, "GetConstructForwardVector"),
  }
end

local function boxDistance(I, point, config)
  local bounds = vehicleBounds(I, config)
  if not bounds or not bounds.center then return nil end
  local delta = point - bounds.center
  local localPoint
  if bounds.right and bounds.up and bounds.forward then
    localPoint = {
      x = dot(delta, bounds.right),
      y = dot(delta, bounds.up),
      z = dot(delta, bounds.forward),
    }
  else
    -- This is also useful for tests and for older bindings.  Without axes,
    -- the dimensions are conservatively treated as world-aligned.
    localPoint = delta
  end
  return componentwiseClampDistance(localPoint, bounds.minimum, bounds.maximum)
end

local function sizeFrom(origin, config, detectionRange)
  local value = origin and (origin.projectileSize or origin.size or origin.diameter or origin.caliber)
  if value == nil and config then
    value = config.projectileSize
  end
  if type(value) == "table" then
    value = value.estimate or value.value
  end
  if type(value) == "number" and value >= 0 then
    return value, "origin-or-config"
  end
  if config and type(config.projectileSizeFromDetectionRange) == "function" then
    local ok, estimate = pcall(config.projectileSizeFromDetectionRange,
      detectionRange and detectionRange.lowerBound,
      detectionRange and detectionRange.upperBound, origin)
    if ok and type(estimate) == "number" and estimate >= 0 then
      return estimate, "detection-range-estimator"
    end
  end
  if config and type(config.projectileSizeEstimator) == "function" then
    local ok, estimate = pcall(config.projectileSizeEstimator, origin, detectionRange)
    if ok and type(estimate) == "number" and estimate >= 0 then
      return estimate, "configured-estimator"
    end
  end
  if config and type(config.projectileSizeDetectionRanges) == "table"
      and detectionRange and detectionRange.lowerBound ~= nil then
    local lower = detectionRange.lowerBound
    for _, band in ipairs(config.projectileSizeDetectionRanges) do
      local maximum = band.maxRange or band.maximumRange
      local estimate = band.size or band.estimate
      if type(maximum) == "number" and type(estimate) == "number"
          and lower <= maximum then
        return estimate, "detection-range-band"
      end
    end
  end
  -- Detection range alone cannot identify physical diameter without the
  -- vehicle's sensor threshold curve.  Keep this observable for calibration.
  if detectionRange and detectionRange.lowerBound ~= nil then
    return nil, "requires-detection-range-calibration"
  end
  return nil, "missing-detection-range"
end

local function originStreamId(origin)
  if origin and origin.streamId then return origin.streamId end
  if origin and origin.type == 2 and origin.missileId then
    return "missile:" .. tostring(origin.missileId)
  end
  return "custom-axis"
end

function TargetProjectileEstimation.new(config)
  config = config or {}
  return setmetatable({
    config = config,
    maxDetectionRange = config.maxDetectionRange or math.huge,
    missileOverlapDistance = config.missileOverlapDistance or 5,
    observations = {},
    missiles = {},
  }, {__index = TargetProjectileEstimation})
end

function TargetProjectileEstimation:observeMissiles(I, observations, stamp)
  self.missiles = {}
  for _, observation in ipairs(observations or {}) do
    if observation.position then
      self.missiles[#self.missiles + 1] = {
        id = observation.id,
        position = observation.position,
        stamp = stamp,
      }
    end
  end
  return CounterbatteryTypes.collection(
    "target/track-cluster/missile-observation", stamp, self.missiles)
end

function TargetProjectileEstimation:rangeBounds(I, point, centerOfMass)
  local center = centerOfMass or readVector(I, "GetConstructCenterOfMass")
  if not point or not center then return nil end
  local upper = distance(point, center)
  local lower = boxDistance(I, point, self.config)
  if lower == nil then lower = upper end
  if lower > upper then lower, upper = upper, lower end
  return {lower = lower, upper = upper}
end

-- Record one sensor stream's state.  `position` may be the last known
-- position when detected=false; that lets a one-tick disappearance create a
-- conservative interval without pretending the projectile was observed.
function TargetProjectileEstimation:observe(I, observation, stamp)
  observation = observation or {}
  local streamId = observation.streamId or "custom-axis"
  local detected = observation.detected == true
  local bounds = observation.position and self:rangeBounds(
    I, observation.position, observation.centerOfMass)
  local previous = self.observations[streamId]
  local missileId
  if detected and observation.position then
    for _, missile in ipairs(self.missiles) do
      if distance(observation.position, missile.position)
          <= self.missileOverlapDistance then
        missileId = missile.id
        break
      end
    end
  end
  local accepted = detected and missileId == nil
  local detectionLower = bounds and bounds.lower or nil
  local detectionUpper = accepted and self.maxDetectionRange or nil
  local reacquired = false
  if accepted and previous and not previous.detected and not previous.excluded
      and previous.range and bounds then
    -- The threshold lies between the last absent and first present sensor
    -- distances.  The union of the CoM/bounding-box intervals is conservative.
    detectionLower = math.min(previous.range.lower, bounds.lower)
    detectionUpper = math.min(self.maxDetectionRange,
      math.max(previous.range.upper, bounds.upper))
    reacquired = true
  end

  local product = CounterbatteryTypes.product(
    "target/track-cluster/projectile-observation", nil, stamp, {
      streamId = streamId,
      detected = detected,
      accepted = accepted,
      excludedByMissileId = missileId,
      excluded = missileId ~= nil,
      position = observation.position,
      centerOfMass = observation.centerOfMass,
      observedRange = bounds,
      reportedRange = observation.reportedRange,
      detectionRangeLower = detectionLower,
      detectionRangeUpper = detectionUpper,
      reacquired = reacquired,
    })
  self.observations[streamId] = {
    detected = accepted,
    position = observation.position,
    range = bounds,
    stamp = stamp,
    detectionLower = detectionLower,
    detectionUpper = detectionUpper,
    excluded = missileId ~= nil,
  }
  return product
end

function TargetProjectileEstimation:estimateOrigin(I, origin, velocity, stamp)
  local streamId = originStreamId(origin)
  local observation = self.observations[streamId]
  local range = origin.detectPos and self:rangeBounds(I, origin.detectPos,
    origin.detectCenterOfMass)
  local detectionLower = origin.detectionRangeLower or (range and range.lower)
  local detectionUpper = origin.detectionRangeUpper
  if observation and observation.detected and observation.stamp == origin.detectTime then
    detectionLower = observation.detectionLower or detectionLower
    detectionUpper = observation.detectionUpper or detectionUpper
  end
  if detectionUpper == nil and detectionLower ~= nil then
    detectionUpper = self.maxDetectionRange
  end

  local muzzleVelocity = origin.muzzleVelocity
  if not muzzleVelocity and origin.projectileVelocity then
    muzzleVelocity = magnitude(origin.projectileVelocity)
  end
  if not muzzleVelocity and velocity then muzzleVelocity = magnitude(velocity) end
  local size, sizeSource = sizeFrom(origin, self.config, {
    lowerBound = detectionLower,
    upperBound = detectionUpper,
    observed = range,
  })
  return CounterbatteryTypes.product("target/track-cluster/projectile", nil, stamp, {
    streamId = streamId,
    detectionRange = {
      lowerBound = detectionLower,
      upperBound = detectionUpper,
      observed = range,
    },
    muzzleVelocity = {
      estimate = muzzleVelocity,
      lowerBound = muzzleVelocity,
      upperBound = muzzleVelocity,
      range = range,
    },
    size = {
      available = size ~= nil,
      estimate = size,
      lowerBound = size,
      upperBound = size,
      source = sizeSource,
      -- Preserve the range observable even when no physical-size calibration
      -- has been configured yet.
      detectionRangeProxy = detectionLower,
    },
  })
end

TargetProjectileEstimation.magnitude = magnitude
TargetProjectileEstimation.originStreamId = originStreamId
TargetProjectileEstimation.distance = distance

_G.TargetProjectileEstimation = TargetProjectileEstimation
