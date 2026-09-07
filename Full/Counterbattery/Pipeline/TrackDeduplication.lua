-- target/track-cluster
-- Reconstructed projectile trajectories, deduplicated by shortest distance
-- between their local linearizations.
--
-- An origin is the closest Mahalanobis approach found by the counterbattery
-- solver.  Around that point, approximate the projectile path by a line whose
-- point is the reconstructed world-space origin and whose direction is the
-- projectile's instantaneous world velocity.  Origin.relVel is already a
-- world-space relative velocity in the existing origin producers, so only the
-- local position is rotated.  The input origin records and target state are
-- never modified; this module publishes a separate product.
local TargetTrackDeduplication = {}

local LINE_EPSILON = 1e-9

local function median(values)
  if #values == 0 then return nil end
  table.sort(values)
  local middle = math.floor((#values + 1) / 2)
  if #values % 2 == 1 then return values[middle] end
  return 0.5 * (values[middle] + values[middle + 1])
end

local function vectorMagnitude(vector)
  return math.sqrt(vector.sqrMagnitude)
end

local function normalize(vector)
  local magnitude = vectorMagnitude(vector)
  if magnitude <= LINE_EPSILON then return nil end
  return vector / magnitude
end

local function sampleTime(state, index, now, ticksPerSecond)
  local history = state.sampleTimes or state.sampleTimeHistory
  if history and history[index] ~= nil then
    return history[index]
  end
  local size = state.pos and state.pos.size or 0
  return now - (size - index) / (ticksPerSecond or 1)
end

local function nearestSample(state, absoluteTime, now, ticksPerSecond)
  local positions = state.pos or state.positionHistory
  if not positions or positions.size < 1 then return nil end

  local bestIndex
  local bestDistance
  for index = 1, positions.size do
    local distance = math.abs(sampleTime(state, index, now, ticksPerSecond) - absoluteTime)
    if bestDistance == nil or distance < bestDistance then
      bestIndex = index
      bestDistance = distance
    end
  end
  return bestIndex
end

local function latestValue(history, fallback)
  if history and history.size and history.size > 0 then
    return history[history.size]
  end
  return fallback
end

local function targetStateAtOrigin(state, origin, now, ticksPerSecond)
  local index = origin.time and nearestSample(state, origin.time, now, ticksPerSecond)
  if not index then
    index = state.pos and state.pos.size or nil
  end
  local position = index and (state.pos and state.pos[index])
  local velocity = index and (state.vel and state.vel[index])
  if not position then
    position = latestValue(state.pos, state.position)
  end
  if not velocity then
    velocity = latestValue(state.vel, state.velocity)
  end
  return position, velocity, index
end

local function makeLine(state, origin, now, ticksPerSecond, memberIndex, projectileEstimator, I)
  if not origin or not origin.position or not origin.relVel then return nil end
  local targetPosition, targetVelocity, sampleIndex = targetStateAtOrigin(
    state, origin, now, ticksPerSecond)
  if not targetPosition or not targetVelocity then return nil end

  local rotation = origin.rot or Quaternion.identity
  local worldVelocity = origin.relVel + targetVelocity
  local direction = normalize(worldVelocity)
  if not direction then return nil end

  local line = {
    type = "target/trajectory-line",
    memberIndex = memberIndex,
    point = targetPosition + rotation * origin.position,
    direction = direction,
    velocity = worldVelocity,
    targetPosition = targetPosition,
    targetVelocity = targetVelocity,
    originTime = origin.time,
    origin = origin,
    sampleIndex = sampleIndex,
  }
  if projectileEstimator then
    line.projectile = projectileEstimator:estimateOrigin(I, origin, worldVelocity, now)
  end
  return line
end

local function shortestLineDistance(first, second)
  local offset = second.point - first.point
  local cross = Vector3.Cross(first.direction, second.direction)
  local crossMagnitude = vectorMagnitude(cross)
  if crossMagnitude > LINE_EPSILON then
    return math.abs(Vector3.Dot(offset, cross)) / crossMagnitude
  end
  return vectorMagnitude(Vector3.Cross(offset, first.direction))
end

local function disjointSet(size)
  local parent = {}
  local rank = {}
  for index = 1, size do
    parent[index] = index
    rank[index] = 0
  end

  local function find(index)
    if parent[index] ~= index then
      parent[index] = find(parent[index])
    end
    return parent[index]
  end

  local function union(first, second)
    local firstRoot = find(first)
    local secondRoot = find(second)
    if firstRoot == secondRoot then return end
    if rank[firstRoot] < rank[secondRoot] then
      parent[firstRoot] = secondRoot
    elseif rank[firstRoot] > rank[secondRoot] then
      parent[secondRoot] = firstRoot
    else
      parent[secondRoot] = firstRoot
      rank[firstRoot] = rank[firstRoot] + 1
    end
  end

  return find, union
end

function TargetTrackDeduplication.new(config)
  config = config or {}
  return setmetatable({
    distanceThreshold = config.trackClusterDistance or 5,
    TICKS_PER_S = config.TICKS_PER_S or 1,
    projectileEstimation = TargetProjectileEstimation.new(config),
    rateOfFire = TargetWeaponRateOfFire.new(config),
    lastRateProduct = nil,
  }, {__index = TargetTrackDeduplication})
end

function TargetTrackDeduplication.lineDistance(first, second)
  return shortestLineDistance(first, second)
end

function TargetTrackDeduplication:cluster(first, second, third)
  local I, state, now
  if third == nil then
    state, now = first, second
  else
    I, state, now = first, second, third
  end
  local lines = {}
  local origins = state.origins
  if not origins then return lines end
  for index = 1, origins.size do
    -- Missile warnings have their own typed product and must never enter the
    -- ballistic cannon clustering stream.
    if origins[index].type ~= 2 then
      local line = makeLine(state, origins[index], now, self.TICKS_PER, index,
        self.projectileEstimation, I)
      if line then lines[#lines + 1] = line end
    end
  end

  local find, union = disjointSet(#lines)
  for first = 1, #lines - 1 do
    for second = first + 1, #lines do
      if shortestLineDistance(lines[first], lines[second]) <= self.distanceThreshold then
        union(first, second)
      end
    end
  end

  -- Roots are collected in input order.  This makes the result stable even
  -- when several lines belong to the same cluster.
  local byRoot = {}
  local clusters = {}
  for index, line in ipairs(lines) do
    local root = find(index)
    if not byRoot[root] then
      local cluster = {
        type = "target/track-cluster",
        id = #clusters + 1,
        count = 0,
        members = {},
        line = line,
        representative = line.origin,
      }
      byRoot[root] = cluster
      clusters[#clusters + 1] = cluster
    end
    local cluster = byRoot[root]
    cluster.count = cluster.count + 1
    cluster.members[#cluster.members + 1] = line
  end
  for _, cluster in ipairs(clusters) do
    local lower
    local upper
    local speeds = {}
    local sizes = {}
    local minSize
    local maxSize
    local sizeSource
    local sizeProxy
    for _, member in ipairs(cluster.members) do
      local projectile = member.projectile
      if projectile then
        local detectionRange = projectile.detectionRange
        if detectionRange and detectionRange.lowerBound ~= nil then
          lower = lower and math.max(lower, detectionRange.lowerBound) or detectionRange.lowerBound
        end
        if detectionRange and detectionRange.upperBound ~= nil and detectionRange.upperBound < math.huge then
          upper = upper and math.min(upper, detectionRange.upperBound) or detectionRange.upperBound
        end
        local speed = projectile.muzzleVelocity and projectile.muzzleVelocity.estimate
        if speed then speeds[#speeds + 1] = speed end
        local size = projectile.size and projectile.size.estimate
        if size then
          sizes[#sizes + 1] = size
          minSize = minSize and math.min(minSize, size) or size
          maxSize = maxSize and math.max(maxSize, size) or size
        end
        if projectile.size then
          sizeSource = sizeSource or projectile.size.source
          local proxy = projectile.size.detectionRangeProxy
          if proxy ~= nil then
            sizeProxy = sizeProxy and math.max(sizeProxy, proxy) or proxy
          end
        end
      end
    end
    local muzzleVelocity = median(speeds)
    local sizeEstimate = median(sizes)
    cluster.detectionRange = {
      lowerBound = lower,
      upperBound = upper,
    }
    cluster.muzzleVelocity = {
      estimate = muzzleVelocity,
      lowerBound = muzzleVelocity,
      upperBound = muzzleVelocity,
      sampleCount = #speeds,
    }
    cluster.projectileSize = {
      available = sizeEstimate ~= nil,
      estimate = sizeEstimate,
      lowerBound = minSize,
      upperBound = maxSize,
      sampleCount = #sizes,
      source = sizeSource or "detection-range",
      detectionRangeProxy = sizeProxy,
      reason = sizeEstimate and "calibrated" or "requires-detection-range-calibration",
    }
    cluster.projectile = CounterbatteryTypes.product("target/track-cluster/projectile",
      nil, now, {
        detectionRange = cluster.detectionRange,
        muzzleVelocity = cluster.muzzleVelocity,
        size = cluster.projectileSize,
      })
  end
  return clusters
end

function TargetTrackDeduplication:observeProjectile(I, observation, stamp)
  return self.projectileEstimation:observe(I, observation, stamp)
end

function TargetTrackDeduplication:observeMissiles(I, observations, stamp)
  return self.projectileEstimation:observeMissiles(I, observations, stamp)
end

function TargetTrackDeduplication:update(I, tracks, stamp)
  local byId = {}
  local rateById = {}
  for id, track in pairs(tracks.byId) do
    local clusters = self:cluster(I, track.state, stamp)
    local rates = self.rateOfFire:update(id, clusters, stamp,
      TargetTrackDeduplication.lineDistance)
    local enrichedClusters = {}
    for index, cluster in ipairs(clusters) do
      enrichedClusters[index] = CounterbatteryTypes.product("target/track-cluster", id,
        stamp, {
          count = cluster.count,
          members = cluster.members,
          line = cluster.line,
          representative = cluster.representative,
          detectionRange = cluster.detectionRange,
          muzzleVelocity = cluster.muzzleVelocity,
          projectileSize = cluster.projectileSize,
          projectile = cluster.projectile,
          weaponId = rates[index] and rates[index].weaponId,
          rateOfFire = rates[index],
        })
    end
    byId[id] = CounterbatteryTypes.product("target/track-cluster", id, stamp, {
      frame = "world",
      distanceThreshold = self.distanceThreshold,
      clusters = enrichedClusters,
      source = track,
      state = track.state,
    })
    rateById[id] = CounterbatteryTypes.collection("target/track-cluster/rate-of-fire",
      stamp, {[id] = CounterbatteryTypes.product("target/track-cluster/rate-of-fire", id,
        stamp, {weapons = rates, source = byId[id]})}, {frame = "world"})
  end
  self.lastRateProduct = CounterbatteryTypes.collection("target/track-cluster/rate-of-fire",
    stamp, rateById, {frame = "world"})
  return CounterbatteryTypes.collection("target/track-cluster", stamp, byId, {
    frame = "world",
    distanceThreshold = self.distanceThreshold,
    rateOfFire = self.lastRateProduct,
  })
end

_G.TargetTrackDeduplication = TargetTrackDeduplication
