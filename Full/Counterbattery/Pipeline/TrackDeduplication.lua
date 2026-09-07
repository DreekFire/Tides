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

local function makeLine(state, origin, now, ticksPerSecond, memberIndex)
  if not origin or not origin.position or not origin.relVel then return nil end
  local targetPosition, targetVelocity, sampleIndex = targetStateAtOrigin(
    state, origin, now, ticksPerSecond)
  if not targetPosition or not targetVelocity then return nil end

  local rotation = origin.rot or Quaternion.identity
  local worldVelocity = origin.relVel + targetVelocity
  local direction = normalize(worldVelocity)
  if not direction then return nil end

  return {
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
  }, {__index = TargetTrackDeduplication})
end

function TargetTrackDeduplication.lineDistance(first, second)
  return shortestLineDistance(first, second)
end

function TargetTrackDeduplication:cluster(state, now)
  local lines = {}
  local origins = state.origins
  if not origins then return lines end
  for index = 1, origins.size do
    local line = makeLine(state, origins[index], now, self.TICKS_PER_S, index)
    if line then lines[#lines + 1] = line end
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
  return clusters
end

function TargetTrackDeduplication:update(I, tracks, stamp)
  local byId = {}
  for id, track in pairs(tracks.byId) do
    local clusters = self:cluster(track.state, stamp)
    byId[id] = CounterbatteryTypes.product("target/track-cluster", id, stamp, {
      frame = "world",
      distanceThreshold = self.distanceThreshold,
      clusters = clusters,
      source = track,
      state = track.state,
    })
  end
  return CounterbatteryTypes.collection("target/track-cluster", stamp, byId, {
    frame = "world",
    distanceThreshold = self.distanceThreshold,
  })
end

_G.TargetTrackDeduplication = TargetTrackDeduplication
