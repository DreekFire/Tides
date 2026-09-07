local vectorMeta = {}
vectorMeta.__index = function(vector, key)
  if key == "sqrMagnitude" then
    return vector.x * vector.x + vector.y * vector.y + vector.z * vector.z
  end
  if key == "magnitude" then
    return math.sqrt(vector.sqrMagnitude)
  end
  if key == "normalized" then
    local magnitude = vector.magnitude
    if magnitude == 0 then return Vector3.zero end
    return vector / magnitude
  end
end
vectorMeta.__add = function(a, b) return Vector3(a.x + b.x, a.y + b.y, a.z + b.z) end
vectorMeta.__sub = function(a, b) return Vector3(a.x - b.x, a.y - b.y, a.z - b.z) end
vectorMeta.__unm = function(a) return Vector3(-a.x, -a.y, -a.z) end
vectorMeta.__mul = function(a, b)
  if type(a) == "number" then return Vector3(a * b.x, a * b.y, a * b.z) end
  return Vector3(a.x * b, a.y * b, a.z * b)
end
vectorMeta.__div = function(a, b) return Vector3(a.x / b, a.y / b, a.z / b) end

Vector3 = setmetatable({}, {
  __call = function(_, x, y, z)
    return setmetatable({x = x, y = y, z = z}, vectorMeta)
  end,
})

Vector3.zero = Vector3(0, 0, 0)
function Vector3.Dot(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
function Vector3.Cross(a, b)
  return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
end

local quaternionMeta = {}
quaternionMeta.__mul = function(a, b)
  if b.w ~= nil then
    return Quaternion(
      a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
      a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z)
  end
  local qv = Vector3(a.x, a.y, a.z)
  local uv = Vector3.Cross(qv, b)
  local uuv = Vector3.Cross(qv, uv)
  return b + 2 * (a.w * uv + uuv)
end

Quaternion = setmetatable({}, {
  __call = function(_, x, y, z, w)
    return setmetatable({x = x, y = y, z = z, w = w}, quaternionMeta)
  end,
})

Quaternion.identity = Quaternion(0, 0, 0, 1)
function Quaternion.Inverse(q)
  local norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
  return Quaternion(-q.x / norm, -q.y / norm, -q.z / norm, q.w / norm)
end

function Quaternion.AngleAxis(degrees, axis)
  local halfAngle = math.rad(degrees) / 2
  local unitAxis = axis.normalized
  local scale = math.sin(halfAngle)
  return Quaternion(unitAxis.x * scale, unitAxis.y * scale, unitAxis.z * scale, math.cos(halfAngle))
end

function Quaternion.LookRotation(forward, up)
  if forward.sqrMagnitude == 0 then return nil end
  local z = forward.normalized
  local x = Vector3.Cross(up, z)
  if x.sqrMagnitude == 0 then return nil end
  x = x.normalized
  local y = Vector3.Cross(z, x)
  local m00, m01, m02 = x.x, y.x, z.x
  local m10, m11, m12 = x.y, y.y, z.y
  local m20, m21, m22 = x.z, y.z, z.z
  local trace = m00 + m11 + m22
  local q
  if trace > 0 then
    local scale = math.sqrt(trace + 1) * 2
    q = Quaternion((m21 - m12) / scale, (m02 - m20) / scale, (m10 - m01) / scale, 0.25 * scale)
  elseif m00 > m11 and m00 > m22 then
    local scale = math.sqrt(1 + m00 - m11 - m22) * 2
    q = Quaternion(0.25 * scale, (m01 + m10) / scale, (m02 + m20) / scale, (m21 - m12) / scale)
  elseif m11 > m22 then
    local scale = math.sqrt(1 + m11 - m00 - m22) * 2
    q = Quaternion((m01 + m10) / scale, 0.25 * scale, (m12 + m21) / scale, (m02 - m20) / scale)
  else
    local scale = math.sqrt(1 + m22 - m00 - m11) * 2
    q = Quaternion((m02 + m20) / scale, (m12 + m21) / scale, 0.25 * scale, (m10 - m01) / scale)
  end
  local norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
  return Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)
end

local function closeVector(actual, expected, tolerance, message)
  assert((actual - expected).sqrMagnitude <= tolerance * tolerance, message)
end

local function assertConsensus(consensus, expected)
  assert(#consensus == #expected, "consensus should contain every inlier exactly once")
  for index = 1, #expected do
    assert(consensus[index] == expected[index], "consensus ordering should be deterministic")
  end
end

local rotation = Quaternion.AngleAxis(27, Vector3(0.3, 1, -0.2))
local oldPoints = {
  Vector3(0, 0, 0),
  Vector3(3, 0, 0),
  Vector3(0, 4, 0),
  Vector3(1, 1, 2),
  Vector3(-2, 1, 1),
}
local newPoints = {}
for index = 1, #oldPoints do newPoints[index] = rotation * oldPoints[index] end
newPoints[5] = Vector3(50, -40, 30)

local estimator = TargetRotationModule.new({
  aimPointTrackers = {1, 2, 3},
  maxTurnRateTracking = 100000,
  TICKS_PER_S = 1,
  ransacSeed = 17,
})
local estimated, consensus = estimator:getRotation(oldPoints, newPoints, 25)
assert(estimated ~= nil, "RANSAC should recover a rotation with one outlier")
assertConsensus(consensus, {1, 2, 3, 4})
for index = 1, 4 do closeVector(estimated * oldPoints[index], newPoints[index], 1e-5, "inlier rotation") end

local estimatedAgain, consensusAgain = estimator:getRotation(oldPoints, newPoints, 25)
assertConsensus(consensusAgain, consensus)
closeVector(estimatedAgain * oldPoints[2], estimated * oldPoints[2], 1e-8, "RANSAC should be deterministic")

local collinearOld = {Vector3(0, 0, 0), Vector3(1, 0, 0), Vector3(2, 0, 0)}
local collinearNew = {rotation * collinearOld[1], rotation * collinearOld[2], rotation * collinearOld[3]}
assert(estimator:getRotation(collinearOld, collinearNew, 25) == nil, "collinear samples must be rejected")
assert(estimator:getRotation({oldPoints[1], oldPoints[2]}, {newPoints[1], newPoints[2]}, 25) == nil,
  "fewer than three correspondences must be rejected")
assert(estimator:getRotation(oldPoints, {newPoints[1], newPoints[2]}, 25) == nil,
  "mismatched correspondence arrays must be rejected")
assert(estimator:getRotation({oldPoints[1], {x = 0, y = 0}}, {newPoints[1], newPoints[2]}, 25) == nil,
  "malformed points must be rejected")

local scaledOld = {
  Vector3(0, 0, 0),
  Vector3(300, 0, 0),
  Vector3(0, 400, 0),
}
local scaledNew = {rotation * scaledOld[1], rotation * scaledOld[2], rotation * scaledOld[3]}
scaledNew[2] = scaledNew[2] + Vector3(0.02, 0, 0)
local scaledEstimated, scaledConsensus = estimator:getRotation(scaledOld, scaledNew, 25)
assert(scaledEstimated ~= nil and #scaledConsensus == 3,
  "length validation should scale with aimpoint geometry")

local equivalentEstimator = TargetRotationModule.new({
  aimPointTrackers = {1, 2, 3},
  maxTurnRateTracking = 180,
  TICKS_PER_S = 1,
})
local equivalentRotation = Quaternion.AngleAxis(200, Vector3(0, 1, 0))
local equivalentNew = {}
for index = 1, 3 do equivalentNew[index] = equivalentRotation * scaledOld[index] end
local equivalentEstimated = equivalentEstimator:getRotation(scaledOld, equivalentNew, 25)
assert(equivalentEstimated ~= nil, "quaternion sign should not inflate the measured turn angle")

local sampledOld = {
  oldPoints[1], oldPoints[2], oldPoints[3], oldPoints[4], oldPoints[5],
  Vector3(2, 3, -1), Vector3(-1, -2, 3),
}
local sampledNew = {}
for index = 1, #sampledOld do sampledNew[index] = rotation * sampledOld[index] end
sampledNew[7] = Vector3(-70, 20, 80)
local sampledEstimator = TargetRotationModule.new({
  aimPointTrackers = {1, 2, 3},
  maxTurnRateTracking = 100000,
  TICKS_PER_S = 1,
  ransacSeed = 17,
})
math.randomseed(12345)
local randomBefore = math.random()
local sampledEstimated, sampledConsensus = sampledEstimator:getRotation(sampledOld, sampledNew, 10)
assert(sampledEstimated ~= nil and #sampledConsensus == 6,
  "bounded sampling should find the six-point consensus")
local randomAfter = math.random()
math.randomseed(12345)
assert(math.random() == randomBefore and math.random() == randomAfter,
  "RANSAC sampling must not consume the global random stream")

print("target rotation RANSAC tests passed")
