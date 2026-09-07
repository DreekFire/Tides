-- target/closest-point
-- Closest point between the projectile trace and the target distribution.
--
-- The analytic pass freezes the target rotation at an observed history sample.
-- The final pass evaluates every available history sample with that sample's
-- rotation, so the scan does not inherit the constant-rotation approximation.
local CounterbatteryClosestPointSolver = {}
local solverWarningShown = false
local degenerateCubicWarningShown = false
local ROOT_EPS = 1e-7
local CUBIC_EPS = 1e-12

local function covarianceMatrix(covariance)
  if covariance and covariance.shape and covariance.data then
    local values = {}
    for row = 0, 2 do
      for col = 0, 2 do
        values[row * 3 + col + 1] = covariance.data[(covariance.offset or 0)
          + row * covariance.stride[1] + col * covariance.stride[2] + 1]
      end
    end
    return Matrix3.Matrix3(values)
  end
  return covariance
end

local function max(a, b)
  if a > b then return a end
  return b
end

local function min(a, b)
  if a < b then return a end
  return b
end

local function logInterpreterChange(I)
  if solverWarningShown then return end
  solverWarningShown = true
  if I and I.Log then
    I:Log("Counterbattery: legacy covariance-inverse negation/row-shifting workaround removed; FtD now uses the newer Lua interpreter.")
  end
end

local function historySize(enemy)
  local history = enemy.sampleTimes or enemy.sampleTimeHistory
  if history then return history.size end
  return enemy.pos and enemy.pos.size or 0
end

function CounterbatteryClosestPointSolver.sampleTime(enemy, index, now, ticksPerSecond)
  local history = enemy.sampleTimes or enemy.sampleTimeHistory
  if history and history[index] ~= nil then
    return history[index]
  end
  -- Compatibility fallback for enemies created by the old monolith.  New
  -- targets always have explicit timestamps, including jittered samples.
  local size = enemy.pos and enemy.pos.size or 0
  ticksPerSecond = ticksPerSecond or 1
  return now - (size - index) / ticksPerSecond
end

function CounterbatteryClosestPointSolver.indexAtTime(enemy, absoluteTime, now, ticksPerSecond)
  local size = historySize(enemy)
  if size < 1 then return nil end

  local history = enemy.sampleTimes or enemy.sampleTimeHistory
  local monotonic = true
  if history then
    for i = 2, size do
      if history[i] == nil or history[i] < history[i - 1] then
        monotonic = false
        break
      end
    end
  end
  if history and monotonic and size > 1 and history[size] > history[1]
      and Search and Search.interpolatedSearch then
    -- Search.interpolatedSearch returns the closest entry when findClosest is
    -- true.  The explicit scan below remains a safe fallback for a malformed
    -- or non-monotonic timestamp ring.
    local index = Search.interpolatedSearch(history, 1, size, absoluteTime, true)
    if index and history[index] ~= nil then
      return index
    end
  end

  local bestIndex
  local bestDistance
  for index = 1, size do
    local sample = CounterbatteryClosestPointSolver.sampleTime(enemy, index, now, ticksPerSecond)
    local distance = math.abs(sample - absoluteTime)
    if not bestDistance or distance < bestDistance then
      bestIndex = index
      bestDistance = distance
    end
  end
  return bestIndex
end

function CounterbatteryClosestPointSolver.validInterval(enemy, line, now, ticksPerSecond)
  local size = historySize(enemy)
  if size < 1 or not line or not line.tStart or not line.dt or line.dt <= 0 then
    return nil
  end
  local oldest = CounterbatteryClosestPointSolver.sampleTime(enemy, 1, now, ticksPerSecond)
  local newest = CounterbatteryClosestPointSolver.sampleTime(enemy, size, now, ticksPerSecond)
  local lower = max(0, oldest - line.tStart)
  local upper = min(line.dt, newest - line.tStart)
  if upper < lower then return nil end
  return lower, upper
end

local function quadraticForm(vector, matrix, other)
  return Matrix3.quadform(vector, matrix, other or vector)
end

function CounterbatteryClosestPointSolver.polynomialCoefficients(di, relativeVelocity, gravity, covInv)
  -- For r(s) = di + relativeVelocity*s + 1/2*gravity*s^2,
  -- d(r^T A r)/ds = 2*(r^T A*(relativeVelocity + gravity*s)).
  -- The common factor 2 is omitted because it does not change the roots.
  local q3 = 0.5 * quadraticForm(gravity, covInv)
  local q2 = quadraticForm(relativeVelocity, covInv, gravity)
    + 0.5 * quadraticForm(gravity, covInv, relativeVelocity)
  local q1 = quadraticForm(di, covInv, gravity)
    + quadraticForm(relativeVelocity, covInv)
  local q0 = quadraticForm(di, covInv, relativeVelocity)
  return q3, q2, q1, q0
end

local function addRoot(roots, root, lower, upper)
  if not root or root ~= root then return end
  if root >= lower - ROOT_EPS and root <= upper + ROOT_EPS then
    root = min(upper, max(lower, root))
    for _, existing in ipairs(roots) do
      if math.abs(existing - root) <= ROOT_EPS then return end
    end
    roots[#roots + 1] = root
  end
end

function CounterbatteryClosestPointSolver.selectAnalyticRoot(candidateRoots, lower, upper, objective)
  local roots = {}
  addRoot(roots, lower, lower, upper)
  addRoot(roots, upper, lower, upper)
  for _, root in ipairs(candidateRoots or {}) do
    addRoot(roots, root, lower, upper)
  end

  local bestRoot
  local bestObjective
  for _, root in ipairs(roots) do
    local value = objective(root)
    if value and (not bestObjective or value < bestObjective) then
      bestRoot = root
      bestObjective = value
    end
  end
  return bestRoot, bestObjective
end

local function solveCandidates(q3, q2, q1, q0)
  if math.abs(q3) > CUBIC_EPS then
    local a, b, c = MathUtil.solveCubic(q3, q2, q1, q0)
    return {a, b, c}, "cubic"
  end
  if math.abs(q2) > CUBIC_EPS then
    local a, b = MathUtil.solveQuadratic(q2, q1, q0)
    return {a, b}, "quadratic"
  end
  if math.abs(q1) > CUBIC_EPS then
    return {-q0 / q1}, "linear"
  end
  return {}, "constant"
end

function CounterbatteryClosestPointSolver.chooseAnalyticRoot(q3, q2, q1, q0, lower, upper, objective)
  local candidates, degree = solveCandidates(q3, q2, q1, q0)
  local root, value = CounterbatteryClosestPointSolver.selectAnalyticRoot(
    candidates, lower, upper, objective)
  return root, value, degree
end

local function gravityAt(I, altitude)
  return I:GetGravityForAltitude(altitude)
end

local function projectileDifference(line, projectileVelocity, gravity, targetPosition, dt)
  return line.start - targetPosition + dt * projectileVelocity + 0.5 * gravity * dt * dt
end

local function rotationAt(enemy, index)
  return enemy.rotation and enemy.rotation[index] or Quaternion.identity
end

local function scanClosest(I, line, enemy, covInv, lower, upper, now, ticksPerSecond, gravity, projectileVelocity)
  local size = historySize(enemy)
  local bestIndex
  local bestObjective
  for index = 1, size do
    local sampleTime = CounterbatteryClosestPointSolver.sampleTime(enemy, index, now, ticksPerSecond)
    local dt = sampleTime - line.tStart
    if dt >= lower - ROOT_EPS and dt <= upper + ROOT_EPS then
      local diff = projectileDifference(line, projectileVelocity, gravity, enemy.pos[index], dt)
      local localDiff = Quaternion.Inverse(rotationAt(enemy, index)) * diff
      local objective = quadraticForm(localDiff, covInv)
      if not bestObjective or objective < bestObjective then
        bestIndex = index
        bestObjective = objective
      end
    end
  end
  return bestIndex, bestObjective
end

function CounterbatteryClosestPointSolver.runTrace(I, line, enemy, timeGuess)
  local now = I:GetTimeSinceSpawn()
  local ticksPerSecond = TICKS_PER_S or 1
  local lower, upper = CounterbatteryClosestPointSolver.validInterval(enemy, line, now, ticksPerSecond)
  if not lower then return nil end

  logInterpreterChange(I)
  local covInv = Matrix3.inverse(covarianceMatrix(enemy.density.cov))
  if not covInv then return nil end
  local projectileVelocity = line.ds / line.dt
  local gravity = gravityAt(I, line.start.y)
  local anchorIndex = timeGuess and enemy.pos[timeGuess] and timeGuess
    or CounterbatteryClosestPointSolver.indexAtTime(enemy, line.tStart + upper, now, ticksPerSecond)
  if not anchorIndex then return nil end

  -- Re-anchor the constant-rotation polynomial at the selected history sample
  -- twice.  This keeps the cubic cheap while moving the local linearization
  -- toward the candidate time before the changing-rotation scan.
  for _ = 1, 2 do
    local anchorTime = CounterbatteryClosestPointSolver.sampleTime(enemy, anchorIndex, now, ticksPerSecond)
    local targetPosition = enemy.pos[anchorIndex]
    local targetVelocity = enemy.vel[anchorIndex]
    if not targetPosition or not targetVelocity then return nil end
    local di = line.start - (targetPosition + (line.tStart - anchorTime) * targetVelocity)
    local relativeVelocity = projectileVelocity - targetVelocity
    local inverseRotation = Quaternion.Inverse(rotationAt(enemy, anchorIndex))
    local localDi = inverseRotation * di
    local localRelativeVelocity = inverseRotation * relativeVelocity
    local localGravity = inverseRotation * gravity
    local q3, q2, q1, q0 = CounterbatteryClosestPointSolver.polynomialCoefficients(
      localDi, localRelativeVelocity, localGravity, covInv)
    if math.abs(q3) <= CUBIC_EPS and not degenerateCubicWarningShown and I and I.Log then
      degenerateCubicWarningShown = true
      I:Log("Counterbattery: Mahalanobis cubic term is degenerate; using a lower-degree fallback.")
    end
    local root = CounterbatteryClosestPointSolver.chooseAnalyticRoot(
      q3, q2, q1, q0, lower, upper, function(candidate)
        local candidateDiff = localDi
          + candidate * localRelativeVelocity
          + 0.5 * candidate * candidate * localGravity
        return quadraticForm(candidateDiff, covInv)
      end)
    if not root then return nil end
    anchorIndex = CounterbatteryClosestPointSolver.indexAtTime(
      enemy, line.tStart + root, now, ticksPerSecond)
    if not anchorIndex then return nil end
  end

  local bestIndex = scanClosest(
    I, line, enemy, covInv, lower, upper, now, ticksPerSecond, gravity, projectileVelocity)
  if not bestIndex then return nil end
  local bestTime = CounterbatteryClosestPointSolver.sampleTime(enemy, bestIndex, now, ticksPerSecond)
  local dt = bestTime - line.tStart
  return projectileDifference(line, projectileVelocity, gravity, enemy.pos[bestIndex], dt), bestIndex
end

-- Exposed for the Lua test harness; the generated FtD script calls runTrace
-- through the global RunTrace entrypoint below.
_G.CounterbatteryClosestPointSolver = CounterbatteryClosestPointSolver

function RunTrace(I, line, enemy, timeGuess)
  return CounterbatteryClosestPointSolver.runTrace(I, line, enemy, timeGuess)
end
