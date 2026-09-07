local solver = CounterbatteryClosestPointSolver

local function near(actual, expected, tolerance)
  assert(math.abs(actual - expected) <= tolerance,
    string.format("expected %.9f, got %.9f", expected, actual))
end

-- The timestamp ring is authoritative, even when samples are not uniformly
-- spaced.  The interpolated search should be used for the normal monotonic
-- case, with the same logical 1-based index as all other history rings.
local enemy = {
  pos = {size = 3},
  vel = {size = 3},
  sampleTimes = {size = 3, [1] = 0.00, [2] = 0.08, [3] = 0.21},
}

-- RingBuffer.get exposes chronological logical indices after wraparound;
-- the parallel timestamp ring therefore remains aligned with position and
-- velocity rings when the oldest sample is evicted.
local wrappedTimes = RingBuffer.RingBuffer(3)
RingBuffer.push(wrappedTimes, 0.00)
RingBuffer.push(wrappedTimes, 0.08)
RingBuffer.push(wrappedTimes, 0.21)
RingBuffer.push(wrappedTimes, 0.31)
near(wrappedTimes[1], 0.08, 1e-12)
near(wrappedTimes[2], 0.21, 1e-12)
near(wrappedTimes[3], 0.31, 1e-12)
local wrappedEnemy = {pos = {size = 3}, sampleTimes = wrappedTimes}
assert(solver.indexAtTime(wrappedEnemy, 0.29, 0.31, 10) == 3)
near(solver.sampleTime(enemy, 2, 10, 10), 0.08, 1e-12)
assert(solver.indexAtTime(enemy, 0.19, 10, 10) == 3)
assert(Search.calls > 0)

local lower, upper = solver.validInterval(
  enemy, {tStart = 0.0, dt = 0.4}, 0.21, 10)
near(lower, 0.0, 1e-12)
near(upper, 0.21, 1e-12)
local noInterval = solver.validInterval(
  enemy, {tStart = 0.3, dt = 0.4}, 0.21, 10)
assert(noInterval == nil)

-- Candidate roots outside [lower, upper] are discarded.  Among the remaining
-- roots, the Mahalanobis objective, not Euclidean distance or root order,
-- selects the answer.
local selected, objective = solver.selectAnalyticRoot(
  {-2.0, 0.25, 0.75, 3.0}, 0.0, 1.0,
  function(time)
    if math.abs(time - 0.75) < 1e-9 then return 1.0 end
    return 10.0
  end)
near(selected, 0.75, 1e-12)
near(objective, 1.0, 1e-12)

-- For an SPD inverse covariance and nonzero gravity, the cubic coefficient is
-- strictly positive: q3 = 1/2 * g^T Sigma^-1 g.
local q3 = solver.polynomialCoefficients(
  Vec(1, 2, 3), Vec(4, 5, 6), Vec(0, -10, 0),
  Matrix3.Matrix3({2, 0, 0, 0, 1, 0, 0, 0, 3}))
assert(q3 > 0)

-- Integration regression: the closest sample is selected from its timestamp,
-- not from the old fixed-cadence reconstruction.  A projectile at x=10 at
-- t=.20 is closest to the .21 sample, which is index 3 despite the jitter.
enemy.pos[1], enemy.pos[2], enemy.pos[3] = Vec(10, 0, 0), Vec(10, 0, 0), Vec(10, 0, 0)
enemy.vel[1], enemy.vel[2], enemy.vel[3] = Vec(0, 0, 0), Vec(0, 0, 0), Vec(0, 0, 0)
enemy.rotation = {
  [1] = Quaternion.identity,
  [2] = Quaternion.identity,
  [3] = Quaternion.identity,
}
enemy.density = {cov = Matrix3.Matrix3({1, 0, 0, 0, 1, 0, 0, 0, 1})}
local logs = {}
local interface = {
  GetTimeSinceSpawn = function() return 0.23 end,
  GetGravityForAltitude = function() return Vec(0, 0, 0) end,
  Log = function(_, message) logs[#logs + 1] = message end,
}
local diff, index = solver.runTrace(interface, {
  start = Vec(0, 0, 0),
  tStart = 0.0,
  ds = Vec(20, 0, 0),
  dt = 0.4,
}, enemy)
assert(index == 3)
near(diff.x, 0.5, 1e-9)
assert(#logs >= 1)
assert(string.find(logs[1], "newer Lua interpreter", 1, true))

print("Counterbattery closest-point solver and timestamp indexing: ok")
