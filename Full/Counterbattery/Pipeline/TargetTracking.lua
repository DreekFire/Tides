-- target/track
-- Base target observations and the target-owned history buffers.
local TargetTracking = {}

local function initialDensityCovariance()
  -- The modular artifact bundles Low/NdArray.lua and Low/Stats.lua, whose
  -- vector distributions store covariance as an NdArray.  Keep the source
  -- module usable beside the legacy monolith as well.
  if NdArray and NdArray.zeros then
    local covariance = NdArray.zeros({3, 3}, "float")
    for i = 1, 3 do
      covariance.data[(i - 1) * 3 + i] = 2500
    end
    return covariance
  end
  return Matrix3.scalarmul(Matrix3.Identity(), 2500)
end

local function derivedVelocity(enemy, position, stamp)
  local previousPosition = enemy.pos[enemy.pos.size]
  local previousStamp = enemy.sampleTimes[enemy.sampleTimes.size]
  if previousPosition and previousStamp then
    local deltaTime = stamp - previousStamp
    if deltaTime > 0 then
      return (position - previousPosition) / deltaTime
    end
  end
  -- The first sample, or a non-increasing timestamp, has no finite
  -- position-derived velocity yet.
  return Vector3.zero
end

function TargetTracking.new(config)
  return {
    enemies = config.enemies,
    targetTrackTime = config.targetTrackTime,
    TICKS_PER_S = config.TICKS_PER_S,
    numOrigins = config.numOrigins,
    onTargetLost = config.onTargetLost,
  }
end

function TargetTracking:update(I, stamp)
  local enemies = self.enemies

  -- Copied from the original UpdateEnemyData target-history section.
  for id, en in pairs(enemies) do
    en.valid = false
  end
  for tarIdx = 0, I:GetNumberOfTargets(mainframeIdx) - 1 do
    local target = I:GetTargetInfo(mainframeIdx, tarIdx)
    if target.Valid then
      if not enemies[target.Id] then
        local rbsize = self.targetTrackTime * self.TICKS_PER_S
        local newEnemy = {
                            pos = RingBuffer.RingBuffer(rbsize),
                            vel = RingBuffer.RingBuffer(rbsize),
                            sampleTimes = RingBuffer.RingBuffer(rbsize),
                            rotation = RingBuffer.RingBuffer(rbsize),
                            origins = RingBuffer.RingBuffer(self.numOrigins),
                            density = Stats.Distribution({"x", "y", "z"}),
                            valid = true,
                            oldAimpoints = {},
                            aimpoints = {},
                          }
        newEnemy.density.n = 15
        -- todo: set initial size estimate based on block count
        newEnemy.density.cov = initialDensityCovariance()
        enemies[target.Id] = newEnemy
      end
      local e = enemies[target.Id]
      e.valid = true
      -- TargetInfo.Position is authoritative. TargetInfo.Velocity is still
      -- detection-derived, so derive velocity from true COM samples instead.
      e.trueCenterOfMass = target.Position
      local velocity = derivedVelocity(e, target.Position, stamp)
      RingBuffer.push(e.pos, target.Position)
      RingBuffer.push(e.vel, velocity)
      -- Keep the actual sample time beside the position and velocity.  The
      -- old solver reconstructed time from the ring index, which is only
      -- exact when UpdateEnemyData runs at a perfectly uniform cadence.
      RingBuffer.push(e.sampleTimes, stamp)
    end
  end

  -- have to do this because there is no get target by id function
  for id, en in pairs(enemies) do
    if not en.valid then
      enemies[id] = nil
      if self.onTargetLost then
        self.onTargetLost(id)
      end
    end
  end

  local byId = {}
  for id, en in pairs(enemies) do
    byId[id] = CounterbatteryTypes.product("target/track", id, stamp, {
      frame = "world",
      state = en,
      position = en.pos[en.pos.size],
      velocity = en.vel[en.vel.size],
      trueCenterOfMass = en.trueCenterOfMass,
      positionHistory = en.pos,
      velocityHistory = en.vel,
      sampleTimeHistory = en.sampleTimes,
      rotationHistory = en.rotation,
    })
  end
  return CounterbatteryTypes.collection("target/track", stamp, byId, {
    frame = "world",
  })
end
