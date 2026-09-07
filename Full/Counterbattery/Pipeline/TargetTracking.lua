-- target/track
-- Base target observations and the target-owned history buffers.
local TargetTracking = {}

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
    local targetPosition = I:GetTargetPositionInfo(mainframeIdx, tarIdx)
    if target.Valid then
      if not enemies[target.Id] then
        local rbsize = self.targetTrackTime * self.TICKS_PER_S
        local newEnemy = {
                            pos = RingBuffer.RingBuffer(rbsize),
                            vel = RingBuffer.RingBuffer(rbsize),
                            rotation = RingBuffer.RingBuffer(rbsize),
                            origins = RingBuffer.RingBuffer(self.numOrigins),
                            density = Stats.Distribution({"x", "y", "z"}),
                            valid = true,
                            oldAimpoints = {},
                            aimpoints = {},
                          }
        newEnemy.density.n = 15
        -- todo: set initial size estimate based on block count
        newEnemy.density.cov = Matrix3.scalarmul(Matrix3.Identity(), 2500)
        enemies[target.Id] = newEnemy
      end
      local e = enemies[target.Id]
      e.valid = true
      RingBuffer.push(e.pos, targetPosition.Position)
      RingBuffer.push(e.vel, targetPosition.Velocity)
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
      positionHistory = en.pos,
      velocityHistory = en.vel,
      rotationHistory = en.rotation,
    })
  end
  return CounterbatteryTypes.collection("target/track", stamp, byId, {
    frame = "world",
  })
end
