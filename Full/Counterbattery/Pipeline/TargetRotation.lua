-- target/rotation
-- Rotation tracking from corresponding aimpoint triplets.
local TargetRotation = {}

function TargetRotation.new(config)
  return {
    aimPointTrackers = config.aimPointTrackers,
    maxTurnRateTracking = config.maxTurnRateTracking,
    TICKS_PER_S = config.TICKS_PER_S,
    trackLossTime = 0,
  }
end

function TargetRotation:update(I, tracks, aimpoints, stamp)
  local byId = {}
  for id, track in pairs(tracks.byId) do
    local en = track.state
    local dRot, consensusPoints
    if #self.aimPointTrackers < 3 then
      I:Log("Need at least 3 AIs to track rotation")
      dRot = nil
    else
      dRot, consensusPoints = self:getRotation(en.oldAimpoints, en.aimpoints)
    end

    if dRot then
      RingBuffer.push(en.rotation, dRot * (en.rotation[en.rotation.size] or Quaternion.identity))
    else
      RingBuffer.push(en.rotation, en.rotation[en.rotation.size] or Quaternion.identity)
      self.trackLossTime = self.trackLossTime + 1 / self.TICKS_PER_S
      I:Log(string.format("track loss time: %.2f", self.trackLossTime))
    end

    byId[id] = CounterbatteryTypes.product("target/rotation", id, stamp, {
      frame = "target-local",
      deltaRotation = dRot,
      rotation = en.rotation[en.rotation.size],
      consensusPoints = consensusPoints,
      state = en,
    })
  end
  return CounterbatteryTypes.collection("target/rotation", stamp, byId, {
    frame = "target-local",
  })
end

function TargetRotation:getRotation(oldPts, newPts, iterLim)
  -- Copied from the original GetRotation implementation.
  local nTrack = #newPts
  iterLim = iterLim or 25

  local indices = {}
  for i = 1, nTrack do
    indices[i] = i
  end
  local consensusPoints = {}
  -- iterate through triples of aimpoints, check if the legs are the same length
  for trip = 1, iterLim do
    MathUtil.shuffle(indices, true)
    local oldAimpoints = {}
    local aimpoints = {}
    for i = 1, 3 do
      oldAimpoints[i] = oldPts[indices[i]]
      aimpoints[i] = newPts[indices[i]]
    end

    local legs, oldLegs = {}, {}
    local valid = true
    for i = 1, 3 do
      legs[i] = aimpoints[i % 3 + 1] - aimpoints[(i - 1) % 3 + 1]
      oldLegs[i] = oldAimpoints[i % 3 + 1] - oldAimpoints[(i - 1) % 3 + 1]
      -- check if aimpoints have changed
      if math.abs(legs[i].sqrMagnitude - oldLegs[i].sqrMagnitude) > 0.1 then
        valid = false
        break
      end
    end
    if valid then
      local oldRot = Quaternion.LookRotation(oldLegs[1], oldLegs[2])
      local rot = Quaternion.LookRotation(legs[1], legs[2])
      local deltaRot = rot * Quaternion.Inverse(oldRot)
      if math.acos(math.max(math.min(deltaRot.w, 1), -1)) * 360 / math.pi * self.TICKS_PER_S < self.maxTurnRateTracking then
        -- check for consensus
        local votes = 0
        for idx, ap in ipairs(newPts) do
          local diff = ap - newPts[indices[1]]
          local oldDiff = oldPts[idx] - oldPts[indices[1]]
          if (diff - deltaRot * oldDiff).sqrMagnitude < 0.1 then
            votes = votes + 1
            table.insert(consensusPoints, idx)
          end
          if 2 * votes - 3 >= nTrack then
            return deltaRot, consensusPoints
          end
          consensusPoints = {}
        end
      end
    end
  end
  return nil
end
