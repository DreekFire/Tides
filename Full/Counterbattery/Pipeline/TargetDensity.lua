-- target/density
-- Aimpoint-derived empirical geometry distribution.
local TargetDensity = {}

function TargetDensity.new(config)
  return {
    aimPointTrackers = config.aimPointTrackers,
    TICKS_PER_S = config.TICKS_PER_S,
  }
end

function TargetDensity:update(I, tracks, aimpoints, rotations, stamp)
  local byId = {}
  for id, track in pairs(tracks.byId) do
    local en = track.state
    local rotation = rotations.byId[id]
    local dRot = rotation and rotation.deltaRotation
    local consensusPoints = rotation and rotation.consensusPoints

    if dRot then
      -- Copied from the original UpdateEnemyData density section.
      local changedAimpoints = {}
      local nTrack = #en.aimpoints
      local refIdx = consensusPoints[1]
      local invRot = Quaternion.Inverse(en.rotation[en.rotation.size])
      local trueCenterOfMass = track.trueCenterOfMass or en.trueCenterOfMass
      for i = 1, nTrack do
        local predicted = dRot * (en.oldAimpoints[i] - en.oldAimpoints[refIdx]) + en.aimpoints[refIdx]
        if (predicted - en.aimpoints[i]).sqrMagnitude > 3 * 3 then
          table.insert(changedAimpoints, invRot * (en.aimpoints[i] - trueCenterOfMass))
        end
      end
      -- Only update distribution if aimpoint has changed
      Stats.updateDistributionBatched(en.density, changedAimpoints)
      -- reduce n so distribution is allowed to change faster
      en.density.n = math.min(en.density.n, 50)
    end

    byId[id] = CounterbatteryTypes.product("target/density", id, stamp, {
      frame = "target-local",
      distribution = en.density,
      state = en,
    })
  end
  return CounterbatteryTypes.collection("target/density", stamp, byId, {
    frame = "target-local",
  })
end
