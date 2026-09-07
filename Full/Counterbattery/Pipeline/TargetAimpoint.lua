-- target/aimpoint
-- Mainframe aimpoint observations.  The underlying target/track product is
-- not modified.
--
-- `TargetInfo.Position` is the authoritative target center of mass and
-- `TargetInfo.AimPointPosition` is already a world-space aimpoint.
-- The counterbattery pipeline deliberately does not consume TargetPositionInfo:
-- detected COM measurements only add noise when ground truth is available.
--
-- `position` stays world-space so target/rotation can estimate rotation from
-- corresponding aimpoints.  `relativePosition` is the same observation
-- expressed relative to the true target center of mass for geometry stages.
--
-- No rotation is applied here; target/rotation needs the world-space points
-- before it estimates the current rotation.
local TargetAimpoint = {}

function TargetAimpoint.new(config)
  return {
    enemies = config.enemies,
    aimPointTrackers = config.aimPointTrackers,
  }
end

function TargetAimpoint:update(I, tracks, stamp)
  local observations = {}

  -- Copied from the original UpdateEnemyData aimpoint section.
  for idx, tracker in ipairs(self.aimPointTrackers) do
    for tarIdx = 0, I:GetNumberOfTargets(tracker.idx) - 1 do
      local target = I:GetTargetInfo(tracker.idx, tarIdx)
      if target.Valid then
        local e = self.enemies[target.Id]
        local aimpoint = target.AimPointPosition
        e.oldAimpoints[idx] = e.aimpoints[idx] or aimpoint
        e.aimpoints[idx] = aimpoint

        observations[target.Id] = observations[target.Id] or {}
        observations[target.Id][idx] = CounterbatteryTypes.product(
          "target/aimpoint", target.Id, stamp, {
            frame = "world",
            sourceId = tracker.idx,
            position = e.aimpoints[idx],
            previousPosition = e.oldAimpoints[idx],
            trueCenterOfMass = target.Position,
            relativePosition = aimpoint - target.Position,
            state = e,
          })
      end
    end
  end

  local byId = {}
  for id, track in pairs(tracks.byId) do
    byId[id] = CounterbatteryTypes.product("target/aimpoint", id, stamp, {
      frame = "world",
      observations = observations[id] or {},
      trueCenterOfMass = track.trueCenterOfMass,
      oldAimpoints = track.state.oldAimpoints,
      aimpoints = track.state.aimpoints,
      state = track.state,
    })
  end
  return CounterbatteryTypes.collection("target/aimpoint", stamp, byId, {
    frame = "world",
  })
end
