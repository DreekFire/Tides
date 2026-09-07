-- target/aimpoint
-- Mainframe aimpoint observations normalized against each mainframe's target
-- position estimate.  The underlying target/track product is not modified.
--
-- `position` is deliberately world-space.  The normalization is:
--
--   TargetPositionInfo.Position
--     + (AimPointPosition - TargetInfo.Position)
--
-- This preserves the target-relative offset reported by one mainframe while
-- translating it onto that mainframe's target-position estimate.  It is not a
-- center-of-mass claim and it is not rotation-corrected here.  TargetRotation
-- needs these world-space points, and TargetDensity applies the inverse tracked
-- rotation after subtracting the target track position.
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
        local detectionError = target.Position - I:GetTargetPositionInfo(tracker.idx, tarIdx).Position
        e.oldAimpoints[idx] = e.aimpoints[idx] or (target.AimPointPosition - detectionError)
        e.aimpoints[idx] = target.AimPointPosition - detectionError

        observations[target.Id] = observations[target.Id] or {}
        observations[target.Id][idx] = CounterbatteryTypes.product(
          "target/aimpoint", target.Id, stamp, {
            frame = "world",
            sourceId = tracker.idx,
            position = e.aimpoints[idx],
            previousPosition = e.oldAimpoints[idx],
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
      oldAimpoints = track.state.oldAimpoints,
      aimpoints = track.state.aimpoints,
      state = track.state,
    })
  end
  return CounterbatteryTypes.collection("target/aimpoint", stamp, byId, {
    frame = "world",
  })
end
