-- Pipeline entrypoint appended to the legacy counterbattery script.
-- The original implementation remains above this section as a reference;
-- this definition is the active UpdateEnemyData dispatch in the generated
-- CounterBatteryPipeline.lua artifact.
local counterbatteryPipeline

local function getCounterbatteryPipeline()
  if not counterbatteryPipeline then
    counterbatteryPipeline = CounterbatteryPipeline.new({
      enemies = enemies,
      targetTrackTime = targetTrackTime,
      TICKS_PER_S = TICKS_PER_S,
      numOrigins = numOrigins,
      trackClusterDistance = trackClusterDistance,
      aimPointTrackers = aimPointTrackers,
      maxTurnRateTracking = maxTurnRateTracking,
      onTargetLost = function(id)
        if id == currentTargetId then
          lastOrigin = nil
          lastOriginSwitchTime = -originSwitchTime - 1
          currentTargetId = nil
        end
      end,
    })
  end
  return counterbatteryPipeline
end

function UpdateEnemyData(I)
  return getCounterbatteryPipeline():update(I, t)
end
