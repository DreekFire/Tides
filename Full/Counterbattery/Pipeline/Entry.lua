-- Pipeline entrypoint appended to the legacy counterbattery script.
-- The original implementation remains above this section as a reference;
-- this definition is the active UpdateEnemyData dispatch in the generated
-- CounterBatteryPipeline.lua artifact.
local counterbatteryPipeline

function GetCounterbatteryPipeline()
  if not counterbatteryPipeline then
    counterbatteryPipeline = CounterbatteryPipeline.new({
      enemies = enemies,
      targetTrackTime = targetTrackTime,
      TICKS_PER_S = TICKS_PER_S,
      numOrigins = numOrigins,
      trackClusterDistance = trackClusterDistance,
      missileOverlapDistance = missileOverlapDistance,
      aimPointTrackers = aimPointTrackers,
      maxTurnRateTracking = maxTurnRateTracking,
      debugVisualization = counterbatteryDebugVisualization,
      debugVisualizationDuration = counterbatteryDebugVisualizationDuration,
      debugVisualizationMaxPrimitives = counterbatteryDebugVisualizationMaxPrimitives,
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
  return GetCounterbatteryPipeline():update(I, t)
end

function ObserveCounterbatteryProjectile(I, observation)
  return GetCounterbatteryPipeline():observeProjectile(I, observation, t)
end

function ObserveCounterbatteryMissiles(I, observations)
  return GetCounterbatteryPipeline():observeMissiles(I, observations, t)
end

function VisualizeCounterbattery(I)
  return GetCounterbatteryPipeline():visualize(I, t)
end
