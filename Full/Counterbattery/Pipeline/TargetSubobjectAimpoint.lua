-- target/subobject-aimpoint
-- Aimpoint residuals that are not explained by the current main-body rigid
-- transform.  This stage only publishes enrichment products; it never edits
-- the target/aimpoint or target/rotation products it consumes.
local TargetSubobjectAimpoint = {}

local AXIS_EPSILON = 1e-9

local function vectorValid(vector)
  return vector and type(vector.x) == "number"
    and type(vector.y) == "number" and type(vector.z) == "number"
end

local function normalized(vector)
  if not vectorValid(vector) or vector.sqrMagnitude <= AXIS_EPSILON then
    return nil
  end
  return vector / math.sqrt(vector.sqrMagnitude)
end

local function axesAgree(first, second)
  return first and second and math.abs(Vector3.Dot(first, second)) >= 0.95
end

local function relativePosition(observation, previous)
  if previous then
    if observation.previousRelativePosition then
      return observation.previousRelativePosition
    end
    if observation.previousPosition and observation.previousTrueCenterOfMass then
      return observation.previousPosition - observation.previousTrueCenterOfMass
    end
    return nil
  end
  if observation.relativePosition then
    return observation.relativePosition
  end
  if observation.position and observation.trueCenterOfMass then
    return observation.position - observation.trueCenterOfMass
  end
  return nil
end

local function axisFromHistory(history)
  if #history < 3 then return nil, "insufficient-temporal-history" end
  local firstDifference = history[#history - 1] - history[#history - 2]
  local secondDifference = history[#history] - history[#history - 1]
  local axis = normalized(Vector3.Cross(firstDifference, secondDifference))
  return axis, axis and "temporal-chords" or "collinear-temporal-difference"
end

function TargetSubobjectAimpoint.new(config)
  config = config or {}
  local threshold = config.subobjectAimpointResidualThreshold or 3
  return setmetatable({
    residualSqrThreshold = config.subobjectAimpointResidualSqrThreshold
      or threshold * threshold,
    historyByTarget = {},
  }, {__index = TargetSubobjectAimpoint})
end

function TargetSubobjectAimpoint:update(I, tracks, aimpoints, rotations, stamp)
  local byId = {}
  for id, track in pairs(tracks.byId) do
    local aimpoint = aimpoints.byId[id]
    local rotation = rotations.byId[id]
    local dRot = rotation and rotation.deltaRotation
    local absoluteRotation = rotation and rotation.rotation
    local candidates = {}
    local underdeterminedReason
    local axes = {}

    if not dRot then
      underdeterminedReason = "main-body-rotation-unavailable"
    else
      for index, observation in pairs((aimpoint and aimpoint.observations) or {}) do
        local currentRelative = relativePosition(observation, false)
        local previousRelative = relativePosition(observation, true)
        local history = self.historyByTarget[id]
        if not history then
          history = {}
          self.historyByTarget[id] = history
        end
        local sourceHistory = history[observation.sourceId or index]
        if not sourceHistory then
          sourceHistory = {}
          history[observation.sourceId or index] = sourceHistory
        end
        local bodyLocal = currentRelative
        if bodyLocal and absoluteRotation then
          bodyLocal = Quaternion.Inverse(absoluteRotation) * bodyLocal
        end
        if bodyLocal then
          sourceHistory[#sourceHistory + 1] = bodyLocal
          if #sourceHistory > 3 then table.remove(sourceHistory, 1) end
        end
        if currentRelative and previousRelative then
          local predictedRelative = dRot * previousRelative
          local residual = currentRelative - predictedRelative
          if residual.sqrMagnitude > self.residualSqrThreshold then
            local axis, axisMethod = axisFromHistory(sourceHistory)
            local axisStatus = axis and "estimated" or "underdetermined"
            if axis then axes[#axes + 1] = axis end
            candidates[#candidates + 1] = CounterbatteryTypes.product(
              "target/subobject-aimpoint", id, stamp, {
                frame = "target-relative",
                sourceId = observation.sourceId,
                aimpointIndex = index,
                position = observation.position,
                previousPosition = observation.previousPosition,
                trueCenterOfMass = observation.trueCenterOfMass,
                previousTrueCenterOfMass = observation.previousTrueCenterOfMass,
                relativePosition = currentRelative,
                previousRelativePosition = previousRelative,
                mainBodyPrediction = predictedRelative,
                residual = residual,
                residualMagnitude = math.sqrt(residual.sqrMagnitude),
                subobjectRotationAxis = axis,
                axisStatus = axisStatus,
                axisMethod = axisMethod,
                axisReason = axis and nil or axisMethod,
                status = "detected",
                state = track.state,
              })
            if not axis then
              underdeterminedReason = underdeterminedReason
                or "subobject-axis-underdetermined"
            end
          end
        elseif currentRelative then
          underdeterminedReason = underdeterminedReason
            or "insufficient-temporal-history"
        end
      end
    end

    -- Multiple residual aimpoints are not necessarily stacked: points on one
    -- turret should share an axis.  Mark stacking only when estimated axes
    -- disagree materially.
    local stackedSubobject = false
    for first = 1, #axes - 1 do
      for second = first + 1, #axes do
        if not axesAgree(axes[first], axes[second]) then
          stackedSubobject = true
        end
      end
    end
    local status
    if stackedSubobject then
      status = "stacked-subobject"
    elseif underdeterminedReason then
      status = "underdetermined"
    elseif #candidates > 0 then
      status = "detected"
    else
      status = "none"
    end

    byId[id] = CounterbatteryTypes.product("target/subobject-aimpoint", id,
      stamp, {
        frame = "target-relative",
        observations = candidates,
        count = #candidates,
        status = status,
        underdetermined = underdeterminedReason ~= nil,
        underdeterminedReason = underdeterminedReason,
        stackedSubobject = stackedSubobject,
        source = aimpoint,
        rotation = rotation,
        state = track.state,
      })
  end
  return CounterbatteryTypes.collection("target/subobject-aimpoint", stamp,
    byId, {frame = "target-relative"})
end
