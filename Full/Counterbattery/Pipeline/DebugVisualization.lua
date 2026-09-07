-- debug/counterbattery-visualization
--
-- This stage turns pipeline products into immutable draw descriptions.  The
-- AtsuLuaEditor renderer is deliberately kept at the edge: the pipeline can
-- be tested without ALE, and the normal FtD interpreter never receives draw
-- syntax unless debugVisualization is explicitly enabled.
local CounterbatteryDebugVisualization = {}

local function addPrimitive(self, primitives, kind, fields)
  if #primitives >= self.maxPrimitives then return false end
  local primitive = {type = kind}
  for key, value in pairs(fields or {}) do primitive[key] = value end
  primitives[#primitives + 1] = primitive
  return true
end

local function add(self, primitives, kind, fields)
  if addPrimitive(self, primitives, kind, fields) then return end
  self.dropped = self.dropped + 1
end

local function validVector(value)
  return value and type(value.x) == "number"
    and type(value.y) == "number" and type(value.z) == "number"
end

local function validQuaternion(value)
  return value and type(value.x) == "number"
    and type(value.y) == "number" and type(value.z) == "number"
    and type(value.w) == "number"
end

local function centerOfMass(track)
  return track and (track.trueCenterOfMass or track.position)
end

local function rotate(rotation, vector)
  if not rotation or not vector then return vector end
  local ok, value = pcall(function() return rotation * vector end)
  if ok and validVector(value) then return value end
  return vector
end

local function addAxes(self, primitives, position, rotation, length, color)
  if not validVector(position) or not rotation then return end
  local axes = {
    Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1),
  }
  for _, axis in ipairs(axes) do
    local endpoint = position + rotate(rotation, axis) * length
    add(self, primitives, "arrow", {
      from = position, to = endpoint, color = color,
    })
  end
end

function CounterbatteryDebugVisualization.new(config)
  config = config or {}
  return setmetatable({
    enabled = config.debugVisualization == true,
    duration = config.debugVisualizationDuration or 2,
    maxPrimitives = config.debugVisualizationMaxPrimitives or 128,
    axisLength = config.debugVisualizationAxisLength or 10,
    trajectoryLength = config.debugVisualizationTrajectoryLength or 100,
    missileRadius = config.debugVisualizationMissileRadius or 3,
    warnedUnavailable = false,
    warnedRenderFailure = false,
    lastProduct = nil,
  }, {__index = CounterbatteryDebugVisualization})
end

function CounterbatteryDebugVisualization:collect(outputs, missileObservation,
                                                   projectileObservation, stamp)
  local primitives = {}
  self.dropped = 0
  if not self.enabled then
    return CounterbatteryTypes.product("debug/counterbattery-visualization",
      nil, stamp, {enabled = false, primitives = {}, primitiveCount = 0})
  end

  local tracks = outputs and outputs["target/track"]
  local aimpoints = outputs and outputs["target/aimpoint"]
  local rotations = outputs and outputs["target/rotation"]
  local orientations = outputs and outputs["target/orientation"]
  local subobjects = outputs and outputs["target/subobject-aimpoint"]
  local densities = outputs and outputs["target/density"]
  local clusters = outputs and outputs["target/track-cluster"]

  for id, track in pairs(tracks and tracks.byId or {}) do
    local position = centerOfMass(track)
    local rotation = rotations and rotations.byId[id]
    local orientation = orientations and orientations.byId[id]
    local density = densities and densities.byId[id]
    local targetClusters = clusters and clusters.byId[id]
    local targetAimpoints = aimpoints and aimpoints.byId[id]
    local targetSubobjects = subobjects and subobjects.byId[id]

    if validVector(position) then
      add(self, primitives, "sphere", {
        pos = position, radius = 2, color = "yellow", targetId = id,
      })
    end

    for _, observation in pairs(targetAimpoints and targetAimpoints.observations or {}) do
      if validVector(observation.position) then
        add(self, primitives, "sphere", {
          pos = observation.position, radius = 1, color = "green", targetId = id,
        })
        if validVector(position) then
          add(self, primitives, "line", {
            from = position, to = observation.position, color = "green", targetId = id,
          })
        end
      end
    end

    if rotation and validQuaternion(rotation.rotation) then
      addAxes(self, primitives, position, rotation.rotation, self.axisLength, "blue")
    end
    if orientation and validQuaternion(orientation.orientation)
        and validVector(position) then
      add(self, primitives, "gimbal", {
        pos = position, rot = orientation.orientation,
        radius = self.axisLength, color = "cyan", targetId = id,
      })
    end

    for _, observation in pairs(targetSubobjects and targetSubobjects.observations or {}) do
      if validVector(observation.position) then
        add(self, primitives, "sphere", {
          pos = observation.position, radius = 1.5, color = "magenta", targetId = id,
        })
        if validVector(observation.subobjectRotationAxis) then
          local axis = observation.subobjectRotationAxis
          if rotation then axis = rotate(rotation.rotation, axis) end
          local half = axis * self.axisLength
          add(self, primitives, "line", {
            from = observation.position - half,
            to = observation.position + half,
            color = "magenta", targetId = id,
          })
        end
      end
    end

    local distribution = density and density.distribution
    local mean = distribution and distribution.mean
    if validVector(position) and mean and type(mean.x) == "number"
        and type(mean.y) == "number" and type(mean.z) == "number" then
      local offset = Vector3(mean.x, mean.y, mean.z)
      if rotation then offset = rotate(rotation.rotation, offset) end
      add(self, primitives, "sphere", {
        pos = position + offset, radius = 0.75, color = "white", targetId = id,
      })
    end

    for _, cluster in ipairs(targetClusters and targetClusters.clusters or {}) do
      local line = cluster.line
      if line and validVector(line.point) and validVector(line.direction) then
        local half = line.direction * self.trajectoryLength
        add(self, primitives, "dash", {
          from = line.point - half, to = line.point + half,
          on = 8, off = 4, color = "#ff8800", targetId = id,
        })
        add(self, primitives, "sphere", {
          pos = line.point, radius = 1.5, color = "#ff8800", targetId = id,
        })
      end
    end
  end

  for _, observation in pairs(missileObservation and missileObservation.byId or {}) do
    if validVector(observation.position) then
      add(self, primitives, "sphere", {
        pos = observation.position, radius = self.missileRadius,
        color = "red", missileId = observation.id,
      })
    end
  end

  if projectileObservation and validVector(projectileObservation.position) then
    add(self, primitives, "sphere", {
      pos = projectileObservation.position, radius = 1,
      color = projectileObservation.excluded and "red" or "#00ff00",
      excludedByMissileId = projectileObservation.excludedByMissileId,
    })
  end

  return CounterbatteryTypes.product("debug/counterbattery-visualization",
    nil, stamp, {
      enabled = true,
      primitives = primitives,
      primitiveCount = #primitives,
      droppedPrimitiveCount = self.dropped,
      renderer = "atsuluaeditor-logtohud",
    })
end

local function renderMessage(primitive, duration)
  local prefix = "draw color " .. tostring(primitive.color or "white")
    .. " duration " .. tostring(duration) .. " "
  if primitive.type == "sphere" then
    return prefix .. "sphere $pos $radius", {
      pos = primitive.pos, radius = primitive.radius,
    }
  elseif primitive.type == "line" or primitive.type == "arrow" then
    return prefix .. primitive.type .. " $from $to", {
      from = primitive.from, to = primitive.to,
    }
  elseif primitive.type == "dash" then
    return prefix .. "dash $from $to $on $off", {
      from = primitive.from, to = primitive.to,
      on = primitive.on, off = primitive.off,
    }
  elseif primitive.type == "gimbal" then
    return prefix .. "gimbal $pos $rot $radius", {
      pos = primitive.pos, rot = primitive.rot, radius = primitive.radius,
    }
  end
  return nil
end

function CounterbatteryDebugVisualization:render(I, product)
  if not self.enabled then return end
  if type(_ALE) ~= "string" or not I or type(I.LogToHud) ~= "function" then
    if not self.warnedUnavailable and I and type(I.Log) == "function" then
      I:Log("Counterbattery debug visualization requires AtsuLuaEditor")
      self.warnedUnavailable = true
    end
    return
  end

  for _, primitive in ipairs(product.primitives or {}) do
    local message, args = renderMessage(primitive, self.duration)
    if message then
      local ok, err = pcall(function() I:LogToHud(message, args) end)
      if not ok and not self.warnedRenderFailure and type(I.Log) == "function" then
        I:Log("Counterbattery debug visualization failed: " .. tostring(err))
        self.warnedRenderFailure = true
      end
    end
  end
end

function CounterbatteryDebugVisualization:update(I, outputs, missiles,
                                                  projectile, stamp)
  local product = self:collect(outputs, missiles, projectile, stamp)
  self.lastProduct = product
  self:render(I, product)
  return product
end

_G.CounterbatteryDebugVisualization = CounterbatteryDebugVisualization
