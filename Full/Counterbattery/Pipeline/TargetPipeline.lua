-- The in-process ROS-like composition layer.  Each stage returns a new typed
-- product; only the stage's private target state owns mutable ring buffers.
local CounterbatteryPipeline = {}

local FEATURE_ALIASES = {
  ["target-tracking"] = "targetTracking",
  ["target-aimpoint"] = "targetAimpoint",
  ["target-rotation"] = "targetRotation",
  ["target-subobject-aimpoint"] = "targetSubobjectAimpoint",
  ["target-orientation"] = "targetOrientation",
  ["target-density"] = "targetDensity",
  ["projectile-estimation"] = "projectileEstimation",
  ["weapon-rate-of-fire"] = "weaponRateOfFire",
  ["track-deduplication"] = "trackDeduplication",
  ["projectile-health"] = "projectileHealth",
  ["debug-visualization"] = "debugVisualization",
}

local function featureEnabled(config, name, legacyName)
  local features = config.features or config.enabledFeatures
  local alias = FEATURE_ALIASES[name]
  if features and features[name] ~= nil then return features[name] == true end
  if features and alias and features[alias] ~= nil then return features[alias] == true end
  if legacyName and config[legacyName] ~= nil then return config[legacyName] == true end
  return name ~= "debug-visualization" and name ~= "projectile-health"
end

function CounterbatteryPipeline.new(config)
  config = config or {}
  if featureEnabled(config, "projectile-health", "projectileHealth") then
    error("counterbattery projectile health is a placeholder; damage measurement is not implemented", 0)
  end
  if config.features or config.enabledFeatures then
    -- The debug stage has its own renderer flag for standalone use.  Keep it
    -- synchronized with the module flag when the pipeline is configured as a
    -- feature graph.
    config.debugVisualization = featureEnabled(config, "debug-visualization", "debugVisualization")
  end
  local pipeline = {
    targetTracking = TargetTracking.new(config),
    targetAimpoint = TargetAimpoint.new(config),
    targetRotation = TargetRotation.new(config),
    targetSubobjectAimpoint = TargetSubobjectAimpoint.new(config),
    targetOrientation = TargetOrientation.new(config),
    targetDensity = TargetDensity.new(config),
    trackDeduplication = TargetTrackDeduplication.new(config),
    projectileHealth = CounterbatteryProjectileHealth.new(config),
    debugVisualization = CounterbatteryDebugVisualization.new(config),
    lastProjectileObservation = nil,
    lastMissileObservation = nil,
  }
  pipeline.modules = CounterbatteryModuleSystem.new({
    {
      name = "target-tracking",
      enabled = featureEnabled(config, "target-tracking", "enableTargetTracking"),
      provides = {"target/track"},
      update = function(context)
        return pipeline.targetTracking:update(context.I, context.stamp)
      end,
    },
    {
      name = "target-aimpoint",
      enabled = featureEnabled(config, "target-aimpoint", "enableTargetAimpoint"),
      requires = {"target/track"},
      provides = {"target/aimpoint"},
      update = function(context)
        return pipeline.targetAimpoint:update(context.I,
          context.outputs["target/track"], context.stamp)
      end,
    },
    {
      name = "target-rotation",
      enabled = featureEnabled(config, "target-rotation", "enableTargetRotation"),
      requires = {"target/track", "target/aimpoint"},
      provides = {"target/rotation"},
      update = function(context)
        return pipeline.targetRotation:update(context.I,
          context.outputs["target/track"], context.outputs["target/aimpoint"], context.stamp)
      end,
    },
    {
      name = "target-subobject-aimpoint",
      enabled = featureEnabled(config, "target-subobject-aimpoint", "enableTargetSubobjectAimpoint"),
      requires = {"target/track", "target/aimpoint", "target/rotation"},
      provides = {"target/subobject-aimpoint"},
      update = function(context)
        return pipeline.targetSubobjectAimpoint:update(context.I,
          context.outputs["target/track"], context.outputs["target/aimpoint"],
          context.outputs["target/rotation"], context.stamp)
      end,
    },
    {
      name = "target-orientation",
      enabled = featureEnabled(config, "target-orientation", "enableTargetOrientation"),
      requires = {"target/track", "target/aimpoint", "target/rotation", "target/subobject-aimpoint"},
      provides = {"target/orientation"},
      update = function(context)
        return pipeline.targetOrientation:update(context.I,
          context.outputs["target/track"], context.outputs["target/aimpoint"],
          context.outputs["target/rotation"], context.outputs["target/subobject-aimpoint"],
          context.stamp)
      end,
    },
    {
      name = "target-density",
      enabled = featureEnabled(config, "target-density", "enableTargetDensity"),
      requires = {"target/track", "target/aimpoint", "target/rotation"},
      provides = {"target/density"},
      update = function(context)
        return pipeline.targetDensity:update(context.I,
          context.outputs["target/track"], context.outputs["target/aimpoint"],
          context.outputs["target/rotation"], context.stamp)
      end,
    },
    {
      name = "projectile-estimation",
      enabled = featureEnabled(config, "projectile-estimation", "enableProjectileEstimation"),
      requires = {"target/density"},
      provides = {"target/projectile-estimation"},
      update = function(context)
        return true
      end,
    },
    {
      name = "weapon-rate-of-fire",
      enabled = featureEnabled(config, "weapon-rate-of-fire", "enableWeaponRateOfFire"),
      requires = {"target/projectile-estimation"},
      provides = {"target/weapon-rate-of-fire"},
      update = function(context)
        return true
      end,
    },
    {
      name = "track-deduplication",
      enabled = featureEnabled(config, "track-deduplication", "enableTrackDeduplication"),
      requires = {"target/density", "target/projectile-estimation", "target/weapon-rate-of-fire"},
      provides = {"target/track-cluster"},
      update = function(context)
        return pipeline.trackDeduplication:update(context.I,
          context.outputs["target/density"], context.stamp)
      end,
    },
    {
      name = "projectile-health",
      -- This is an explicit future boundary, not a no-op implementation.
      -- Enabling it is rejected above until damage observations exist.
      enabled = false,
      requires = {"target/track-cluster"},
      provides = {"target/track-cluster/projectile-health"},
      update = function(context)
        return pipeline.projectileHealth:update(
          context.I, context.outputs["target/track-cluster"],
          context.damageObservations, context.stamp)
      end,
    },
    {
      name = "debug-visualization",
      enabled = featureEnabled(config, "debug-visualization", "debugVisualization"),
      requires = {"target/track", "target/aimpoint", "target/rotation",
        "target/subobject-aimpoint", "target/orientation", "target/density",
        "target/track-cluster"},
      provides = {"debug/counterbattery-visualization"},
      update = function(context)
        return pipeline.debugVisualization:update(context.I, context.outputs,
          pipeline.lastMissileObservation, pipeline.lastProjectileObservation,
          context.stamp)
      end,
    },
  }, {
    stopOnError = config.stopOnModuleError,
  })
  return pipeline
end

function CounterbatteryPipeline:update(I, stamp)
  self.outputs = self.modules:update({I = I, stamp = stamp, outputs = {}})
  return self.outputs
end

function CounterbatteryPipeline:observeProjectile(I, observation, stamp)
  self.lastProjectileObservation = self.trackDeduplication:observeProjectile(
    I, observation, stamp)
  return self.lastProjectileObservation
end

function CounterbatteryPipeline:observeMissiles(I, observations, stamp)
  self.lastMissileObservation = self.trackDeduplication:observeMissiles(
    I, observations, stamp)
  return self.lastMissileObservation
end

function CounterbatteryPipeline:visualize(I, stamp)
  return self.debugVisualization:update(I, self.outputs,
    self.lastMissileObservation, self.lastProjectileObservation, stamp)
end
