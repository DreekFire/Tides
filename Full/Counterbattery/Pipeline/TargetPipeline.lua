-- The in-process ROS-like composition layer.  Each stage returns a new typed
-- product; only the stage's private target state owns mutable ring buffers.
local CounterbatteryPipeline = {}

function CounterbatteryPipeline.new(config)
  local pipeline = {
    targetTracking = TargetTracking.new(config),
    targetAimpoint = TargetAimpoint.new(config),
    targetRotation = TargetRotation.new(config),
    targetSubobjectAimpoint = TargetSubobjectAimpoint.new(config),
    targetOrientation = TargetOrientation.new(config),
    targetDensity = TargetDensity.new(config),
    trackDeduplication = TargetTrackDeduplication.new(config),
  }
  return pipeline
end

function CounterbatteryPipeline:update(I, stamp)
  local track = self.targetTracking:update(I, stamp)
  local aimpoint = self.targetAimpoint:update(I, track, stamp)
  local rotation = self.targetRotation:update(I, track, aimpoint, stamp)
  local subobjectAimpoint = self.targetSubobjectAimpoint:update(
    I, track, aimpoint, rotation, stamp)
  local orientation = self.targetOrientation:update(
    I, track, aimpoint, rotation, subobjectAimpoint, stamp)
  local density = self.targetDensity:update(I, track, aimpoint, rotation, stamp)
  local trackClusters = self.trackDeduplication:update(I, density, stamp)

  -- Paths identify enrichment products without mutating the previous product.
  self.outputs = {
    ["target/track"] = track,
    ["target/aimpoint"] = aimpoint,
    ["target/rotation"] = rotation,
    ["target/subobject-aimpoint"] = subobjectAimpoint,
    ["target/orientation"] = orientation,
    ["target/density"] = density,
    ["target/track-cluster"] = trackClusters,
  }
  return self.outputs
end

function CounterbatteryPipeline:observeProjectile(I, observation, stamp)
  return self.trackDeduplication:observeProjectile(I, observation, stamp)
end

function CounterbatteryPipeline:observeMissiles(I, observations, stamp)
  return self.trackDeduplication:observeMissiles(I, observations, stamp)
end
