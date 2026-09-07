-- The in-process ROS-like composition layer.  Each stage returns a new typed
-- product; only the stage's private target state owns mutable ring buffers.
local CounterbatteryPipeline = {}

function CounterbatteryPipeline.new(config)
  local pipeline = {
    targetTracking = TargetTracking.new(config),
    targetAimpoint = TargetAimpoint.new(config),
    targetRotation = TargetRotation.new(config),
    targetDensity = TargetDensity.new(config),
  }
  return pipeline
end

function CounterbatteryPipeline:update(I, stamp)
  local track = self.targetTracking:update(I, stamp)
  local aimpoint = self.targetAimpoint:update(I, track, stamp)
  local rotation = self.targetRotation:update(I, track, aimpoint, stamp)
  local density = self.targetDensity:update(I, track, aimpoint, rotation, stamp)

  -- Paths identify enrichment products without mutating the previous product.
  self.outputs = {
    ["target/track"] = track,
    ["target/aimpoint"] = aimpoint,
    ["target/rotation"] = rotation,
    ["target/density"] = density,
  }
  return self.outputs
end
