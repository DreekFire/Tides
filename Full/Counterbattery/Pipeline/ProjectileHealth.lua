-- target/track-cluster/projectile-health
-- Placeholder boundary for the future projectile-damage measurement module.
--
-- FtD does not currently provide the per-projectile damage observations this
-- product needs.  Do not infer shell health from detection range, muzzle
-- velocity, or rate of fire: those are different observables.  The boundary
-- stays disabled until a producer can supply damage deltas for a tracked
-- projectile.
local CounterbatteryProjectileHealth = {}

local PATH = "target/track-cluster/projectile-health"

function CounterbatteryProjectileHealth.new(config)
  return setmetatable({config = config or {}, enabled = false},
    {__index = CounterbatteryProjectileHealth})
end

function CounterbatteryProjectileHealth.placeholder(targetId, stamp)
  return CounterbatteryTypes.product(PATH, targetId, stamp, {
    enabled = false,
    available = false,
    estimate = nil,
    lowerBound = nil,
    upperBound = nil,
    sampleCount = 0,
    source = "projectile-damage-measurement",
    reason = "damage-measurement-producer-not-implemented",
  })
end

function CounterbatteryProjectileHealth:isAvailable()
  return false
end

function CounterbatteryProjectileHealth:update()
  error("counterbattery projectile health requires the unimplemented damage-measurement producer", 0)
end

function CounterbatteryProjectileHealth:observeDamage()
  error("counterbattery projectile health cannot observe damage until its producer is implemented", 0)
end

_G.CounterbatteryProjectileHealth = CounterbatteryProjectileHealth
