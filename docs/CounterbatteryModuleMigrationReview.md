# Counterbattery module migration review

Status: coroutine-manager migration complete; projectile-health remains intentionally incomplete.

The canonical counterbattery path is:

```text
CounterBattery.lua:CoreUpdate
  → Pipeline/Entry.lua:UpdateEnemyData
  → Pipeline/TargetPipeline.lua:CounterbatteryPipeline:update
  → Pipeline/ModuleSystem.lua:update
  → typed target products
  → legacy firing and visualization consumers
```

`Pipeline/ModuleSystem.lua` combines the output-DAG scheduler with the
coroutine lifecycle contract from `ModuleExample.lua`: it creates one
coroutine per enabled module, resumes them in dependency order, logs failures,
recreates retryable modules, and honors stop-on-error configuration.

The graph entries named `projectile-estimation` and `weapon-rate-of-fire`
remain explicit dependency boundaries while their stateful work is owned by
`TrackDeduplication.lua`; they still execute through the manager’s coroutine
path and preserve the existing consumer contract.

## Shell-health boundary

`Pipeline/ProjectileHealth.lua` defines the future
`target/track-cluster/projectile-health` product contract. The canonical
launcher sets `projectile-health` to `false`, and `TargetPipeline.new` rejects
an explicit attempt to enable it. There is no damage-measurement producer or
runtime observation call yet, so the pipeline cannot publish an estimated
health value and does not fall back to detection range, muzzle velocity, or
rate of fire.
