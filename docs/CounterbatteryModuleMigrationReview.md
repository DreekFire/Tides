# Counterbattery module migration review

Status: partial and intentionally incomplete.

The canonical counterbattery path is:

```text
CounterBattery.lua:CoreUpdate
  → Pipeline/Entry.lua:UpdateEnemyData
  → Pipeline/TargetPipeline.lua:CounterbatteryPipeline:update
  → Pipeline/ModuleSystem.lua:update
  → typed target products
  → legacy firing and visualization consumers
```

`Pipeline/ModuleSystem.lua` is a synchronous output-DAG scheduler. It does not
create, resume, or yield coroutines. The coroutine-based manager currently
lives only in `ModuleExample.lua` and its integration tests, so the
counterbattery implementation has not completed that migration.

There is also existing composition debt: the graph entries named
`projectile-estimation` and `weapon-rate-of-fire` currently provide dependency
paths while the corresponding stateful work is still owned by
`TrackDeduplication.lua`. They are not independent coroutine modules. This is
left visible rather than described as completed modularization.

## Shell-health boundary

`Pipeline/ProjectileHealth.lua` defines the future
`target/track-cluster/projectile-health` product contract. The canonical
launcher sets `projectile-health` to `false`, and `TargetPipeline.new` rejects
an explicit attempt to enable it. There is no damage-measurement producer or
runtime observation call yet, so the pipeline cannot publish an estimated
health value and does not fall back to detection range, muzzle velocity, or
rate of fire.
