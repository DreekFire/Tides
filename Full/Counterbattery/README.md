# Counterbattery pipeline

`CounterBattery.lua` is the original monolithic implementation and is kept
as a behavioral reference during the modularization work.

The modular source lives in `Pipeline/` and is assembled into the paste-ready
`CounterBatteryPipeline.lua` artifact with:

```sh
.venv/bin/python Util/concat_counterbattery.py
```

The pipeline emits typed, path-like products:

```text
target/track → target/aimpoint → target/rotation → target/subobject-aimpoint
                                  ↘ target/orientation
                                                → target/density
           → target/track-cluster
```

Products are separate values keyed by target ID.  Pipeline stages may retain
private history, but they do not add fields to or replace the preceding
product.

The pipeline uses the coroutine lifecycle model from `ModuleExample.lua`:
each enabled feature has a private coroutine, and the manager resumes those
coroutines in dependency order. Module failures are logged, retried according
to their policy, and optionally stop the manager. A failed prerequisite blocks
its dependents for that tick.
Individual stages can be enabled with `config.features`, for example:

```lua
features = {
  ["target-tracking"] = true,
  ["target-aimpoint"] = true,
  ["target-rotation"] = true,
  ["target-subobject-aimpoint"] = false,
  ["target-orientation"] = false,
  ["target-density"] = true,
  ["projectile-estimation"] = true,
  ["weapon-rate-of-fire"] = true,
  ["track-deduplication"] = true,
  ["projectile-health"] = false,
  ["debug-visualization"] = false,
}
```

An enabled stage whose prerequisite is disabled fails during pipeline
construction with the missing output name.  The scheduler also rejects cycles
and reports the complete dependency path.  This keeps a feature switch from
silently changing the meaning of downstream products.

`target/track-cluster` linearizes each reconstructed projectile origin into a
world-space tangent line and publishes deterministic clusters using shortest
line distance.  It is observational only: the legacy origin ring and firing
selection are unchanged until a later stage explicitly consumes this product.
Each cluster also carries projectile detection-range bounds, an observed muzzle
velocity estimate, optional size metadata, and a persistent per-weapon
rate-of-fire estimate. Sensor reacquisition intervals are represented as
conservative bounds rather than as a fabricated exact detection distance.

`Pipeline/ProjectileEstimation.lua` records the sensor projectile stream,
including missed ticks and reacquisition intervals. `Pipeline/WeaponRateOfFire.lua`
associates reconstructed lines over time and estimates firing cadence.

`Pipeline/ProjectileHealth.lua` reserves the typed
`target/track-cluster/projectile-health` boundary for a future module that
measures per-projectile damage. It is disabled in the canonical launcher and
rejects explicit enablement until that producer exists; no shell-health value
is inferred from the existing range, velocity, or rate-of-fire observations.

`Pipeline/TargetSubobjectAimpoint.lua` compares true-CoM-relative aimpoints
with the main-body rotation product. Residual points are published as a
separate enrichment, with a temporal rotation-axis estimate when three or
more samples support one. Conflicting axes are marked as potentially stacked
subobjects; the original aimpoint and rotation products remain unchanged.

`Pipeline/TargetOrientation.lua` estimates a separate world-frame orientation
from exact grid-constrained aimpoints. It evaluates axes inferred from
subobject motion first, then bounded integer-grid/yaw candidates under the
normal local-Y/global-Y spawn assumption. Candidate scoring uses grid residual
and temporal continuity; mothership-spawned targets can disable the Y-alignment
assumption in the pipeline configuration. The existing rotation estimate is a
fallback and is never overwritten.

`Pipeline/DebugVisualization.lua` is an optional terminal stage. It publishes
`debug/counterbattery-visualization` draw primitives for the true target CoM,
aimpoints, body/orientation axes, subobject axes, density mean, reconstructed
trajectory lines, missile warnings, and the current ballistic observation.
When `counterbatteryDebugVisualization` is enabled, an edge adapter emits the
primitives through the documented AtsuLuaEditor `I:LogToHud` draw syntax. It
does not call the draw overload when `_ALE` is unavailable, and logs one
warning instead. See `docs/AtsuLuaEditor.md` for the downloaded documentation
snapshot and attribution.

The generated artifact bundles `Low/NdArray.lua` and the NdArray-backed
`Low/Stats.lua`, so density covariance is an NdArray.  Legacy Matrix3-only
callers convert at that boundary rather than changing the distribution.

`Pipeline/ClosestPointSolver.lua` is the active closest-point implementation.
It stores an explicit timestamp beside each authoritative target position and
its velocity sample. The first sample bootstraps velocity from the API's
detection-derived value; later samples use position differences, while
non-increasing timestamps retain the previous velocity estimate.
uses `Search.interpolatedSearch` to map absolute times back to ring indices,
freezes rotation only for the analytic Mahalanobis cubic, and uses each
sample's rotation during the final scan.  `CounterBattery.lua` samples target
observations on every eligible game tick; timestamps, rather than a cadence
counter or reconstructed fixed-rate index, define synchronization.
The origin precheck and missile-origin lookup use the same timestamp-to-ring
index mapping.  The compatibility helpers retain a fixed-rate fallback only
for old target records that have no timestamp ring; newly recorded targets
and the generated artifact use explicit timestamps throughout the active
path.
