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
target/track → target/aimpoint → target/rotation → target/density
           → target/track-cluster
```

Products are separate values keyed by target ID.  Pipeline stages may retain
private history, but they do not add fields to or replace the preceding
product.

`target/track-cluster` linearizes each reconstructed projectile origin into a
world-space tangent line and publishes deterministic clusters using shortest
line distance.  It is observational only: the legacy origin ring and firing
selection are unchanged until a later stage explicitly consumes this product.

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
