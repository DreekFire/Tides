# Artifact index

- `target/track`: target tracking product.
- `target/aimpoint`, `target/rotation`, `target/subobject-aimpoint`, `target/orientation`: immutable target enrichments.
- `target/density`: aimpoint-derived density product.
- `target/track-cluster`: reconstructed trajectory clusters with detection-range, muzzle-velocity, size-proxy, and rate-of-fire metadata.
- `target/track-cluster/projectile-health`: reserved typed boundary; disabled and unpublished until damage observations exist.
- `debug/counterbattery-visualization`: optional diagnostic draw product.
- `Full/Counterbattery/CounterBatteryPipeline.lua`: generated paste-ready artifact consumed by the game; source of truth is `Full/Counterbattery/Pipeline/` plus the legacy counterbattery source files.
