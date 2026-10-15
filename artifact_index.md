# Artifact index

- `target/track`: target tracking product.
- `target/aimpoint`, `target/rotation`, `target/subobject-aimpoint`, `target/orientation`: immutable target enrichments.
- `target/density`: aimpoint-derived density product.
- `target/track-cluster`: reconstructed trajectory clusters with detection-range, muzzle-velocity, size-proxy, and rate-of-fire metadata.
- `target/track-cluster/projectile-health`: reserved typed boundary; disabled and unpublished until damage observations exist.
- `debug/counterbattery-visualization`: optional diagnostic draw product.
- `Full/Counterbattery/CounterBatteryPipeline.lua`: generated paste-ready artifact consumed by the game; source of truth is `Full/Counterbattery/Pipeline/` plus the legacy counterbattery source files.

- `Tides.lua`: canonical library artifact, sorted `Low/`, `Medium/`, `High/` sources via `Util/concat.py`; paired with `Header.lua`.
- Fourier state: window `size`, filled `count`, reused `real`/`imag` arrays indexed k+1, k=0..floor(N/2); finite real samples, uniform cadence owned by caller.
- `target/prediction-comparison` v1: in-memory world-frame collection `stamp/byId`; targets expose ready-gated `forecasts`, per-horizon `count/meanError/rmse/maxError`, tie-aware `winners`; owned by `Medium/TargetPredictionComparison.lua`, consumed by standalone launcher/logs. Contract: `docs/TargetPrediction.md`.
- Prediction input: complete target-ID → finite XYZ position map plus finite timestamp; no Counterbattery product required. Generated `Tides.lua` includes both prediction modules; no new persisted artifact.
