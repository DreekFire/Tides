# Implementation index

- `counterbattery-entry`: `Full/Counterbattery/CounterBattery.lua:CoreUpdate`
- `pipeline-entry`: `Full/Counterbattery/Pipeline/Entry.lua:UpdateEnemyData`
- `composition`: `Full/Counterbattery/Pipeline/TargetPipeline.lua` (canonical feature graph and flags)
- `scheduler`: `Full/Counterbattery/Pipeline/ModuleSystem.lua` (coroutine-backed output DAG and lifecycle manager)
- `coroutine-reference`: `ModuleExample.lua` (manager contract and standalone acceptance fixture)
- `tracking`: `Full/Counterbattery/Pipeline/TargetTracking.lua`
- `trajectory-metadata`: `Full/Counterbattery/Pipeline/TrackDeduplication.lua`
- `shell-health-boundary`: `Full/Counterbattery/Pipeline/ProjectileHealth.lua` (disabled)
- `artifact-builder`: `Util/concat_counterbattery.py`
- `paste-artifact`: `Full/Counterbattery/CounterBatteryPipeline.lua`

- `streaming-fourier`: `Low/StreamingFourier.lua` (`new`, `update`, `reset`); public declaration in `Header.lua`; rectangular sliding real DFT.
- `array`: `Low/NdArray.lua` (shape/stride/data/offset, iterators and math).
- `linear-algebra`: `Low/Linalg.lua`; `Low/Matrix3.lua` delegates inverse.
- `statistics`: `Low/Stats.lua` (NdArray covariance).
- Library public API and Fourier conventions: `README.md`.
- `target-enrichment`: `Full/Counterbattery/Pipeline/TargetAimpoint.lua`, `TargetRotation.lua`, `TargetSubobjectAimpoint.lua`, `TargetOrientation.lua`, `TargetDensity.lua`.
- `projectile-metadata`: `Full/Counterbattery/Pipeline/ProjectileEstimation.lua`, `WeaponRateOfFire.lua`; consumed by `TrackDeduplication.lua`.
- `closest-point`: `Full/Counterbattery/Pipeline/ClosestPointSolver.lua`; timestamp-to-ring-index adapter used by `Full/Counterbattery/CounterBattery.lua`.
- `debug`: `Full/Counterbattery/Pipeline/DebugVisualization.lua`; ALE reference in `docs/AtsuLuaEditor.md`, vendor snapshot refreshed by `Util/download_atsuluaeditor_docs.py`.
- `endpoint-test-builders`: `tests/build_tests.py`, `tests/build_module_tests.py`; fixtures `tests/combined_header.lua`, `tests/combined_footer.lua`, `tests/module_header.lua`, `tests/module_footer.lua`.
- `target-prediction`: `Medium/TargetPrediction.lua` (`new`, `update`, `predict`, `reset`, `kinematic`); depends on `Low/StreamingFourier.lua`, `Low/FourierWindow.lua`, and Vector3.
- `prediction-comparison`: `Medium/TargetPredictionComparison.lua` owns target lifecycle, forecast queues and matched-origin metrics; independent of Counterbattery.
- `prediction-launcher`: `Full/TargetPredictionComparison.lua:Update`, `GetTargetPredictionComparisonReport`; detailed contracts in `docs/TargetPrediction.md`.

- Frequency-dependent harmonic windows: `Low/FourierWindow.lua` (`new`, `halfLife`, `weight`, `fit`); canonical weighted least-squares owner, depends on NdArray/Linalg.
