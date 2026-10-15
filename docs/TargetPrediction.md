# Standalone target prediction

## Ownership and execution

- Models: `Medium/TargetPrediction.lua`; transform: `Low/StreamingFourier.lua`.
- Weighted harmonic fitting: `Low/FourierWindow.lua`; uses `NdArray` and `Linalg`.
- Comparison and target lifecycle: `Medium/TargetPredictionComparison.lua`.
- Public declarations: `Header.lua`; library artifact: `Tides.lua` (`Util/concat.py`).
- FtD launcher: `Full/TargetPredictionComparison.lua`; canonical paste order:
  `Header.lua` → launcher → `Tides.lua`, outside any function.
- Runtime: `Update(I)` reads mainframe 0 authoritative `TargetInfo.Position`,
  calls the comparison, returns its report, and logs per-horizon RMSE every 5 s.
- Latest report: `GetTargetPredictionComparisonReport()`; errors before first Update.
- This is an observational comparison module. Counterbattery is independent.

## Library API

```lua
-- Run initialization after the pasted library definitions have executed.
local comparison = TargetPredictionComparison.new({
    windowSize = 128, sampleInterval = 0.1, harmonics = 4,
    windowCycles = 1, windowMinHalfLife = 1.6, windowMaxHalfLife = 12.8,
    minRelativePower = 0.01, maxGap = 0.5, horizons = {0.5, 1, 2},
})
local report = comparison:update({[42] = Vector3(100, 20, 30)}, now)
local target = report.byId[42]
local forecast = target.forecasts[2].models.fourier -- horizon 1 s
if forecast.ready then
    local predictedPosition = forecast.position
end
local metrics = target.horizons[2].models.circle
-- metrics.count; meanError/rmse/maxError exist only after count > 0.
```

Direct single-target model API:

```lua
local model = TargetPrediction.new({windowSize = 128, sampleInterval = 0.1})
local accepted, reason = TargetPrediction.update(model, worldPosition, now)
local forecast = TargetPrediction.predict(model, "fourier", 1.5)
TargetPrediction.reset(model)
-- Analytic derivative inputs, without sampled history:
local point = TargetPrediction.kinematic(position, velocity, acceleration, 1.5, "circle")
```

- Models: `fourier`, `linear`, `quadratic`, `circle`; unknown names fail.
- Units: positions in world metres, timestamps/horizons in seconds.
- Inputs: finite XYZ vectors and finite timestamps; horizon finite and nonnegative.
- Complete target map per frame; absent IDs are lost and their histories/scores removed.
- Repeated timestamps: `accepted=false`, reason `duplicate-timestamp`; no mutation or scoring.
- Backwards clock / gap > `maxGap`: new epoch, clear spectra, pending forecasts and metrics.
- Reacquisition begins a fresh history. Supplied positions are copied into private state.
- Outputs are read-only snapshots; later updates do not mutate prior reports.

## Model contract

| Model | Estimate | Ready after |
| --- | --- | --- |
| Linear | Last position-difference velocity, constant velocity | 2 observations |
| Quadratic | Nonuniform 3-point derivative, endpoint velocity, constant acceleration | 3 observations |
| Circle | Same derivatives; constant speed and curvature in osculating plane | 3 observations |
| Fourier | DC drift + selected periodic displacement components | N uniform increments |

For the last observations `(p0,t0),(p1,t1),(p2,t2)`:

- `v01=(p1-p0)/(t1-t0)`, `v12=(p2-p1)/(t2-t1)`.
- `a=2(v12-v01)/(t2-t0)`; endpoint velocity `v=v12+a(t2-t1)/2`.
- Linear uses `v12`; quadratic uses `p2+v*h+a*h²/2`.
- Circle uses `aN=a-v*(a·v)/(v·v)`, `w=|aN|/|v|`,
  `p2+v*sin(w*h)/w+aN*(1-cos(w*h))/w²`.
- Circle excludes tangential acceleration; zero speed gives stationary motion,
  zero curvature gives straight motion. Small angles use the analytic limit.

Fourier processing:

1. Linearly resample actual observations on an anchored grid of spacing `dt`.
2. Feed each XYZ position increment to its own `StreamingFourier` instance.
3. Require N genuine increments (N+1 grid positions), without padding forecasts.
4. Convert increment DFT `D[k]` to position coefficient
   `P[k]=D[k]/(1-exp(-i*w[k]))`; retain DC as steady translational drift.
5. Rank bins jointly by XYZ displacement power; retain at most `harmonics`
   whose power is at least `minRelativePower` times the strongest bin.
6. Refit the selected increment harmonics jointly with exponential age weights.
   Each bin uses its own half-life; DC uses the maximum. Convert the fitted
   increment coefficients to position coefficients with the same integration.
7. Extrapolate phases at `w[k]=2*pi*k/(N*dt)`. Subtract the phase at the current
   observation and anchor displacement to the latest actual position, including
   observations between grid points. Fractional horizons are supported.

Defaults: N=128, dt=0.1 s, 4 bins, relative power cutoff 0.01, gap=0.5 s.
History duration `N*dt=12.8 s`; frequency resolution `1/(N*dt)` Hz.
Nyquist is counted once; odd windows retain normal conjugate-pair scaling.
For K selected bins, the weighted basis has P <= 2K+1 columns. Each update
uses at most K+1 weighted fits, each O(N*P² + P³), plus O(N log N) bin ranking.
Working storage is O(N*P + P²), plus pending forecasts; each forecast costs O(K).
Raw kinematic derivatives are unsmoothed. Resampling has no anti-alias filter.

## Frequency-dependent window API

`FourierWindow.new(N, dt, options)` configures the reusable fitter.
`halfLife(window, bin)` returns seconds; `weight(window, bin, ageSeconds)`
returns `2^(-ageSeconds/halfLife)`. For positive frequency f:

`halfLife = clamp(windowCycles/f, windowMinHalfLife, windowMaxHalfLife)`

Defaults are one cycle, minimum history duration/8, maximum history duration.
At N=128 and dt=0.1, bin 1 retains a 12.8 s half-life, bin 2 uses 6.4 s,
and bins 8 and above use 1.6 s. DC uses 12.8 s. The finite N-sample history
still imposes a hard cutoff. Minimum half-life must be at least history
duration/16 to bound conditioning; all options must be positive and max >= min.

`FourierWindow.fit(window, signals, bins)` accepts full aligned StreamingFourier
states and distinct positive bins. It returns DC per signal and cosine/sine
coefficients per bin, phased at the newest sample. Every weighting fits the
entire selected basis to avoid crosstalk from nonorthogonal tapered columns;
equal half-lives share a solve. Missing history and invalid options fail clearly.
Frequency selection still uses the rectangular DFT. This improves adaptation
after selection; it does not refine off-bin frequencies or guarantee lower noise.

## Comparison result v1

- Collection: `type="target/prediction-comparison"`, `schemaVersion=1`, `frame="world"`,
  `stamp`, `byId`.
- Target: same tag/version/frame, `targetId`, `sourceStamp`, `epoch`, `accepted`,
  `reason`, `status`, `sampleCount`, `windowSize`, `sampleInterval`, `pendingCount`.
- `forecasts[i]`: `horizon`, `models[model]` with `ready`, `sourceStamp`, `horizon`,
  `model`, and either `position` or an explicit warmup `reason`.
- Ready Fourier forecasts also expose `window`: method `frequency-exponential`,
  `cycles`, `minHalfLife`, `maxHalfLife`, `dcHalfLife`, and per-component
  `bin`, `frequency` (Hz), `halfLife` (seconds).
- `horizons[i]`: `horizon`, `models[model]` with `count`, and after scoring,
  mean Euclidean error, RMSE, maximum error (all metres).
- `winners`: all models within 1e-6 m RMSE of the minimum; `winner` only for a
  unique winner. No winner before scoring; ties are not arbitrary victories.
- `matchedOrigins=true`; ground truth label `timestamp-bracket-linear-interpolation`.

Scoring rules:

- Issue all four forecasts together only when all four models are ready;
  warmup is excluded for every model. At most one issue per dt.
- Save forecast positions at issue time. Score only once the future due time
  has been bracketed by actual observations; interpolate that bracket.
- Compare identical origins, horizons and sample counts. Never refit a saved
  forecast with future observations. Bracket interpolation is an approximation
  to true motion; high cadence reduces its error.
- Accumulate metrics over the current target epoch. Gap/reset/loss clears
  pending evaluations instead of scoring across discontinuities.

## Fourier versus circle and helix

A steady helix is representable as `c + u*t + a*cos(w*t) + b*sin(w*t)`;
a circle is the zero-drift special case. Thus a suitable Fourier basis can
represent either motion. Representation does not imply reliable estimation.

The implemented predictor has a fixed frequency grid. Noninteger cycles per
window leak across bins; extrapolating those bins can fail badly. Changing
frequency, short arcs, sudden turns, long horizons, aliasing and limited history
also break its repeated-motion assumption. No continuous frequency refinement
or automatic model switching is implemented. Recursive DFT roundoff can
accumulate on long uninterrupted streams; reset on a new independent track.

An osculating-circle model estimates local curvature from recent motion and
can react before a full period is observed. A helical model constrains axial
drift and rotation jointly; this can reduce estimation variance when those
assumptions hold. Neither is made redundant by a more expressive Fourier basis.
A separate helical predictor is not part of this four-model comparison.

Mathematical references: [NIST Fourier series](https://dlmf.nist.gov/1.8),
[MIT curvature and torsion](https://math.mit.edu/~djk/18_022/chapter02/section07.html).

## Verification

- `.venv/bin/python tests/run_target_prediction_tests.py`: source modules,
  Header + generated library, and actual standalone FtD Update paste order.
- Add `--ale` for Lua 5.1; add `--report` for deterministic synthetic RMSE rows.
- `tests/run_tests.sh` / `tests/run_tests.sh --ale`: include these contracts.
- Scenarios: stationary, drift, acceleration, bin-aligned circle, helix,
  multiple harmonics, off-bin circle, short arc, noise, changing turn, turn onset.
- Contracts: actual timestamp derivatives, resampling phase, odd/Nyquist bins,
  fractional horizons, invalid inputs, duplicate/gap/clock handling, target
  isolation/loss, bounded queues, causal scoring, ties, snapshot preservation.
- Results are synthetic validation, not live FtD accuracy measurements.
