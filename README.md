# Tides
## Lua Library for From the Depths

Project is heavily WIP, but I will try to keep the documentation up to date.

I am currently rewriting many of my old Lua scripts to use this library, which will be uploaded in one batch.

If you have any questions on how to use it, how it works, or how to program in general, contact me at Knife-Kun#3265 on Discord.

## Installation

Unfortunately, FtD does not provide a nice way to import libraries. Copy the content of `Header.lua` to the beginning of your lua script (outside of the Update function) and the content of `Tides.lua` anywhere else in your script, such as the very end (again, outside of any functions).

For the counterbattery example, the modular paste-ready artifact is
`Full/Counterbattery/CounterBatteryPipeline.lua`. Regenerate it after source
changes with `.venv/bin/python Util/concat_counterbattery.py`.

If you wish to reduce the size of your code, you can remove any classes that you don't need.

## Full, High, Medium, Low

The classes in Tides are divided into four categories based on their level of abstraction and specialization.

**Low Tide** contains Lua code for basic constructs that are generally useful in many applications. It contains some Unity-specific code but no FtD specific code.

**Medium Tide** contains code which builds on the structures defined in Low, and is slightly more specific to the fields of programming that FtD uses. It contains some Unity-specific code but no FtD specific code.

**High Tide** contains code that is narrowly applicable to applications in FtD and similar fields. It contains FtD-specific code.

**Full** contains complete examples. Currently, there are 3 examples:

1. Mouse Aim Aircraft:
* Mouse control of aircraft
* Manual and auto aim (slot 1 for manual, 2 for auto)
* Targeting, lock-on, and lead reticule (default lead reticule mode is "trailing," which shows where you will hit if you fire now)

2. Vehicle Missile Interceptor:
* Drone that rams incoming huge missiles
* Spreads out arms mounted on spinblocks prior to collision
* Rolls to rotate arms into place, using information about which arms have already been destroyed (obsolete, now that thump propagates through subobjects)

3. Counterbattery:
* Uses projectile avoidance routine to measure trajectories of incoming shells
* Traces those trajectories back to the time of closest approach to the enemy ship based on recorded enemy positions and returns fire
* Uses multiple AIs to gather aimpoints to track rotation

4. Ship Identification (WIP):
* Uses target prioritization cards to measure stats of enemy ships
* Compares stats to table of known ships to identify them
* May be used to improve counterbattery accuracy based on known locations of weapons

5. Swerve Drive (WIP):
* Rotates wheels to turn a tank, allowing it to drive in any direction
* Automatically detects and accounts for wheel placement to properly handle steering angle (i.e. Ackermann steering)

6. Ramp Targeting (WIP):
* Aims unguided missiles while taking into account initial launch velocity and acceleration time

## Streaming Fourier transform

`Low/StreamingFourier.lua` owns the reusable streaming DFT API. Build the
canonical paste artifact with `.venv/bin/python Util/concat.py`; paste
`Header.lua` before your script and `Tides.lua` after it, outside `Update`.

```lua
local spectrum = StreamingFourier.new(64) -- keep between Update calls
-- Call once per uniformly spaced real sample:
local ready = StreamingFourier.update(spectrum, sample)
if ready then
    local k = 3
    local re, im = spectrum.real[k + 1], spectrum.imag[k + 1]
    local amplitude = 2 * math.sqrt(re * re + im * im) / spectrum.size
end
-- StreamingFourier.reset(spectrum) starts an independent stream.
```

Each update costs O(N) time and retains O(N) storage, with arbitrary positive
integer window sizes. The rectangular window holds oldest-to-newest samples,
left-padded with zeros until ready. Coefficients use the unnormalized
negative-exponent DFT; bin k represents k * sampleRate / N Hz, for
k=0..floor(N/2). Negative bins follow by conjugate symmetry. For one-sided
amplitude divide magnitude by N and double interior bins only (exclude DC
and even-N Nyquist). `real` and `imag` are reused tables; copy values to retain
a snapshot. Inputs must be finite real numbers. Long-running recursive
transforms accumulate floating-point error; reset when starting a new stream.
The obsolete Fourier drafts and stale manually minified artifact were removed.

## Target prediction comparison

`Medium/TargetPrediction.lua` implements Fourier, linear, quadratic, and
osculating-circle forecasts. `Medium/TargetPredictionComparison.lua` compares
them against future observations using matched origins and fixed horizons.
They are reusable library modules. The standalone FtD launcher is
`Full/TargetPredictionComparison.lua`: paste `Header.lua`, the launcher, and
`Tides.lua` in that order. It observes target CoM positions and logs RMSE every
five seconds. Read the latest result with `GetTargetPredictionComparisonReport()`.

For another caller, create `TargetPredictionComparison.new(options)` once,
then call `comparison:update({[targetId] = worldPosition}, timestamp)` with the
complete visible-target map each observation. Results live in
`report.byId[targetId].forecasts` and `.horizons`. Initialization must happen
after the library definitions execute; the standalone launcher initializes on
its first `Update`.

The default Fourier history is 12.8 seconds. This implementation uses fixed
DFT bins, so off-bin periods and changing manoeuvres can produce large errors;
it does not make circle or helical prediction redundant. See
[the API and model contracts](docs/TargetPrediction.md).

Reproduce the synthetic comparisons with:

```sh
.venv/bin/python tests/run_target_prediction_tests.py --report
```
