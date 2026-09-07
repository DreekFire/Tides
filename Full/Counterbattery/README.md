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
```

Products are separate values keyed by target ID.  Pipeline stages may retain
private history, but they do not add fields to or replace the preceding
product.  The existing solver and the known statistical/math issues are
intentionally unchanged in this step.
