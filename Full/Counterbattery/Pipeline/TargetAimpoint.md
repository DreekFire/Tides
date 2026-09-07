# `target/aimpoint` coordinate and time semantics

The FtD target APIs used by this stage provide world-space vectors:

- `TargetInfo.Position`: the target position reported by that mainframe;
- `TargetInfo.AimPointPosition`: the world position of the currently selected
  target aimpoint;
- `TargetPositionInfo.Position`: the target-position estimate used to align the
  observation.

The stage copies the original monolith's normalization:

```text
aimpointWorld = TargetPositionInfo.Position
              + (AimPointPosition - TargetInfo.Position)
```

Therefore `aimpointWorld` is an absolute/world-space point. The parenthesized
term is an offset from the mainframe's target reference, not a proven offset
from the target's center of mass. The FtD API calls in this repository do not
provide a remote target center-of-mass value, so the implementation must not
label this value `target-com`.

The aimpoint stage also intentionally does not apply a rotation. The next
stage estimates the target rotation from differences between these world-space
points, and the density stage then computes the target-local value as:

```text
aimpointLocal = inverse(trackedRotation)
                * (aimpointWorld - trackedTargetPosition)
```

Applying rotation in `target/aimpoint` would either remove the information
needed by `target/rotation` or apply the previous rotation before the current
one is estimated. No behavioral correction is needed in this stage.

All observations produced during one pipeline update receive the same `stamp`
from `GetTimeSinceSpawn()`. The FtD target structures have no sample timestamp
in the fields consumed here, so the code assumes that the sequential API calls
within one update represent one usable snapshot. The stage keeps the existing
mutable `oldAimpoints`/`aimpoints` arrays owned by the target state; the typed
products only reference that state and do not replace it.

For API field descriptions, see the community-maintained FtD Lua binding
reference: <https://gist.github.com/tvagames/9c7c29ec749c85ecab87cc74624dd34b>.
