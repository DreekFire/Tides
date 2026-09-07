# `target/aimpoint` coordinate and time semantics

The FtD target API fields used by this stage provide world-space vectors:

- `TargetInfo.Position`: the authoritative target center of mass;
- `TargetInfo.Velocity`: the authoritative target velocity;
- `TargetInfo.AimPointPosition`: the world position of the currently selected
  target aimpoint;

The counterbattery implementation does not consume `TargetPositionInfo`.
That detected center-of-mass measurement is noisy and is unnecessary when the
`TargetInfo` ground-truth state is available. Aimpoints are therefore copied
directly:

```text
aimpointWorld = TargetInfo.AimPointPosition
relativePosition = aimpointWorld - TargetInfo.Position
```

`aimpointWorld` remains absolute/world-space for the rotation estimator, while
`relativePosition` is the same observation relative to the true COM for
geometry stages. The target tracking history also records
`TargetInfo.Position` and `TargetInfo.Velocity` directly.

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
from `GetTimeSinceSpawn()`. The stamp is attached to the ground-truth sample
read during that update, so ring-buffer history and derived products share an
explicit synchronization value. The stage keeps the existing
mutable `oldAimpoints`/`aimpoints` arrays owned by the target state; the typed
products only reference that state and do not replace it.

For API field descriptions, see the community-maintained FtD Lua binding
reference: <https://gist.github.com/tvagames/9c7c29ec749c85ecab87cc74624dd34b>.
