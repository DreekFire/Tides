#!/usr/bin/env python3
"""Contract tests for the intentionally disabled shell-health boundary."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
TYPES_SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "Types.lua"
HEALTH_SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "ProjectileHealth.lua"
PIPELINE_SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetPipeline.lua"


def main() -> int:
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.execute(TYPES_SOURCE.read_text(encoding="utf-8"))
    # Types.lua is intentionally local to the concatenated artifact.  Supply
    # the same tiny public boundary that the standalone module test needs.
    lua.execute(
        """
        CounterbatteryTypes = {}
        function CounterbatteryTypes.product(path, targetId, stamp, fields)
          local product = {type = path, targetId = targetId, stamp = stamp}
          for key, value in pairs(fields or {}) do product[key] = value end
          return product
        end
        """
    )
    health = lua.execute(HEALTH_SOURCE.read_text(encoding="utf-8") + "\nreturn CounterbatteryProjectileHealth")

    instance = health.new({})
    assert instance.isAvailable(instance) is False
    placeholder = health.placeholder(17, 123)
    assert placeholder["type"] == "target/track-cluster/projectile-health"
    assert placeholder["targetId"] == 17
    assert placeholder["stamp"] == 123
    assert placeholder["enabled"] is False
    assert placeholder["available"] is False
    assert placeholder["estimate"] is None
    assert placeholder["sampleCount"] == 0
    assert placeholder["reason"] == "damage-measurement-producer-not-implemented"

    # Opt-in must fail instead of fabricating health or falling back to legacy data.
    pipeline_source = PIPELINE_SOURCE.read_text(encoding="utf-8")
    pipeline = lua.execute(pipeline_source + "\nreturn CounterbatteryPipeline")
    try:
        pipeline.new(
            lua.table_from({"features": lua.table_from({"projectile-health": True})})
        )
    except Exception as error:
        assert "placeholder" in str(error)
        assert "damage measurement" in str(error)
    else:
        raise AssertionError("enabling projectile-health must fail")

    assert 'name = "projectile-health"' in pipeline_source
    assert 'enabled = false' in pipeline_source
    assert 'context.damageObservations' in pipeline_source
    print("Counterbattery projectile health boundary: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
