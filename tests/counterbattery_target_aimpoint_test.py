#!/usr/bin/env python3
"""Regression tests for true-CoM target/aimpoint semantics."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetAimpoint.lua"
TRACKING_SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetTracking.lua"
LEGACY_SOURCE = ROOT / "Full" / "Counterbattery" / "CounterBattery.lua"


def main() -> int:
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.execute(
        """
        local vectorMeta = {}
        vectorMeta.__add = function(a, b)
          return Vec(a.x + b.x, a.y + b.y, a.z + b.z)
        end
        vectorMeta.__sub = function(a, b)
          return Vec(a.x - b.x, a.y - b.y, a.z - b.z)
        end
        function Vec(x, y, z)
          return setmetatable({x = x, y = y, z = z}, vectorMeta)
        end

        CounterbatteryTypes = {}
        function CounterbatteryTypes.product(path, targetId, stamp, fields)
          local product = {type = path, targetId = targetId, stamp = stamp}
          for key, value in pairs(fields or {}) do
            product[key] = value
          end
          return product
        end
        function CounterbatteryTypes.collection(path, stamp, byId, fields)
          local product = CounterbatteryTypes.product(path, nil, stamp, fields)
          product.byId = byId or {}
          return product
        end
        """
    )
    module = lua.execute(SOURCE.read_text(encoding="utf-8") + "\nreturn TargetAimpoint")
    lua.globals().TargetAimpointModule = module
    result = lua.execute(
        """
        local state = {oldAimpoints = {}, aimpoints = {}}
        local sample = {
          target = {
            Valid = true,
            Id = 42,
            Position = Vec(100, 20, 5),
            AimPointPosition = Vec(103, 22, 8),
          },
        }
        local interface = {
          GetNumberOfTargets = function(_, _) return 1 end,
          GetTargetInfo = function(_, _, _) return sample.target end,
        }
        local tracks = {byId = {[42] = {state = state}}}
        local tracker = {idx = 7}
        local aimpoint = TargetAimpointModule.new({
          enemies = {[42] = state},
          aimPointTrackers = {tracker},
        })

        local first = TargetAimpointModule.update(aimpoint, interface, tracks, 12.5)
        local firstTarget = first.byId[42]
        local firstObservation = firstTarget.observations[1]
        assert(first.frame == "world")
        assert(firstTarget.frame == "world")
        assert(firstObservation.frame == "world")
        assert(firstObservation.position.x == 103)
        assert(firstObservation.position.y == 22)
        assert(firstObservation.position.z == 8)
        assert(firstObservation.previousPosition.x == 103)
        assert(firstObservation.previousPosition.y == 22)
        assert(firstObservation.previousPosition.z == 8)
        assert(firstObservation.trueCenterOfMass.x == 100)
        assert(firstObservation.trueCenterOfMass.y == 20)
        assert(firstObservation.trueCenterOfMass.z == 5)
        assert(firstObservation.relativePosition.x == 3)
        assert(firstObservation.relativePosition.y == 2)
        assert(firstObservation.relativePosition.z == 3)
        assert(firstTarget.state == state)
        assert(state.aimpoints[1].x == 103)
        assert(state.aimpoints[1].y == 22)
        assert(state.aimpoints[1].z == 8)

        sample.target = {
          Valid = true,
          Id = 42,
          Position = Vec(101, 21, 6),
          AimPointPosition = Vec(104, 23, 9),
        }
        local second = TargetAimpointModule.update(aimpoint, interface, tracks, 13.5)
        local secondObservation = second.byId[42].observations[1]
        assert(secondObservation.position.x == 104)
        assert(secondObservation.position.y == 23)
        assert(secondObservation.position.z == 9)
        assert(secondObservation.previousPosition.x == 103)
        assert(secondObservation.previousPosition.y == 22)
        assert(secondObservation.previousPosition.z == 8)
        assert(second.stamp == 13.5)
        assert(secondObservation.stamp == 13.5)

        return true
        """
    )
    assert result is True
    assert "GetTargetPositionInfo" not in SOURCE.read_text(encoding="utf-8")
    tracking_source = TRACKING_SOURCE.read_text(encoding="utf-8")
    assert "GetTargetPositionInfo" not in tracking_source
    assert "RingBuffer.push(e.pos, target.Position)" in tracking_source
    assert "RingBuffer.push(e.vel, velocity)" in tracking_source
    assert "target.Velocity" in tracking_source
    legacy_source = LEGACY_SOURCE.read_text(encoding="utf-8")
    assert "GetTargetPositionInfo" not in legacy_source
    assert "detectionError" not in legacy_source
    assert "target.Velocity" in legacy_source
    print("Counterbattery target/aimpoint semantics: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
