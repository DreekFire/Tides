#!/usr/bin/env python3
"""Regression tests for target/aimpoint frame normalization."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetAimpoint.lua"


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
          targetPosition = Vec(110, 30, 15),
        }
        local interface = {
          GetNumberOfTargets = function(_, _) return 1 end,
          GetTargetInfo = function(_, _, _) return sample.target end,
          GetTargetPositionInfo = function(_, _, _)
            return {Position = sample.targetPosition}
          end,
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
        assert(firstObservation.position.x == 113)
        assert(firstObservation.position.y == 32)
        assert(firstObservation.position.z == 18)
        assert(firstObservation.previousPosition.x == 113)
        assert(firstObservation.previousPosition.y == 32)
        assert(firstObservation.previousPosition.z == 18)
        assert(firstTarget.state == state)
        assert(state.aimpoints[1].x == 113)
        assert(state.aimpoints[1].y == 32)
        assert(state.aimpoints[1].z == 18)

        sample.target = {
          Valid = true,
          Id = 42,
          Position = Vec(101, 21, 6),
          AimPointPosition = Vec(104, 23, 9),
        }
        sample.targetPosition = Vec(111, 31, 16)
        local second = TargetAimpointModule.update(aimpoint, interface, tracks, 13.5)
        local secondObservation = second.byId[42].observations[1]
        assert(secondObservation.position.x == 114)
        assert(secondObservation.position.y == 33)
        assert(secondObservation.position.z == 19)
        assert(secondObservation.previousPosition.x == 113)
        assert(secondObservation.previousPosition.y == 32)
        assert(secondObservation.previousPosition.z == 18)
        assert(second.stamp == 13.5)
        assert(secondObservation.stamp == 13.5)

        return true
        """
    )
    assert result is True
    print("Counterbattery target/aimpoint semantics: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
