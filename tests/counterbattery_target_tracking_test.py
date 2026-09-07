#!/usr/bin/env python3
"""Regression tests for position-derived target velocity."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
RING_BUFFER_SOURCE = ROOT / "Low" / "RingBuffer.lua"
TRACKING_SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetTracking.lua"


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
        vectorMeta.__div = function(a, b)
          return Vec(a.x / b, a.y / b, a.z / b)
        end
        function Vec(x, y, z)
          return setmetatable({x = x, y = y, z = z}, vectorMeta)
        end
        Vector3 = {zero = Vec(0, 0, 0)}
        RingBuffer = {}

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

        Matrix3 = {
          Identity = function() return {} end,
          scalarmul = function(_, _) return {} end,
        }
        Stats = {
          Distribution = function(vars) return {n = 0, vars = vars} end,
        }
        mainframeIdx = 1
        """
    )
    lua.execute(RING_BUFFER_SOURCE.read_text(encoding="utf-8"))
    module = lua.execute(TRACKING_SOURCE.read_text(encoding="utf-8") + "\nreturn TargetTracking")
    lua.globals().TargetTrackingModule = module
    result = lua.execute(
        """
        local samples = {
          {Valid = true, Id = 42, Position = Vec(100, 20, 5), Velocity = Vec(999, 999, 999)},
          {Valid = true, Id = 42, Position = Vec(104, 22, 9), Velocity = Vec(-999, -999, -999)},
          {Valid = true, Id = 42, Position = Vec(106, 25, 10), Velocity = Vec(123, 123, 123)},
        }
        local sampleIndex = 1
        local interface = {
          GetNumberOfTargets = function(_, _) return 1 end,
          GetTargetInfo = function(_, _, _) return samples[sampleIndex] end,
        }
        local enemies = {}
        local tracker = TargetTrackingModule.new({
          enemies = enemies,
          targetTrackTime = 5,
          TICKS_PER_S = 10,
          numOrigins = 2,
        })

        local first = TargetTrackingModule.update(tracker, interface, 10.0)
        -- The detected velocity is used only to bootstrap the first sample.
        assert(first.byId[42].velocity.x == 999)
        assert(first.byId[42].velocity.y == 999)
        assert(first.byId[42].velocity.z == 999)

        sampleIndex = 2
        TargetTrackingModule.update(tracker, interface, 10.25)
        local second = TargetTrackingModule.update(tracker, interface, 10.25)
        -- The repeated timestamp is guarded and must not overwrite the valid
        -- derivative with an infinite/invalid value.
        assert(second.byId[42].velocity.x == 16)
        assert(second.byId[42].velocity.y == 8)
        assert(second.byId[42].velocity.z == 16)

        sampleIndex = 3
        local third = TargetTrackingModule.update(tracker, interface, 10.40)
        local velocity = third.byId[42].velocity
        local function near(actual, expected)
          return math.abs(actual - expected) < 1e-9
        end
        assert(near(velocity.x, 2 / 0.15))
        assert(near(velocity.y, 3 / 0.15))
        assert(near(velocity.z, 1 / 0.15))
        return true
        """
    )
    assert result is True
    tracking_source = TRACKING_SOURCE.read_text(encoding="utf-8")
    assert "TargetInfo.Velocity" in tracking_source
    assert "observedVelocity" in tracking_source
    assert "return (position - previousPosition) / deltaTime" in tracking_source
    assert "RingBuffer.push(e.vel, velocity)" in tracking_source
    print("Counterbattery target velocity derivation: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
