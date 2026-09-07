#!/usr/bin/env python3
"""Regression tests for the ALE edge renderer and visualization product."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "DebugVisualization.lua"


def main() -> int:
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.execute(
        """
        local vectorMeta = {}
        vectorMeta.__add = function(a, b) return Vec(a.x + b.x, a.y + b.y, a.z + b.z) end
        vectorMeta.__sub = function(a, b) return Vec(a.x - b.x, a.y - b.y, a.z - b.z) end
        vectorMeta.__mul = function(a, b)
          if type(a) == "number" then return Vec(a * b.x, a * b.y, a * b.z) end
          return Vec(a.x * b, a.y * b, a.z * b)
        end
        function Vec(x, y, z)
          return setmetatable({x = x, y = y, z = z,
            sqrMagnitude = x * x + y * y + z * z}, vectorMeta)
        end
        Vector3 = setmetatable({}, {__call = function(_, x, y, z)
          return Vec(x, y, z)
        end})
        local quaternionMeta = {}
        quaternionMeta.__mul = function(rotation, vector)
          if vector.x then return vector end
          return {x = rotation.x, y = rotation.y, z = rotation.z, w = rotation.w}
        end
        local identity = setmetatable({x = 0, y = 0, z = 0, w = 1}, quaternionMeta)
        Quaternion = {identity = identity}
        CounterbatteryTypes = {}
        function CounterbatteryTypes.product(path, targetId, stamp, fields)
          local product = {type = path, targetId = targetId, stamp = stamp}
          for key, value in pairs(fields or {}) do product[key] = value end
          return product
        end
        """
    )
    module = lua.execute(SOURCE.read_text(encoding="utf-8") + "\nreturn CounterbatteryDebugVisualization")
    lua.globals().DebugVisualization = module
    result = lua.execute(
        """
        local debug = DebugVisualization.new({
          debugVisualization = true,
          debugVisualizationDuration = 4,
          debugVisualizationMaxPrimitives = 64,
          debugVisualizationAxisLength = 5,
          debugVisualizationTrajectoryLength = 20,
        })
        local q = Quaternion.identity
        local outputs = {
          ["target/track"] = {byId = {[7] = {
            trueCenterOfMass = Vec(10, 0, 0), position = Vec(10, 0, 0),
          }}},
          ["target/aimpoint"] = {byId = {[7] = {
            observations = {[1] = {position = Vec(12, 0, 0)}}
          }}},
          ["target/rotation"] = {byId = {[7] = {rotation = q}}},
          ["target/orientation"] = {byId = {[7] = {orientation = q}}},
          ["target/subobject-aimpoint"] = {byId = {[7] = {
            observations = {[1] = {
              position = Vec(11, 1, 0), subobjectRotationAxis = Vec(0, 1, 0),
            }}
          }}},
          ["target/density"] = {byId = {[7] = {
            distribution = {mean = {x = 1, y = 0, z = 0}},
          }}},
          ["target/track-cluster"] = {byId = {[7] = {clusters = {
            {line = {point = Vec(20, 0, 0), direction = Vec(1, 0, 0)}}
          }}}},
        }
        local missiles = {byId = {[1] = {id = 22, position = Vec(30, 0, 0)}}}
        local projectile = {position = Vec(25, 0, 0), excluded = true,
          excludedByMissileId = 22}
        local product = debug:collect(outputs, missiles, projectile, 12)
        assert(product.type == "debug/counterbattery-visualization")
        assert(product.enabled == true)
        assert(product.primitiveCount >= 12)
        assert(product.droppedPrimitiveCount == 0)
        assert(debug.lastProduct == nil)

        local calls = {}
        local logs = {}
        local I = {}
        function I:LogToHud(message, args)
          calls[#calls + 1] = {message = message, args = args}
        end
        function I:Log(message) logs[#logs + 1] = message end
        _ALE = "2.1.1"
        local rendered = debug:update(I, outputs, missiles, projectile, 13)
        assert(rendered == debug.lastProduct)
        assert(#calls == rendered.primitiveCount)
        assert(string.find(calls[1].message, "draw color") ~= nil)
        assert(string.find(calls[1].message, "duration 4") ~= nil)

        _ALE = nil
        local callsBefore = #calls
        debug:update(I, outputs, missiles, projectile, 14)
        assert(#calls == callsBefore)
        assert(#logs == 1)
        debug:update(I, outputs, missiles, projectile, 15)
        assert(#logs == 1)

        local disabled = DebugVisualization.new({debugVisualization = false})
        local disabledProduct = disabled:update(I, outputs, missiles, projectile, 16)
        assert(disabledProduct.enabled == false)
        assert(disabledProduct.primitiveCount == 0)
        assert(#calls == callsBefore)
        return true
        """
    )
    assert result is True
    source = SOURCE.read_text(encoding="utf-8")
    assert "debug/counterbattery-visualization" in source
    assert "atsuluaeditor-logtohud" in source
    assert "LogToHud" in source
    print("Counterbattery debug visualization: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
