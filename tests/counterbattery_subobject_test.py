#!/usr/bin/env python3
"""Deterministic tests for subobject aimpoint enrichment."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetSubobjectAimpoint.lua"


def main() -> int:
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.execute(
        """
        local vectorMeta = {}
        vectorMeta.__add = function(a, b) return Vec(a.x + b.x, a.y + b.y, a.z + b.z) end
        vectorMeta.__sub = function(a, b) return Vec(a.x - b.x, a.y - b.y, a.z - b.z) end
        vectorMeta.__div = function(a, b) return Vec(a.x / b, a.y / b, a.z / b) end
        function Vec(x, y, z)
          return setmetatable({x = x, y = y, z = z,
            sqrMagnitude = x * x + y * y + z * z}, vectorMeta)
        end
        Vector3 = {}
        Vector3.Dot = function(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
        Vector3.Cross = function(a, b)
          return Vec(a.y * b.z - a.z * b.y,
                     a.z * b.x - a.x * b.z,
                     a.x * b.y - a.y * b.x)
        end
        local quaternionMeta = {}
        quaternionMeta.__mul = function(rotation, vector) return vector end
        Quaternion = {
          identity = setmetatable({}, quaternionMeta),
          Inverse = function(rotation) return rotation end,
        }
        CounterbatteryTypes = {}
        function CounterbatteryTypes.product(path, targetId, stamp, fields)
          local product = {type = path, targetId = targetId, stamp = stamp}
          for key, value in pairs(fields or {}) do product[key] = value end
          return product
        end
        function CounterbatteryTypes.collection(path, stamp, byId, fields)
          local product = CounterbatteryTypes.product(path, nil, stamp, fields)
          product.byId = byId or {}
          return product
        end
        """
    )
    module = lua.execute(SOURCE.read_text(encoding="utf-8") + "\nreturn TargetSubobjectAimpoint")
    lua.globals().Subobject = module.new(lua.table())
    result = lua.execute(
        """
        local state = {}
        local tracks = {byId = {[7] = {state = state}}}
        local rotation = {
          byId = {[7] = {deltaRotation = Quaternion.identity,
                         rotation = Quaternion.identity}},
        }
        local function update(point, previous)
          return Subobject:update(nil, tracks, {
            byId = {[7] = {observations = {
              [1] = {sourceId = 1, position = point, previousPosition = previous,
                     relativePosition = point, previousRelativePosition = previous,
                     trueCenterOfMass = Vec(0, 0, 0),
                     previousTrueCenterOfMass = Vec(0, 0, 0)},
              [2] = {sourceId = 2, position = Vec(5, 0, 0),
                     previousPosition = Vec(5, 0, 0), relativePosition = Vec(5, 0, 0),
                     previousRelativePosition = Vec(5, 0, 0)},
            }}}}, rotation, 1)
        end
        local first = update(Vec(10, 0, 0), Vec(10, 0, 0))
        assert(first.byId[7].count == 0)
        local second = update(Vec(0, 0, 10), Vec(10, 0, 0))
        assert(second.byId[7].count == 1)
        assert(second.byId[7].observations[1].axisStatus == "underdetermined")
        local third = update(Vec(-10, 0, 0), Vec(0, 0, 10))
        local found = third.byId[7].observations[1]
        assert(third.type == "target/subobject-aimpoint")
        assert(found.axisStatus == "estimated")
        assert(math.abs(found.subobjectRotationAxis.y) > 0.99)
        assert(third.byId[7].stackedSubobject == false)
        return true
        """
    )
    assert result is True
    print("Counterbattery subobject aimpoint: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
