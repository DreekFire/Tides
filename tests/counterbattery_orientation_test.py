#!/usr/bin/env python3
"""Regression tests for grid-constrained target orientation enrichment."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TargetOrientation.lua"


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
        vectorMeta.__div = function(a, b) return Vec(a.x / b, a.y / b, a.z / b) end
        function Vec(x, y, z)
          return setmetatable({x = x, y = y, z = z,
            sqrMagnitude = x * x + y * y + z * z}, vectorMeta)
        end
        Vector3 = setmetatable({}, {__call = function(_, x, y, z)
          return Vec(x, y, z)
        end})
        Vector3.Dot = function(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end
        Vector3.Cross = function(a, b)
          return Vec(a.y * b.z - a.z * b.y,
                     a.z * b.x - a.x * b.z,
                     a.x * b.y - a.y * b.x)
        end

        local quaternionMeta = {}
        quaternionMeta.__mul = function(rotation, vector)
          if type(vector) == "table" and vector.x then
            local c, s = math.cos(rotation.yaw), math.sin(rotation.yaw)
            return Vec(c * vector.x + s * vector.z, vector.y,
                       -s * vector.x + c * vector.z)
          end
          return {yaw = rotation.yaw + vector.yaw}
        end
        local function rotation(yaw)
          return setmetatable({yaw = yaw, x = 0, y = math.sin(yaw / 2),
                               z = 0, w = math.cos(yaw / 2)}, quaternionMeta)
        end
        Quaternion = {
          identity = rotation(0),
          Inverse = function(value) return rotation(-value.yaw) end,
          AngleAxis = function(degrees, _) return rotation(math.rad(degrees)) end,
          LookRotation = function(_, _) return rotation(0) end,
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
    module = lua.execute(SOURCE.read_text(encoding="utf-8") + "\nreturn TargetOrientation")
    lua.globals().OrientationModule = module
    lua.globals().Orientation = module.new(lua.table())
    result = lua.execute(
        """
        local tracks = {byId = {[7] = {state = {}}}}
        local aimpoints = {byId = {[7] = {observations = {
          {relativePosition = Vec(1, 0, 0)},
          {relativePosition = Vec(0, 0, 1)},
          {relativePosition = Vec(2, 1, 0)},
        }}}}
        local rotations = {byId = {[7] = {rotation = Quaternion.identity}}}
        local subobjects = {byId = {}}
        local product = Orientation:update(nil, tracks, aimpoints, rotations,
                                          subobjects, 12.5)
        local target = product.byId[7]
        assert(product.type == "target/orientation")
        assert(target.status == "estimated")
        assert(target.targetYAligned == true)
        assert(target.candidateCount > 0)
        assert(target.gridInliers >= 3)
        assert(target.gridResidual < 1e-9)

        local freeOrientation = OrientationModule.new({assumeTargetYAligned = false})
        local freeProduct = freeOrientation:update(nil, tracks, aimpoints, rotations,
                                                   subobjects, 12.5)
        assert(freeProduct.byId[7].targetYAligned == false)
        assert(freeProduct.byId[7].candidateCount == 1)
        return true
        """
    )
    assert result is True
    print("Counterbattery orientation enrichment: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
