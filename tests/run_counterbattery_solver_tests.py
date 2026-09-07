#!/usr/bin/env python3
"""Run closest-point solver and timestamp-indexing regression tests."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "ClosestPointSolver.lua"
TEST = ROOT / "tests" / "counterbattery" / "closest_point_solver_test.lua"
LEGACY_SOURCE = ROOT / "Full" / "Counterbattery" / "CounterBattery.lua"
RING_BUFFER_SOURCE = ROOT / "Low" / "RingBuffer.lua"


PRELUDE = r"""
TICKS_PER_S = 10

local vectorMeta = {}
vectorMeta.__add = function(a, b)
  return Vec(a.x + b.x, a.y + b.y, a.z + b.z)
end
vectorMeta.__sub = function(a, b)
  return Vec(a.x - b.x, a.y - b.y, a.z - b.z)
end
vectorMeta.__unm = function(a)
  return Vec(-a.x, -a.y, -a.z)
end
vectorMeta.__mul = function(a, b)
  if type(a) == "number" then return Vec(a * b.x, a * b.y, a * b.z) end
  return Vec(a.x * b, a.y * b, a.z * b)
end
vectorMeta.__div = function(a, b)
  return Vec(a.x / b, a.y / b, a.z / b)
end
function Vec(x, y, z)
  return setmetatable({x = x, y = y, z = z}, vectorMeta)
end

Vector3 = {}
Vector3.Dot = function(a, b) return a.x * b.x + a.y * b.y + a.z * b.z end

local matrixMeta = {}
Matrix3 = {}
function Matrix3.Matrix3(values)
  local matrix = {}
  for i = 1, 9 do matrix[i] = values[i] end
  return setmetatable(matrix, matrixMeta)
end
function Matrix3.mul(a, b)
  if getmetatable(b) == matrixMeta then
    local values = {}
    for row = 1, 3 do
      for col = 1, 3 do
        local value = 0
        for k = 1, 3 do value = value + a[(row - 1) * 3 + k] * b[(k - 1) * 3 + col] end
        values[(row - 1) * 3 + col] = value
      end
    end
    return Matrix3.Matrix3(values)
  end
  return Vec(
    a[1] * b.x + a[2] * b.y + a[3] * b.z,
    a[4] * b.x + a[5] * b.y + a[6] * b.z,
    a[7] * b.x + a[8] * b.y + a[9] * b.z)
end
matrixMeta.__mul = Matrix3.mul
function Matrix3.quadform(a, matrix, b)
  return Vector3.Dot(a, matrix * b)
end
function Matrix3.inverse(matrix)
  return Matrix3.Matrix3({
    1 / matrix[1], 0, 0,
    0, 1 / matrix[5], 0,
    0, 0, 1 / matrix[9]})
end

MathUtil = {}
MathUtil.solveCubic = function() return nil end
MathUtil.solveQuadratic = function() return nil end

Quaternion = {}
local quaternionMeta = {}
quaternionMeta.__mul = function(_, vector) return vector end
Quaternion.identity = setmetatable({}, quaternionMeta)
Quaternion.Inverse = function(rotation) return rotation end

Search = {calls = 0}
Search.interpolatedSearch = function(list, left, right, target, findClosest)
  Search.calls = Search.calls + 1
  local best, distance
  for index = left, right do
    local current = math.abs(list[index] - target)
    if not distance or current < distance then best, distance = index, current end
  end
  return findClosest and best or nil
end
"""


def main() -> int:
    lua = LuaRuntime(unpack_returned_tuples=True)
    lua.execute(PRELUDE)
    lua.execute("RingBuffer = {}")
    lua.execute(RING_BUFFER_SOURCE.read_text(encoding="utf-8"))
    lua.execute(SOURCE.read_text(encoding="utf-8"))
    lua.execute(TEST.read_text(encoding="utf-8"))

    source = SOURCE.read_text(encoding="utf-8")
    assert "covInv = -covInv" not in source
    assert "for row=1,3" not in source
    legacy_source = LEGACY_SOURCE.read_text(encoding="utf-8")
    assert "nextRecordTime" not in legacy_source
    assert "t >= nextRecordTime" not in legacy_source
    assert "\n  UpdateEnemyData(I)\n" in legacy_source
    print("Counterbattery every-tick sampling gate removal: ok")
    print("Counterbattery solver source workaround removal: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
