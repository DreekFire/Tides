#!/usr/bin/env python3
"""Deterministic tests for counterbattery trajectory deduplication."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "TrackDeduplication.lua"
PROJECTILE_SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "ProjectileEstimation.lua"
RATE_SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "WeaponRateOfFire.lua"


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
        quaternionMeta.__mul = function(rotation, vector)
          local c = math.cos(rotation.angle or 0)
          local s = math.sin(rotation.angle or 0)
          return Vec(c * vector.x + s * vector.z, vector.y,
                     -s * vector.x + c * vector.z)
        end
        Quaternion = {
          identity = setmetatable({angle = 0}, quaternionMeta),
          rotY90 = setmetatable({angle = math.pi / 2}, quaternionMeta),
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
    lua.execute(PROJECTILE_SOURCE.read_text(encoding="utf-8"))
    lua.execute(RATE_SOURCE.read_text(encoding="utf-8"))
    module = lua.execute(SOURCE.read_text(encoding="utf-8") + "\nreturn TargetTrackDeduplication")
    lua.globals().Dedup = module
    result = lua.execute(
        """
        local function near(actual, expected)
          return math.abs(actual - expected) < 1e-9
        end

        -- The shortest distance for parallel lines is their perpendicular
        -- separation; skew lines use the common-normal distance.
        local parallelA = {point = Vec(0, 0, 0), direction = Vec(1, 0, 0)}
        local parallelB = {point = Vec(0, 0, 3), direction = Vec(1, 0, 0)}
        local skew = {point = Vec(0, 1, 1), direction = Vec(0, 1, 0)}
        assert(near(Dedup.lineDistance(parallelA, parallelB), 3))
        assert(near(Dedup.lineDistance(parallelA, skew), 1))

        local state = {
          pos = {size = 1, [1] = Vec(10, 0, 0)},
          vel = {size = 1, [1] = Vec(0, 0, 0)},
          sampleTimes = {size = 1, [1] = 0},
          origins = {
            size = 4,
            -- Two observations of one cannon: parallel lines 0.5m apart.
            [1] = {position = Vec(0, 0, 0), relVel = Vec(100, 0, 0), rot = Quaternion.identity, time = 0},
            [2] = {position = Vec(0, 0.5, 0), relVel = Vec(100, 0, 0), rot = Quaternion.identity, time = 0},
            -- A separate cannon: its line is 10m away and perpendicular.
            [3] = {position = Vec(0, 10, 10), relVel = Vec(100, 0, 0), rot = Quaternion.rotY90, time = 0},
            -- Invalid zero-velocity origins do not become false clusters.
            [4] = {position = Vec(0, 0, 20), relVel = Vec(0, 0, 0), rot = Quaternion.identity, time = 0},
            -- Missile-warning origins are not ballistic clusters.
            [5] = {type = 2, position = Vec(0, 0, 0), relVel = Vec(100, 0, 0), rot = Quaternion.identity, time = 0},
          },
        }
        local dedup = Dedup.new({trackClusterDistance = 1, TICKS_PER_S = 40,
          maxDetectionRange = 1000,
          vehicleBounds = {center = Vec(0, 0, 0), minimum = Vec(-2, -1, -3), maximum = Vec(2, 1, 3)}})
        local miss = dedup:observeProjectile({GetConstructCenterOfMass = function() return Vec(0, 0, 0) end},
          {streamId = 'sensor-projectile', detected = false, position = Vec(0, 0, 20)}, 1)
        assert(miss.detected == false)
        local reacquired = dedup:observeProjectile({GetConstructCenterOfMass = function() return Vec(0, 0, 0) end},
          {streamId = 'sensor-projectile', detected = true, position = Vec(0, 0, 10), reportedRange = 10}, 2)
        assert(reacquired.reacquired == true)
        assert(reacquired.detectionRangeLower >= 7)
        assert(reacquired.detectionRangeUpper >= 10)
        -- A missile warning is a separate stream.  A coincident ballistic
        -- reading is excluded before it can create a cannon trajectory.
        dedup:observeMissiles(nil, {{id = 42, position = Vec(0, 0, 10)}}, 3)
        local missileOverlap = dedup:observeProjectile(nil, {
          streamId = 'sensor-projectile', detected = true, position = Vec(0, 0, 10)}, 3)
        assert(missileOverlap.accepted == false)
        assert(missileOverlap.excludedByMissileId == 42)
        -- Reacquisition after a missile exclusion must not manufacture a
        -- detection threshold interval.
        local postMissile = dedup:observeProjectile(nil, {
          streamId = 'sensor-projectile', detected = true, position = Vec(0, 0, 9)}, 4)
        assert(postMissile.reacquired == false)
        local clusters = dedup:cluster(state, 0)
        assert(#clusters == 2)
        assert(clusters[1].type == "target/track-cluster")
        assert(clusters[1].count == 2)
        assert(clusters[2].count == 1)
        -- The target CoM is part of the reconstructed world-space line point.
        assert(near(clusters[1].line.point.x, 10))
        assert(near(clusters[1].line.point.y, 0))
        assert(near(clusters[2].line.point.x, 20))
        assert(near(clusters[2].line.point.y, 10))
        -- relVel is already world-space; rotating it again would incorrectly
        -- turn this line into the z axis.
        assert(near(clusters[2].line.direction.x, 1))
        assert(state.origins[1].position.x == 0)

        local trackProduct = {
          type = "target/track",
          targetId = 7,
          state = state,
        }
        local tracks = {byId = {[7] = trackProduct}}
        local published = dedup:update(nil, tracks, 0)
        assert(published.type == "target/track-cluster")
        assert(published.byId[7].source == trackProduct)
        assert(#published.byId[7].clusters == 2)
        assert(published.rateOfFire ~= nil)
        return true
        """
    )
    assert result is True
    source = SOURCE.read_text(encoding="utf-8")
    assert "target/trajectory-line" in source
    assert "shortestLineDistance" in source
    assert "target/track-cluster" in source
    print("Counterbattery trajectory deduplication: ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
