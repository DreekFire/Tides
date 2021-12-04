-- dependencies: MathUtil
local Targeting = {}

-- Returns the direction in which the weapon should fire to hit a target
-- Assumes both target and projectile travel in a straight line (i.e. no gravity)
-- Also use this for TPG guidance on missiles
function Targeting.firstOrderTargeting(relPos, targetVel, muzzle)
  local targetAngle = Vector3.Angle(-relPos, targetVel)
  local b1, c1, b2, c2 = MathUtil.angleSSA(muzzle, targetVel.magnitude, targetAngle)
  if not b1 then return nil end
  local firingAngle = b2 or b1
  if not firingAngle then return nil end
  return (Quaternion.AngleAxis(firingAngle, Vector3.Cross(relPos, targetVel)) * relPos).normalized
end

-- math from wltrup's answer to math.stackexchange.com question number 1419643
--[[
  Arguments:
    relPos - position of the target relative to own vehicle
    relVel - velocity of the target relative to own vehicle
    accel - acceleration of the target relative to the projectile (absolute target accel - gravity)
    muzzle - muzzle velocity of the projectile
    minRange - the minimum distance of intercept (see return values)
    maxRange - the maximum distance of intercept (see return values)
  Returns:
    If there exists an intercept point between minRange and maxRange, returns the
    direction the weapon should point to hit the target at that intercept point.
    Range is measured in effective range, which is how far the projectile would've
    traveled if gravity didn't exist.
    Otherwise, returns the direction to the target.
--]]
function Targeting.secondOrderTargeting(relPos, relVel, accel, muzzle, minRange, maxRange)
  local t = Targeting.secondOrderTargetingTime(relPos, relVel, accel, muzzle, minRange / muzzle, maxRange / muzzle)
  if t and t > 0 then
    return (relPos / t + relVel - 0.5 * accel * t).normalized
  end
  return nil
end

-- Same as above, but returns the time from now at which intercept will occur
-- wltrup's answer uses acceleration of the projectile while this takes the acceleration of the target
-- which is why the signs are flipped
function Targeting.secondOrderTargetingTime(relPos, relVel, accel, muzzle, minTime, maxTime)
  local a = 0.25 * accel.sqrMagnitude
  local b = Vector3.Dot(relVel, accel)
  local c = relVel.sqrMagnitude - muzzle * muzzle + Vector3.Dot(relPos, accel)
  local d = 2 * Vector3.Dot(relPos, relVel)
  local e = relPos.sqrMagnitude
  local roots = {MathUtil.solveQuartic(a, b, c, d, e)}
  local t = nil
  for i = 1, 4 do
    if roots[i] and roots[i] > minTime and roots[i] < maxTime then
      if not t or t and roots[i] < t then
        t = roots[i]
      end
    end
  end
  return t
end

--[[
  Modified version of IPPN guidance law from
  "An Improvement in Three-Dimensional Pure
  Proportional Navigation Guidance",
  by Hyo-Sang Shin and Ke-Bo Li
  with added acceleration term for augmentation.
  I have no idea if this is better than IPPN alone
  or even theoretically correct because I do not
  know how to perform the relevant mathematical
  analyses.
--]]
function Targeting.AIPPN(gain, relPos, missileVel, targetVel, targetAccel)
  local relVel = targetVel - missileVel
  local closingRate = Vector3.Dot(-relVel, relPos.normalized)
  if closingRate <= 0 then closingRate = 10 end
  local tgo = relPos.magnitude / closingRate

  local losRot = Vector3.Cross(relPos, relVel) / relPos.sqrMagnitude
  local aRot = Vector3.Cross(relPos, targetAccel) / relPos.sqrMagnitude * tgo / 2
  local omega = losRot + aRot
  local losMotion = Vector3.Cross(omega, relPos.normalized)

  local accelDir = Vector3.ProjectOnPlane(losMotion, missileVel).normalized
  local accelMag = gain * missileVel.magnitude * omega.magnitude

  return accelMag * accelDir
end

--[[
  Augmented True Proportional Navigation
  guidance law which FTD missiles use
  when APN guidance is equipped
  (if my understanding is correct)

  True proportional guidance commands
  acceleration perpendicular to line of
  sight between missile and target
  instead of perpendicular to missile
  velocity like pure proportional
  guidance does.
--]]
function Targeting.ATPN(gain, relPos, missileVel, targetVel, targetAccel)
  local relVel = targetVel - missileVel
  local closingRate = -Vector3.Dot(relVel, relPos.normalized)
  if closingRate <= 0 then closingRate = 10 end

  local losRot = Vector3.Cross(relPos, relVel) / relPos.sqrMagnitude
  local losMotion = Vector3.Cross(losRot, relPos.normalized)

  local orthoAccel = Vector3.ProjectOnPlane(targetAccel, relPos)

  return gain * closingRate * losMotion + 0.5 * gain * targetAccel
end