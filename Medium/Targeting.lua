-- dependencies: MathUtil

-- Returns the direction in which the weapon should fire to hit a target
-- Assumes both target and projectile travel in a straight line (i.e. no gravity)
-- targetVel should be relative to the gun for projectiles, absolute for missiles
-- Also use this for TPG guidance on missiles
--[[
  Arguments:
    relPos - position of the target relative to muzzle
    targetVel - absolute velocity of the target
    muzzle - muzzle velocity of the projectile
  Returns:
    intercept - the position of the target at time of intercept
    interceptTime - the time at which intercept occurs
]]
function Targeting.firstOrderTargeting(relPos, targetVel, muzzle)
  if targetVel.sqrMagnitude == 0 then
    return relPos.normalized
  end
  local closest = relPos - Vector3.Project(relPos, targetVel)
  local timeAlongLine = Vector3.Dot(targetVel, relPos - closest) / targetVel.sqrMagnitude
  -- by pythagorean theorem, intercept time t occurs when
  --   (t + timeAlongLine)^2 * targetVel.sqrMagnitude + closest.sqrMagnitude = (muzzle * t)^2
  local a, b = MathUtil.solveQuadratic(targetVel.sqrMagnitude - muzzle * muzzle,
                                                2 * timeAlongLine * targetVel.sqrMagnitude,
                                                closest.sqrMagnitude + timeAlongLine * timeAlongLine * targetVel.sqrMagnitude)
  local interceptTime = nil
  if a and a >= 0 then interceptTime = a end
  if b and b >= 0 and b < a then interceptTime = b end
  if interceptTime then
    return relPos + interceptTime * targetVel, interceptTime
  end
end

--[[
  Arguments:
    relPos - position of the target relative to own vehicle
    relVel - velocity of the target relative to own vehicle
    accel - acceleration of the target relative to the projectile (absolute target accel - gravity)
    muzzle - muzzle velocity of the projectile
    minRange - the minimum distance of intercept (see return values)
    maxRange - the maximum distance of intercept (see return values)
    guess - [optional] initial guess for intercept time
  Returns:
    intercept - the position of the target at time of intercept if between minRange and maxRange
      nil otherwise
      Range is measured in effective range, which is how far the projectile would've
      traveled if gravity didn't exist.
    interceptTime - the time at which intercept occurs
]]
function Targeting.secondOrderTargetingNewton(relPos, relVel, accel, muzzle, minRange, maxRange, guess)
  local diff = 10000
  local lastT = 0
  local iters = 0
  if not guess then guess = 0 end
  local newPos = relPos + guess * relVel + 0.5 * guess * guess * accel
  while math.abs(diff) > 0.001 and iters < 10 do
    local t = newPos.magnitude / muzzle
    diff = t - lastT
    lastT = t
    iters = iters + 1
    newPos = relPos + t * relVel + 0.5 * t * t * accel
  end
  return newPos, lastT, iters
end

Targeting.secondOrderTargeting = Targeting.secondOrderTargetingNewton

-- Same arguments and return values as secondOrderTargetingNewton.
-- This version uses ITP, and is guaranteed to return an answer if one exists, unlike Newton's method.
-- However, it is much more complicated and, for most scenarios found in FtD, probably slower.
function Targeting.secondOrderTargetingITP(relPos, relVel, accel, muzzle, minRange, maxRange, guess)
  if not guess then guess = 0 end
  local a = -0.25 * accel.sqrMagnitude
  local b = -Vector3.Dot(relVel, accel)
  local c = -(relVel.sqrMagnitude - muzzle * muzzle + Vector3.Dot(relPos, accel))
  local d = -2 * Vector3.Dot(relPos, relVel)
  local e = -relPos.sqrMagnitude
  local t

  local g = accel.magnitude
  local v_T = relVel.magnitude
  local d_0 = relPos.magnitude
  local t1a, t1b = MathUtil.solveQuadratic(0.5 * g, v_T + muzzle, -d_0)
  local t1 = math.max(t1a, t1b)
  local t2

  -- todo: use rule of signs to check for positive roots before solving quadratic
  local coeffs = {0.5 * g, v_T - muzzle, d_0}
  if MathUtil.ruleOfSigns(coeffs, 0) == 2 then
    local t2a, t2b = MathUtil.solveQuadratic(coeffs[1], coeffs[2], coeffs[3])
    if t2a then
      t2 = math.min(t2a, t2b)
    end
  end
  if not t2 or t2 < t1 then
    local s0, s1, s2 = MathUtil.solveCubic(4 * a, 3 * b, 2 * c, d)
    if not s2 then
      if s0 > t1 then
        t2 = s0
      end
    else
      local t2a = math.min(s0, s2)
      local t2b = math.max(s0, s2)
      if t2a > t1 then
        t2 = t2a
      elseif t2b > t1 then
        t2 = t2b
      end
    end
    if not t2 then
      return
    end
  end

  local function poly(x)
    -- Horner's method of evaluating polynomials is slightly faster
    return e + x * (d + x * (c + x * (b + x * a)))
  end
  if guess > t1 and t2 > guess then
    if poly(t1) * poly(t) < 0 then
      t2 = guess
    else
      t1 = guess
    end
  end
  t = MathUtil.ITP(poly, t1, t2, 1e-4, 25)

  if not t then return end
  if t >= t1 and t <= t2 then
    local intercept = relPos + relVel * t + 0.5 * accel * t * t
    if intercept.magnitude >= minRange and intercept.magnitude <= maxRange then
      return intercept, t
    end
  end
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
]]
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
