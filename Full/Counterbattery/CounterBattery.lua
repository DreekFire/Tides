-- Settings
-- use pCall to avoid crash, retry if failed
local AllowErrorRecovery = false

-- seconds to record target movements for
local targetTrackTime = 15
-- number of locations to track per enemy (todo: support tracking multiple projectiles)
  -- currently not useful as a single shell will generate duplicate origins. To be fixed
local numOrigins = 1
-- time between switching targets
local originSwitchTime = 0.25
-- maximum time to remember origin points
local maxStaleness = 3
-- todo: move range and altitude limits to weapon definitions
-- ranges to engage
local minRange = 50
local maxRange = 2000
-- altitudes to engage (estimated altitude of firing piece, not target origin)
  -- checked when adding an origin and when firing
local minAlt = 0
local maxAlt = math.huge
-- parameters to find weapons (Shift+N to name turret blocks)
  -- different turrets should have different names even if they have the same weapon
  -- if all weapons on a turret have the same muzzle velocity then just name the turret
  -- if multiple weapons on same turret with different muzzle velocity then name the turret,
    -- leave primary weapon unnamed and give secondary weapons unique names
    -- turret name should go before names of secondary weapons
  -- checkACB lists ACBs set to detect object presence and trigger custom axis
    -- may also have second ACB with inverted settings that trigger negative custom axis
    -- will be checked before firing to make sure blocks actually exist at target distance
    -- this check is not performed when firing at aimpoint due to idle mode "fire" or "timer"
    -- validRequire is required number of ACBs in range and pointing in the right direction
    -- checkRequire is proportion of valid ACBs that are have detected an object
local weaponDef = {
  {
    name = "laser",
    checkACBs = {
      { axis = "check", minRange = 200, maxRange = 2400, turretName = "checkTurret", offset = Vector3(0, 1, 0), },
    },
    checkRequire = 1,
    validRequire = 0,
  }
}
-- indexes of mainframes to be used to track enemy rotations
  -- 3 is the bare minimum, but will fail if any of them target a subconstruct
  -- so more is preferred
  -- it will also fail if too many mainframes switch their aimpoint simultaneously
  -- this is unavoidable in the case of blocks being destroyed, but can be avoided
  -- in the case of the timer switching blocks. Todo: manage switch times to avoid
  -- synchronized aimpoint switching
local aimPointTrackers = {
  { idx = 1 },
  { idx = 2 },
  { idx = 3 },
  { idx = 4 },
  { idx = 5 },
}
-- which mainframe to use, will affect target prioritization
  -- currently only fires at the highest priority target
  -- can be the same as one of the aimPointTrackers
local mainframeIdx = 0
-- what to do when no firing origin detected
  -- timer: wait for waitTime seconds, then fire at current target if no origin found
  -- fire: aim and fire at current target
  -- enemy: aim at current target
  -- none: return to idle position
  -- last: continue aiming at last absolute bearing
local idleAim = "timer"
  -- time to wait for a new origin before firing at the aimpoint
    -- only used in idle aim mode "timer"
local waitTime = 10
-- offset the aimpoint, i.e. for hitting the necks, tetris, or turret bases instead of the barrels
  -- is a function with the following properties:
  --[[
    Arguments:
      fp - the original aimpoint
      target - the target position
      isCounter - whether we are firing at an origin detected by the counterbattery script or just the AI aimpoint
    Returns:
      fpOffset - the offset of the adjusted aimpoint relative to the original aimpoint
  ]]
  -- to adjust depending on estimated turret location
-- todo: return list of locations to attempt, using the checkACBs to check each one
local function aimOffset(fp, target, isCounter)
  return Vector3.zero
end
-- when tracking rotation, reject any rotation estimates above this rate (in deg/s)
local maxTurnRateTracking = 720
-- whether or not to attempt to snipe missile launchers
  -- since missiles can turn, we can't trace back their trajectory
  -- however, if we detect a missile the instant it is launched (most feasible for huge missiles
    -- and large missiles with low ramp time, no ignition delay, and high thrust),
  -- we can approximate the missile as traveling in a straight line
local missileCounter = true
-- degrees of inaccuracy allowed when firing
-- weapon will start firing within this angle
-- but will always try to obtain perfect accuracy
local AIM_TOL = 0.1
-- physics ticks per second (Lua runs in sync with game physics)
local TICKS_PER_S = 40

-- todo: Integrate with enemy identififcation script to get weapon distances from origin,
    -- and trace to that distance from origin instead of closest
-- todo: Account for target acceleration

-- one way to store previous values is in local variables outside of Update like this
  -- another way is to use global variables, which has the benefit of being able to be
  -- located near where they are used, but are much slower to access (requires a table lookup)
local lastProjectilePos
local enemies
local currentLine
local lastFrameTime
local inited
local prevTime
local lastOrigin
local lastOriginSwitchTime = 0
local currentTargetId
local originPopTime = 0
local lastAim
local nextRecordTime
local continueLine = false
local turrets = {}

local BlockUtil = {}
local Combat = {}
local StringUtil = {}
local Accumulator = {}
local Differ = {}
local Graph = {}
local Heapq = {}
local LinkedList = {}
local MathUtil = {}
local RingBuffer = {}
local VectorN = {}
local Control = {}
local Nav = {}
local Scheduling = {}
local Targeting = {}

function Init(I)
  for idx, weapon in ipairs(weaponDef) do
    turrets[idx] = BlockUtil.getWeaponsByName(I, weapon.name)
    for acbIdx, acb in ipairs(weapon.checkACBs) do
      acb.turretSub = BlockUtil.getSubConstructsByName(I, acb.turretName, 1)[1]
      acb.turret = BlockUtil.getWeaponsByName(I, acb.turretName, 1)[1]
    end
  end

  nextRecordTime = I:GetTimeSinceSpawn()
  enemies = {}
  math.randomseed(I:GetTime())
  math.random()
  math.random()

  originPopTime = I:GetTimeSinceSpawn()
  inited = true
end

function Update(I)
  --I:ClearLogs()
  --I:Log(string.format("Game Time: %.2f", I:GetTimeSinceSpawn()))
  if AllowErrorRecovery then
    ProtectedUpdate(I)
  else
    CoreUpdate(I)
  end
end

function ProtectedUpdate(I)
  local updateRan, err = pcall(CoreUpdate, I)
  if not updateRan then
    I:Log("Error in Update")
    I:Log(err)
    return false --This means we had an error, so just move on in the LUA.
  else
    I:Log("Ran update")
  end
end

function CoreUpdate(I)
  if not inited then Init(I) end
  local target = I:GetTargetInfo(mainframeIdx, 0)
  if not target or not target.Valid then
    return
  end
  local t = I:GetTimeSinceSpawn()

  if t >= nextRecordTime - 0.5 / TICKS_PER_S then
    UpdateEnemyData(I)
    nextRecordTime = nextRecordTime + 1 / TICKS_PER_S
  end
  currentTargetId = target.Id

  local frameTime = lastFrameTime and t - lastFrameTime or 0
  lastFrameTime = t
  -- calculate projectile location
  local alt = 800 * I:GetPropulsionRequest(9) -- A axis, set in projectile avoidance routine
  local relBear = 180 * I:GetPropulsionRequest(10) -- B axis
  local dist = 800 * I:GetPropulsionRequest(12) -- D axis

  local projectile = dist * Vector3.forward
  projectile = Quaternion.AngleAxis(I:GetConstructYaw() + relBear, Vector3.up) * projectile
  projectile = projectile + I:GetConstructCenterOfMass()
  projectile.y = alt

  if dist == 0 then
    currentLine = nil
    lastProjectilePos = nil
  end

  if missileCounter then
    for widx = 0, I:GetNumberOfWarnings(0) - 1 do
      local warn = I:GetMissileWarning(mainframeIdx, widx)
      if warn.Valid and warn.TimeSinceLaunch < 0.1 then
        local launcherPos = warn.Position - warn.TimeSinceLaunch * warn.Velocity
        local targetPosAtLaunch = target.Position - warn.TimeSinceLaunch * target.Velocity
        -- todo: set threshold by target size
        if (launcherPos - targetPosAtLaunch).sqrMagnitude < 150 * 150 then
          local closestAlt = launcherPos.y
          if closestAlt > minAlt and closestAlt < maxAlt then
            local enemy = enemies[target.Id]
            local eRot = enemy.rotation[enemy.rotation.size] or Quaternion.Identity
            local lPos = Quaternion.Inverse(eRot) * (launcherPos - targetPosAtLaunch)
            RingBuffer.push(enemy.origins, lPos)
            RingBuffer.push(enemy.originTimes, t)
          end
        end
      end
    end
  end

  -- compare to past projectile locations
  -- to make sure we're tracking the same projectile

  -- see if it matches current line
  -- todo: store multiple lines and find match

  -- 2/3rds the estimated drop in two frames due to gravity
  -- inconsistent with theoretical formula due to discrete integration
  -- powered missiles have no gravity so their expected error
  -- is the negative of the drop due to gravity
  local eps = 10 * frameTime * frameTime
  -- todo: maybe limit how often the line tracing runs to save processing power
  if lastProjectilePos and currentLine and CheckAndUpdateLine(I, currentLine, projectile, frameTime, eps) then
    if not continueLine then
      local enemy = enemies[currentTargetId]
      local origin = GetOrigin(I, projectile, enemy)
      if origin then
        RingBuffer.push(enemy.origins, origin)
        RingBuffer.push(enemy.originTimes, t)
      end
    end
    continueLine = true
  else
    continueLine = false
    if lastProjectilePos and dist > 0 then
      local start = lastProjectilePos
      currentLine = {
        start = start,
        tStart = t - 1 / TICKS_PER_S,
        ed = projectile
      }
      currentLine.dv = -frameTime * I:GetGravityForAltitude(start.y).y
      currentLine.dy = frameTime * currentLine.dv
      currentLine.ds = projectile + currentLine.dy * Vector3.up - start
      currentLine.dt = frameTime
      currentLine.ev = projectile - start
      prevTime = nil
    end
  end

  lastProjectilePos = projectile

  -- fire weapon at origins
  local enemy = enemies[currentTargetId]
  local eRot = enemy.rotation[enemy.rotation.size] or Quaternion.identity
  local fp
  if lastOriginSwitchTime + originSwitchTime > t then
    fp = lastOrigin and eRot * lastOrigin
  end
  if not fp then
    while enemy.origins.size > 0 and t - enemy.originTimes[1] > maxStaleness do
      RingBuffer.pop(enemy.origins)
      RingBuffer.pop(enemy.originTimes)
      originPopTime = t
    end
    if enemy.origins.size > 0 then
      lastOrigin = enemy.origins[math.random(1, enemy.origins.size)]
      fp = eRot * lastOrigin
      if fp.y + target.Position.y < minAlt or fp.y + target.Position.y > maxAlt then
        -- do a linear search to find valid origin
        local found = false
        for i = 1, enemy.origins.size do
          lastOrigin = enemy.origins[i]
          fp = eRot * lastOrigin
          if fp.y + target.Position.y > minAlt and fp.y + target.Position.y < maxAlt then
            found = true
            break
          end
        end
        if not found then
          fp = nil
          lastOrigin = nil
        end
      end
    end
    lastOriginSwitchTime = t
  end
  for i, turret in ipairs(turrets) do
    local wfp
    local ready = false
    if fp then
      -- check ACBs for object presence
      local success = 0
      local valid = 0
      wfp = fp + aimOffset(fp, target.Position, true)
      for acbIdx, acb in ipairs(weaponDef[i].checkACBs) do
        local bInfo = I:GetSubConstructInfo(acb.turretSub)
        local r = wfp + target.Position - (bInfo.Position + bInfo.Rotation * acb.offset)
        BlockUtil.aimWeapon(I, acb.turret, r, 0)
        if r.magnitude > acb.minRange and r.magnitude < acb.maxRange
            and I:IsAlive(acb.turretSub)
            and Vector3.Angle(r, bInfo.Forwards) < AIM_TOL then
          valid = valid + 1
          if I:GetCustomAxis(acb.axis) > 0 then
            success = success + 1
          end
        end
      end
      ready = valid >= weaponDef[i].validRequire and success >= weaponDef[i].checkRequire * valid
    end
    wfp = ready and wfp or (target.AimPointPosition - target.Position + aimOffset(fp, target.Position, false))
    ready = ready or (idleAim == "fire" or (idleAim == "timer" and t - originPopTime > waitTime))
    for j, weapon in ipairs(turret) do
      local aim
      local fire = false
      local wInfo = BlockUtil.getWeaponInfo(I, weapon)
      if ready then
        if wInfo.Speed >= 1e5 then
          local range = (wfp + target.Position - I:GetConstructPosition()).magnitude
          if range > minRange and range < maxRange then
            aim = wfp + target.Position - wInfo.GlobalFirePoint
            fire = true
          end
        else
          local g = 0.5 * (I:GetGravityForAltitude(I:GetConstructPosition().y) + I:GetGravityForAltitude(target.Position.y))
          aim = Targeting.secondOrderTargeting(wfp + target.Position - wInfo.GlobalFirePoint,
                      target.Velocity - I:GetVelocityVector(),
                      -g,
                      wInfo.Speed, minRange, maxRange)
          if aim then fire = true end
        end
      end
      if not aim then
        if idleAim == "enemy" or idleAim == "fire" or idleAim == "timer" then
          aim = target.Position - wInfo.GlobalFirePoint
        elseif idleAim == "last" then
          aim = lastAim
        end
      end
      if aim then
        lastAim = aim
        if Combat.CheckConstraints(I, aim, weapon.wpnIdx, weapon.subIdx) then
          BlockUtil.aimWeapon(I, weapon, aim, 0)
          if fire and Vector3.Angle(wInfo.CurrentDirection, aim) < AIM_TOL then
            BlockUtil.fireWeapon(I, weapon, 0)
          end
        end
      end
    end
  end
end

function UpdateEnemyData(I)
  for id, en in pairs(enemies) do
    en.valid = false
  end
  for tarIdx = 0, I:GetNumberOfTargets(mainframeIdx) - 1 do
    local target = I:GetTargetInfo(mainframeIdx, tarIdx)
    if target.Valid then
      if not enemies[target.Id] then
        local rbsize = targetTrackTime * TICKS_PER_S
        enemies[target.Id] = {
                              pos = RingBuffer.RingBuffer(rbsize),
                              vel = RingBuffer.RingBuffer(rbsize),
                              rotation = RingBuffer.RingBuffer(rbsize),
                              origins = RingBuffer.RingBuffer(numOrigins),
                              originTimes = RingBuffer.RingBuffer(numOrigins),
                              valid = true,
                              oldAimpoints = {},
                              aimpoints = {},
                            }
      end
      local e = enemies[target.Id]
      e.valid = true
      RingBuffer.push(e.pos, target.Position)
      RingBuffer.push(e.vel, target.Velocity)
    end
  end

  -- have to do this because there is no get target by id function
  for id, en in pairs(enemies) do
    if not en.valid then
      enemies[id] = nil
      if id == currentTargetId then
        lastOrigin = nil
        lastOriginSwitchTime = I:GetTimeSinceSpawn()
      end
    end
  end

  for idx, tracker in ipairs(aimPointTrackers) do
    for tarIdx = 0, I:GetNumberOfTargets(tracker.idx) - 1 do
      local target = I:GetTargetInfo(tracker.idx, tarIdx)
      if target.Valid then
        local e = enemies[target.Id]
        e.oldAimpoints[idx] = e.aimpoints[idx] or target.AimPointPosition
        e.aimpoints[idx] = target.AimPointPosition
      end
    end
  end

  for id, en in pairs(enemies) do
    local dRot = GetRotation(en.oldAimpoints, en.aimpoints) or Quaternion.identity
    RingBuffer.push(en.rotation, dRot * (en.rotation[en.rotation.size] or Quaternion.identity))
  end
end

function GetOrigin(I, projectile, enemy)
  if prevTime and RingBuffer.isFull(enemy.pos) then
    prevTime = prevTime - 1
  end
  local vel = currentLine.ds / currentLine.dt
  local relVel = vel - enemy.vel[enemy.vel.size]
  local relPos = projectile - enemy.pos[enemy.pos.size]
  local time2d = math.sqrt((relPos.x ^ 2 + relPos.z ^ 2) / (relVel.x ^ 2 + relVel.z ^2))
  local estimate2d = relPos - (relVel * time2d) + 0.5 * I:GetGravityForAltitude(currentLine.start.y) * time2d * time2d
  -- todo: adjust threshold based on enemy size, also penalize vertical error greater than horizontal
  if estimate2d.sqrMagnitude < 150 * 150 then
    local p, t = CheckIfCram(projectile, vel, enemy.pos[enemy.pos.size], 150)
    if p then
      I:Log("Projectile determined to be CRAM shell, fired "..t.." seconds ago")
      local pIdx = enemy.pos.size + t * TICKS_PER_S
      if pIdx < 1 then
        I:Log("CRAM direct guess has no target data")
        return
      end
      local enemyPosAtTime = enemy.pos[pIdx]
      local enemyRotAtTime = enemy.rotation[pIdx]
      if enemyPosAtTime then
        return Quaternion.Inverse(enemyRotAtTime) * (p - enemyPosAtTime)
      end
      return
    end
    local closest, closestTimeIdx = RunTrace(I, currentLine, enemy, prevTime)
    if not closest then
      I:Log("no solution found")
      prevTime = nil
      return
    end
    prevTime = closestTimeIdx
    if closest.sqrMagnitude < 100 * 100 then
      local closestAlt = closest.y + enemy.pos[closestTimeIdx].y
      if closestAlt > minAlt and closestAlt < maxAlt then
        return Quaternion.Inverse(enemy.rotation[closestTimeIdx]) * closest
      end
    end
  end
end

function GetRotation(oldPts, newPts, iterLim)
  local nTrack = #newPts
  iterLim = iterLim or 10

  local indices = {}
  for i=1, nTrack do
    indices[i] = i
  end
  -- iterate through triples of aimpoints, check if the legs are the same length
  for trip=1, iterLim do
    -- not the most efficient as we shuffle the entire list even though we only need the first 3. Implement range shuffle later
    MathUtil.shuffle(indices, true)
    local oldAimpoints = {}
    local aimpoints = {}
    for i=1, 3 do
      oldAimpoints[i] = oldPts[indices[i]]
      aimpoints[i] = newPts[indices[i]]
    end

    local legs, oldLegs = {}, {}
    local valid = true
    for i=1, 3 do
      legs[i] = aimpoints[i % 3 + 1] - aimpoints[(i - 1) % 3 + 1]
      oldLegs[i] = oldAimpoints[i % 3 + 1] - oldAimpoints[(i - 1) % 3 + 1]
      -- check if aimpoints have changed
      if math.abs(legs[i].sqrMagnitude - oldLegs[i].sqrMagnitude) > 0.1 then
        valid = false
        break
      end
    end
    if valid then
      local oldRot = Quaternion.LookRotation(oldLegs[1], oldLegs[2])
      local rot = Quaternion.LookRotation(legs[1], legs[2])
      local deltaRot = rot * Quaternion.Inverse(oldRot)
      if math.acos(math.max(math.min(deltaRot.w, 1), -1)) * 360 / math.pi * TICKS_PER_S < maxTurnRateTracking then
        -- check for consensus
        local votes = 0
        for idx, ap in ipairs(newPts) do
          local diff = ap - newPts[indices[1]]
          local oldDiff = oldPts[idx] - oldPts[indices[1]]
          if (diff - deltaRot * oldDiff).sqrMagnitude < 0.1 then
            votes = votes + 1
            if 2 * votes - 3 >= nTrack then
              return deltaRot
            end
          end
        end
      end
    end
  end
  return nil
end

function CheckIfCram(projPos, projVel, enemyPos, tolerance)
  -- y component of velocity changes, but horizontal does not
  -- if horizontal alone exceeds max CRAM speed, cannot be CRAM
  -- no matter what y component becomes
  local hSqrMag = projVel.x * projVel.x + projVel.z * projVel.z
  -- CRAMs usually aren't fired underwater, and if they are, this method breaks anyways 
  local speedAtSeaLevel = math.sqrt(2 * projPos.y * 9.81 + projVel.sqrMagnitude)
  local maybeD = hSqrMag <= 90000 and speedAtSeaLevel >= 300
  local maybeS = hSqrMag <= 57600 and speedAtSeaLevel >= 240
  local p, t1, t2
  if maybeD then
    t1, t2 = GetCramLaunchTime(projVel, 300)
    if t2 <= 0 then
      p = CheckPosition(projPos, projVel, enemyPos, t2, tolerance)
      if p then return p, t2 end
    end
    if t1 <= 0 then
      p = CheckPosition(projPos, projVel, enemyPos, t1, tolerance)
      if p then return p, t1 end
    end
  end
  if maybeS then
    t1, t2 = GetCramLaunchTime(projVel, 240)
    if t2 <= 0 then
      p = CheckPosition(projPos, projVel, enemyPos, t2, tolerance)
      if p then return p, t2 end
    end
    if t1 <= 0 then
      p = CheckPosition(projPos, projVel, enemyPos, t1, tolerance)
      if p then return p, t1 end
    end
  end
end

function CheckPosition(projPos, projVel, enemyPos, t, tolerance)
  local p = projPos + projVel * t - 0.5 * Vector3(0, 9.81, 0) * t * t
  if (p - enemyPos).sqrMagnitude < tolerance * tolerance then
    return p
  end
end

-- provides the launch time in seconds before present
-- assuming initial velocity of either 240 or 300m/s
function GetCramLaunchTime(vel, muzzle)
  local hSqrMag = vel.x * vel.x + vel.z * vel.z

  local yvel = math.sqrt(muzzle * muzzle - hSqrMag)
  return (vel.y - yvel) / 9.81, (vel.y + yvel) / 9.81
end

function CheckAndUpdateLine(I, line, projectile, frameTime, tolerance)
  local expected = line.ed + line.ev + frameTime * frameTime * I:GetGravityForAltitude(line.ed.y)
  local dv = line.dv - frameTime * I:GetGravityForAltitude(line.ed.y).y
  local dy = line.dy + frameTime * line.dv
  if (expected - projectile).sqrMagnitude <= tolerance * tolerance then
    line.ds = projectile + line.dy * Vector3.up - line.start
    line.dv = dv
    line.dy = dy
    line.dt = I:GetTimeSinceSpawn() - line.tStart
    line.ev = projectile - line.ed
    line.ed = projectile
    return true
  end
  return false
end

function RunTrace(I, line, enemy, timeGuess)
  local t = I:GetTimeSinceSpawn()
  local totalIter = 0
  local targetPos = enemy.pos[enemy.pos.size]
  local targetVel = enemy.vel[enemy.vel.size]
  if timeGuess then
    if enemy.pos[timeGuess] then
      targetPos = enemy.pos[timeGuess]
      targetVel = enemy.vel[timeGuess]
    else
      I:Log("initial guess has no target data")
    end
  end
  local tIdxClosest
  for i = 1, timeGuess and 2 or 3 do
    -- find the point of closest approach based on current target position and velocity
    -- x(t) = x_i + v_x t
    -- z(t) = z_i + v_z t
    -- y(t) = y_i + v_y t + 0.5gt^2
    -- squared distance = x^2 + y^2 + z^2

    -- d/dt sqrDistance = 
    -- 2 x_i v_x + 2 v_x^2 t +
    -- 2 z_i v_z + 2 v_z^2 t +
    -- 2 y_i v_y + 2 v_y^2 t + 2 y_i g t + 3 v_y g t^2 + 0.25 g^2 t^3
    -- this is a cubic polynomial in terms of t which we can find the roots of
    local ti = (line.tStart + line.dt) - (timeGuess and t - (enemy.pos.size - timeGuess) / TICKS_PER_S or t)
    local di = line.ed - targetPos + ti * targetVel
    local projRelVel = line.ds / line.dt - line.dv * Vector3.up - targetVel
    -- accounting exactly for gravity changes over altitude is difficult, just approximate and hope the enemy isn't using mortars
    local g = I:GetGravityForAltitude(line.ed.y).y
    local a, b, c = MathUtil.solveCubic(0.125 * g * g, 1.5 * projRelVel.y * g, projRelVel.sqrMagnitude + di.y * g, Vector3.Dot(di, projRelVel))
    -- critical point is a minimum when derivative changes from negative to positive
    -- since leading term is always positive (0.125g^2 = 12.2), if there are three roots, the first and third are minima
    -- if there is one root, it is a minimum
    -- minRoot is in seconds relative to line.tStart + line.dt
    local minRoot
    if a and b and c then
      local firstRoot = math.min(a, b, c)
      local lastRoot = math.max(a, b, c)
      local firstSqrDist = SqrDistance(I, line, targetPos, targetVel, firstRoot)
      local lastSqrDist = SqrDistance(I, line, targetPos, targetVel, lastRoot)
      if firstSqrDist < lastSqrDist then
        minRoot = firstRoot
      else
        minRoot = lastRoot
      end
    else
      minRoot = a
    end
    -- get target position and velocity at estimated time of closest approach
    local approachTime = minRoot + line.tStart + line.dt - t
    tIdxClosest = math.floor(approachTime * TICKS_PER_S + 0.5) + enemy.pos.size
    -- InterpolatedSearch(I, times, 1, times.size, minRoot + line.tStart + line.dt, true)
    if not tIdxClosest then return nil end
    targetPos = enemy.pos[tIdxClosest]
    targetVel = enemy.vel[tIdxClosest]
    if not targetPos then
      I:Log("iterated guess has no target data")
      return nil
    end
  end

  -- linear search to find best point
  -- todo: account for target velocity
  function CalcSqrDist(tIdx)
    local dt = t - (enemy.pos.size - tIdx) / TICKS_PER_S - line.tStart
    return (line.start - enemy.pos[tIdx] + dt * (line.ds / line.dt) + 0.5 * I:GetGravityForAltitude(line.start.y) * dt * dt).sqrMagnitude
  end
  local currentSqrDist = CalcSqrDist(tIdxClosest)
  if tIdxClosest < enemy.pos.size then
    local aftSqrDist = CalcSqrDist(tIdxClosest + 1)
    while aftSqrDist < currentSqrDist and tIdxClosest < enemy.pos.size do
      currentSqrDist = aftSqrDist
      tIdxClosest = tIdxClosest + 1
      if tIdxClosest < enemy.pos.size then
        aftSqrDist = CalcSqrDist(tIdxClosest + 1)
      end
      totalIter = totalIter + 1
      if totalIter >= 100 then
        I:Log("max iterations exceeded on upwards search")
        break
      end
    end
  end
  --I:Log(totalIter.." iterations after upwards search")
  if tIdxClosest > 1 then
    local befSqrDist = CalcSqrDist(tIdxClosest - 1)
    while befSqrDist < currentSqrDist and tIdxClosest > 1 do
      currentSqrDist = befSqrDist
      tIdxClosest = tIdxClosest - 1
      if tIdxClosest > 1 then
        befSqrDist = CalcSqrDist(tIdxClosest -1)
      end
      totalIter = totalIter + 1
      if totalIter >= 100 then
        I:Log("max iterations exceeded on downwards search")
        break
      end
    end
  end
  --I:Log(totalIter.." iterations after downwards search")
  -- time of closest approach relative to line.tStart
  local dt = t - (enemy.pos.size - tIdxClosest) / TICKS_PER_S - line.tStart
  return line.start - enemy.pos[tIdxClosest] + dt * (line.ds / line.dt) + 0.5 * I:GetGravityForAltitude(line.start.y) * dt * dt, tIdxClosest
end

function SqrDistance(I, line, targetAbsPos, targetAbsVel, t)
  local di = line.ed - targetAbsPos
  local viRel = line.ds / line.dt - line.dv * Vector3.up - targetAbsVel
  local diff = di + t * viRel + 0.5 * I:GetGravityForAltitude(line.ed.y) * t * t
  return diff.sqrMagnitude
end

function InterpolatedSearch(I, list, left, right, target, findClosest, iterLim)
  iterLim = iterLim or 50
  local a, b, split
  local totalIter = 0
  while right > left do
    a = list[left]
    if a == target then return left end
    if a > target then return findClosest and left or nil end
    b = list[right]
    if b == target then return right end
    if b < target then return findClosest and right or nil end
    split = math.floor((target - a) / (b - a) * (right - left) + left)
    split = math.min(math.max(split, left + 1), right - 1)
    if list[split] == target then return split end
    if target < list[split] then
      if findClosest and math.abs(list[split - 1] - target) > math.abs(list[split] - target) then
        return split
      end
      right = split - 1
    else
      if findClosest and math.abs(list[split + 1] - target) > math.abs(list[split] - target) then
        return split
      end
      left = split + 1
    end
    totalIter = totalIter + 1
    if totalIter > iterLim then
      I:Log("max iterations exceeded on InterpolatedSearch")
      break
    end
  end
  return findClosest and left or nil
end

-- minified version of Tides library (not meant to be human-readable, see Tides.lua or individual class files for human-readable source)
function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function Heapq.Heapq(i,k)local l={}l.data=i;l.comp=k or function(m,n)return m<n end;local o=#l.data;l.size=o;for p=math.floor(o/2),1,-1 do Heapq.siftDown(l,p)end;return l end;function Heapq.siftDown(l,q)local r=false;local s=q;local o=#l.data;while not r do r=true;local t=2*s;local u=2*s+1;local v=s;if t<=o and l.comp(l.data[t],l.data[v])then v=t;r=false end;if u<=o and l.comp(l.data[u],l.data[v])then v=u;r=false end;if not r then local w=l.data[v]l.data[v]=l.data[s]l.data[s]=w;s=v end end end;function Heapq.siftUp(l,q)local r=false;local s=q;while not r do r=true;local x=math.floor(s/2)if l.comp(l.data[s],l.data[x])then local w=l.data[x]l.data[x]=l.data[s]l.data[s]=w;s=x;r=false end end end;function Heapq.insert(l,y)l.data[l.size+1]=y;l.size=l.size+1;Heapq.siftUp(l,l.size)end;function Heapq.pop(l)local z=l.data[1]l.data[1]=l.data[l.size]l.data[l.size]=nil;l.size=l.size-1;Heapq.siftDown(l,1)return z end;function Heapq.peek(l)return l.data[1]end;function Heapq.size(l)return l.size end;function LinkedList.LinkedList()local A={}A.value=nil;A.next=A;A.prev=A;return A end;function LinkedList.pushFront(B,d)local A={}A.value=d;LinkedList.connect(A,B.next)LinkedList.connect(B,A)end;function LinkedList.pushBack(B,d)local A={}A.value=d;LinkedList.connect(B.prev,A)LinkedList.connect(A,B)end;function LinkedList.popFront(B)local C=B.next.value;LinkedList.connect(B,B.next.next)return C end;function LinkedList.popBack(B)local C=B.prev.value;LinkedList.connect(B.prev.prev,B)return C end;function LinkedList.peekFront(B)return B.next.val end;function LinkedList.peekBack(B)return B.prev.val end;function LinkedList.connect(D,E)D.next=E;E.prev=D end;function LinkedList.toArray(B)local F=1;local G={}local A=B.next;while A~=B do G[F]=A.value;A=A.next end;return G end;function MathUtil.angleOnPlane(H,I,J)local K=Vector3.ProjectOnPlane(H,J)local g=Vector3.ProjectOnPlane(I,J)return Vector3.SignedAngle(K,g,J)end;function MathUtil.min(L,M)local N=nil;M=M or function(m,n)return m<n end;for O in L do if not N or M(O,N)then N=O end end;return N end;function MathUtil.max(L,M)local P=nil;M=M or function(m,n)return m<n end;for O in L do if not P or M(P,O)then P=O end end;return P end;function MathUtil.range(m,n,Q)local R,S=m,n;local T;if not m then return end;if not n then R=0;S=m;T=R<S and 1 or-1 elseif Q then T=Q end;return function(U,V)local W=V+T;if W==S then return nil end;return W end,nil,R-T end;function MathUtil.shuffle(B,X)local s=X and B or{}if not X then for F=1,#B do s[F]=B[F]end end;for F=#B,2,-1 do local Y=math.random(F)s[F],s[Y]=s[Y],s[F]end;return s end;function MathUtil.combine(m,n,Z)if#m==#n then local z={}for _,a0 in pairs(m)do z[_]=Z(_,a0,n[_])end;return z end end;function MathUtil.distribution()return{n=0}end;function MathUtil.updateDistribution(a1,a2)a1.n=a1.n+1;if a1.n==1 then a1.mean=a2;a1.covariance={}local h=#a2;for F=1,h do local a3={}for Y=1,h do a3[Y]=0 end;a1.covariance[F]=a3 end else a1.mean=a1.mean+1/(a1.n+1)*a2 end end;function MathUtil.mean(a1)return a1.mean end;function MathUtil.covariance(a1)return a1.cov end;function MathUtil.normal()local a4,a5=MathUtil.boxMuller()return a4 end;function MathUtil.normalPDF(a4)return math.exp(-0.5*a4*a4)/math.sqrt(2*math.pi)end;function MathUtil.normalCDF(a4)local a6=0.2316419;local a7=0.319381530;local a8=-0.356563782;local a9=1.781477937;local aa=-1.821255978;local ab=1.330274429;local g=1/(1+a6*a4)return 1-MathUtil.normalPDF(a4)*(a7*g+a8*g^2+a9*g^3+aa*g^4+ab*g^5)end;function MathUtil.inverseNorm(ac)local ad=ac>=0.5 and ac or-ac;local a4=5.55556*(1-((1-ad)/ad)^0.1186)if ac<0.5 then a4=-a4 end;return a4 end;function MathUtil.boxMuller()local ae=math.random()local af=math.random()af=math.random()af=math.random()local ag=math.sqrt(-2*math.log(ae))local ah=2*math.pi*af;return ag*math.cos(ah),ag*math.sin(ah)end;function MathUtil.angleSSS(m,n,Q)if m+n<Q or m+Q<n or n+Q<m then return nil end;local ai=math.deg(math.acos((n*n+Q*Q-m*m)/(2*n*Q)))local aj,ak=MathUtil.angleSAS(n,ai,Q)return ai,aj,ak end;function MathUtil.sideSAS(m,ak,n)local al=m*m+n*n-2*m*n*math.cos(math.rad(ak))return math.sqrt(al)end;function MathUtil.angleSAS(m,ak,n)local Q=MathUtil.sideSAS(m,ak,n)if MathUtil.isZero(Q)then return nil end;local ai,aj;if m<n then ai=MathUtil.angleLoSin(Q,m,ak)aj=180-ai-ak else aj=MathUtil.angleLoSin(Q,n,ak)ai=180-aj-ak end;return ai,aj end;function MathUtil.sideSSA(m,n,ai)local am=n*n-m*m;local an=-2*n*math.cos(math.rad(ai))local ao,ap=MathUtil.solveQuadratic(1,an,am)if not ap then return ao,ap end;if ao<ap then return ao,ap end;return ap,ao end;function MathUtil.angleSSA(m,n,ai)local ao,ap=MathUtil.sideSSA(m,n,ai)if not ao then return nil end;local aq,ar=MathUtil.angleSAS(n,ai,ao)if not ap then return aq,ar end;local as,at=MathUtil.angleSAS(n,ai,ap)return aq,ar,as,at end;function MathUtil.sideAAS(ai,aj,m)local ak=180-ai-aj;local n=MathUtil.sideLoSin(ai,aj,m)local Q=MathUtil.sideLoSin(ai,ak,m)return n,Q end;function MathUtil.sideLoSin(m,ai,aj)return m*math.sin(math.rad(aj))/math.sin(math.rad(ai))end;function MathUtil.angleLoSin(m,n,ai)return math.deg(math.asin(n*math.sin(math.rad(ai))/m))end;function MathUtil.clampCone(au,av,aw)local ax=math.min(aw,Vector3.Angle(au,av))local ay=Vector3.Cross(au,av)return Quaternion.AngleAxis(ax,ay)*au end;function MathUtil.newton(az,aA,aB,aC,aD,aE)aC=aC or 1e-5;aE=aE or 10*aC;aD=aD or 25;aA=aA or function(aF)return(az(aF+aE)-az(aF))/aE end;aB=aB or 0;local aG=aC+1;local aH=0;while aG>aC and aH<aD do local aI=az(aB)local aJ=aA(aB)if not aI or not aJ then return nil end;aG=-aI/aJ;aB=aB+aG;aH=aH+1 end;if aH<aD then return aB,false end;return aB,true end;function MathUtil.ITP(az,m,n,aC,aD)if az(m)*az(n)>0 then return nil end;local aK;if az(m)>az(n)then aK=function(aF)return-az(aF)end else aK=az end;aC=aC or 1e-5;aD=aD or 25;local aL=0.2/(n-m)local aM=2;local aN=1;local aO=math.ceil(math.log((n-m)/(2*aC),2))local aP=aO+aN;local Y=0;while n-m>2*aC and Y<aD do local aQ=(m+n)/2;local aR=(n*az(m)-m*az(n))/(az(m)-az(n))local aS=aQ-aR;local aT=aL*math.abs(n-m)^aM;local aU=aS>0 and 1 or(aS==0 and 0 or-1)local aV=aT<=math.abs(aS)and aR+aU*aT or aQ;local aW=aC*2^(aP-Y)-(n-m)/2;local aX=math.abs(aV-aQ)<=aW and aV or aQ-aU*aW;local aY=az(aX)if aY>0 then n=aX elseif aY<0 then m=aX else m=aX;n=aX end;Y=Y+1 end;return(m+n)/2,Y==aD end;function MathUtil.binomCoeffs(aZ,a_)if a_ then coeffs={}else coeffs={}coeffs[1]=1;for _=1,aZ do coeffs[_+1]=coeffs[_]*(aZ-_)/(_+1)end;return coeffs end end;function MathUtil.ruleOfSigns(coeffs,b0)local b1={}local b2=#coeffs;for F=1,b2 do b1[F]=coeffs[b2-F+1]end;if b0~=0 then local b3={}for F=1,b2 do b3[F]=(F-1)*coeffs[b2-F+1]end;local b4=1;for F=2,b2 do local b5=b0^(F-1)for Y=1,b2-F+1 do local b6=F+Y-1;b1[Y]=b1[Y]+b4*b3[b6]*b5;b3[b6]=b3[b6]*(Y-1)end;b4=b4/F end end;local b7={}local o=1;for F,b8 in ipairs(b1)do if b8~=0 then b7[o]=b8;o=o+1 end end;local b9=0;for F=1,#b7-1 do if b7[F]*b7[F+1]<0 then b9=b9+1 end end;return b9 end;function MathUtil.cache(az)local Q={}local ba=getmetatable(Q)or{}function ba.__index(bb,aF)local C=az(aF)bb[aF]=C;return C end;setmetatable(Q,ba)return function(m)return Q[m]end end;function MathUtil.lerp(az,R,S,T,bc)local bd={}for F=1,math.floor((S-R)/T)+1 do bd[F]=az(R+F*T)end;bd.start=R;bd.stop=S;bd.step=T;bd.lval=bc and bd[1]or nil;bd.rval=bc and bd[#bd]or nil;return function(aF)if aF>=bd.stop then return bd.rval end;if aF<=bd.start then return bd.lval end;local F=(aF-bd.start)/bd.step;local be=F%1;F=math.floor(F)return(1-be)*bd[F]+be*bd[F+1]end end;function MathUtil._factorial(o)if o<2 then return 1 end;return MathUtil._factorial(o-1)end;MathUtil.factorial=MathUtil.cache(MathUtil._factorial)MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(aC)MathUtil.eps=aC end;function MathUtil.cuberoot(aF)return aF>0 and aF^(1/3)or-math.abs(aF)^(1/3)end;function MathUtil.solveQuadratic(bf,ao,ap)local bg,bh;local ac,bi,bj;ac=ao/(2*bf)bi=ap/bf;bj=ac*ac-bi;if MathUtil.isZero(bj)then bg=-ac;return bg elseif bj<0 then return else local bk=math.sqrt(bj)bg=bk-ac;bh=-bk-ac;return bg,bh end end;function MathUtil.solveCubic(bf,ao,ap,bl)local bg,bh,bm;local bn,bo;local ai,aj,ak;local bp,ac,bi;local bq,bj;ai=ao/bf;aj=ap/bf;ak=bl/bf;bp=ai*ai;ac=1/3*(-(1/3)*bp+aj)bi=0.5*(2/27*ai*bp-1/3*ai*aj+ak)bq=ac*ac*ac;bj=bi*bi+bq;if MathUtil.isZero(bj)then if MathUtil.isZero(bi)then bg=0;bn=1 else local br=MathUtil.cuberoot(-bi)bg=2*br;bh=-br;bn=2 end elseif bj<0 then local bs=1/3*math.acos(-bi/math.sqrt(-bq))local g=2*math.sqrt(-ac)bg=g*math.cos(bs)bh=-g*math.cos(bs+math.pi/3)bm=-g*math.cos(bs-math.pi/3)bn=3 else local bk=math.sqrt(bj)local br=MathUtil.cuberoot(bk-bi)local a0=-MathUtil.cuberoot(bk+bi)bg=br+a0;bn=1 end;bo=1/3*ai;if bn>0 then bg=bg-bo end;if bn>1 then bh=bh-bo end;if bn>2 then bm=bm-bo end;return bg,bh,bm end;function MathUtil.solveQuartic(bf,ao,ap,bl,bt)local bg,bh,bm,bu;local coeffs={}local a4,br,a0,bo;local ai,aj,ak,bj;local bp,ac,bi,ag;local bn=0;ai=ao/bf;aj=ap/bf;ak=bl/bf;bj=bt/bf;bp=ai*ai;ac=-0.375*bp+aj;bi=0.125*bp*ai-0.5*ai*aj+ak;ag=-(3/256)*bp*bp+0.0625*bp*aj-0.25*ai*ak+bj;if MathUtil.isZero(ag)then coeffs[3]=bi;coeffs[2]=ac;coeffs[1]=0;coeffs[0]=1;local bv={MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])}bn=#bv;bg,bh,bm=bv[1],bv[2],bv[3]elseif MathUtil.isZero(bi)then local bw={MathUtil.solveQuadratic(1,ac,ag)}if bw[1]>=0 then bg=-math.sqrt(bw[1])bh=math.sqrt(bw[1])bn=2 end;if bw[2]>=0 then if bn==0 then bg=-math.sqrt(bw[2])bh=math.sqrt(bw[2])bn=2 else bm=-math.sqrt(bw[2])bu=math.sqrt(bw[2])bn=4 end end else coeffs[3]=0.5*ag*ac-0.125*bi*bi;coeffs[2]=-ag;coeffs[1]=-0.5*ac;coeffs[0]=1;bg,bh,bm=MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])a4=bg;br=a4*a4-ag;a0=2*a4-ac;if MathUtil.isZero(br)then br=0 elseif br>0 then br=math.sqrt(br)else return end;if MathUtil.isZero(a0)then a0=0 elseif a0>0 then a0=math.sqrt(a0)else return end;coeffs[2]=a4-br;coeffs[1]=bi<0 and-a0 or a0;coeffs[0]=1;do local bv={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bn=#bv;bg,bh=bv[1],bv[2]end;coeffs[2]=a4+br;coeffs[1]=bi<0 and a0 or-a0;coeffs[0]=1;if bn==0 then local bv={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bn=bn+#bv;bg,bh=bv[1],bv[2]end;if bn==1 then local bv={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bn=bn+#bv;bh,bm=bv[1],bv[2]end;if bn==2 then local bv={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bn=bn+#bv;bm,bu=bv[1],bv[2]end end;bo=0.25*ai;if bn>0 then bg=bg-bo end;if bn>1 then bh=bh-bo end;if bn>2 then bm=bm-bo end;if bn>3 then bu=bu-bo end;return bg,bh,bm,bu end;function RingBuffer.RingBuffer(bx)local by={}by.buf={}by.capacity=bx;by.size=0;by.head=1;local ba=getmetatable(by)or{}ba.__index=RingBuffer.get;setmetatable(by,ba)return by end;function RingBuffer.isFull(by)return by.size>=by.capacity end;function RingBuffer.setSize(by,bz)by.size=bz end;function RingBuffer.push(by,d)by.buf[(by.head+by.size-1)%by.capacity+1]=d;if by.size==by.capacity then by.head=by.head%by.capacity+1 else by.size=by.size+1 end end;function RingBuffer.pop(by)if by.size==0 then return nil end;local C=by.buf[by.head]by.buf[by.head]=nil;by.head=by.head%by.capacity+1;by.size=by.size-1;return C end;function RingBuffer.get(by,p)if type(p)~="number"or math.floor(p)~=p then return nil end;if p<1 or p>by.size then return nil end;return by.buf[(by.head+p-2)%by.capacity+1]end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(m,n)local bA=type(m)=="number"local bB=type(n)=="number"if not bA and bB then return n+m end;if bA and not bB then return MathUtil.combine(m,n,function(_,aF,bC)return m+bC end)else return MathUtil.combine(m,n,function(_,aF,bC)return aF+bC end)end end;VectorN.mt.__sub=function(m,n)return m+-n end;VectorN.mt.__mul=function(m,n)local bA=type(m)=="number"local bB=type(n)=="number"if not bA and bB then return n*m end;if bA and not bB then local z={}for _,a0 in pairs(n)do z[_]=m*a0 end;return z else return MathUtil.combine(m,n,function(_,aF,bC)return aF*bC end)end end;VectorN.mt.__div=function(m,n)local bA=type(m)=="number"local bB=type(n)=="number"if not bA and bB then return m*1/n end;if bA and not bB then local z={}for _,a0 in pairs(n)do z[_]=m/a0 end;return z else return MathUtil.combine(m,n,function(_,aF,bC)return aF/bC end)end end;VectorN.mt.__unm=function(m)local z={}for _,a0 in pairs(m)do z[_]=-a0 end;return z end;function VectorN.VectorN(B)local bD={}for _,a0 in pairs(B)do if type(a0)=="table"then bD[_]=VectorN.VectorN(a0)else bD[_]=a0 end end;setmetatable(bD,VectorN.mt)return bD end;function Control.PID(bE,bF,bG,bH,bI,bJ)local bK={}bK.kP=bE;bK.kI=bF;bK.kD=bG;bK.Iacc=Accumulator.Accumulator(bH,bI)if bJ and bJ~=0 then bK.period=bJ end;return bK end;function Control.processPID(bL,bM,e)bM=bL.period and(bM+bL.period/2)%bL.period-bL.period/2 or bM;local ac=bL.kP*bM;local F,bN=bL.kI*Accumulator.update(bL.Iacc,bM,e)F=F/bN;local h=bL.kD*(bM-(bL.lastError or bM))/e;bL.lastError=bM;return ac+F+h end;function Control.FF(coeffs,bJ)local bO={}bO.coeffs=coeffs;bO.degree=#coeffs-1;if bJ and bJ~=0 then bO.period=bJ end;bO.derivs={}return bO end;function Control.processFF(bL,bP,e)local bQ=0*bP;local bR=bP;local bS=bP;for F=1,bL.degree+1 do bS=bL.derivs[F]bL.derivs[F]=bR;bQ=bQ+bL.coeffs[F]*bR;if bS then local aS=bR-bS;if F==1 and bL.period then aS=(aS+bL.period/2)%bL.period-bL.period/2 end;bR=aS/e else break end end;return bQ end;function Nav.toLocal(bT,bU,bV)local bW=bT-bU;return Quaternion.Inverse(bV)*bW end;function Nav.toGlobal(bX,bU,bV)local bW=bV*bX;return bW+bU end;function Nav.cartToPol(bY)local ag=bY.magnitude;local ah=Vector3.SignedAngle(Vector3.forward,bY,Vector3.up)local bs=90-Vector3.Angle(Vector3.up,bY)return Vector3(ag,ah,bs)end;function Nav.cartToCyl(bY)local bZ=Vector3(bY.x,0,bY.z)local b_=bZ.magnitude;local bs=Vector3.SignedAngle(Vector3.forward,bY,Vector3.up)local a4=bY.y;return Vector3(b_,bs,a4)end;function Nav.polToCart(bY)local ag,ah,bs=bY.x,bY.y,bY.z;local aF=Mathf.Sin(ah)*Mathf.Cos(bs)local bC=Mathf.Sin(bs)local a4=Mathf.Cos(ah)*Mathf.Cos(bs)return ag*Vector3(aF,bC,a4)end;function Nav.cylToCart(bY)local b_,bs,c0=bY.x,bY.y,bY.z;local aF=b_*Mathf.Sin(bs)local bC=c0;local a4=b_*Mathf.Cos(bs)return Vector3(aF,bC,a4)end;function Targeting.firstOrderTargeting(c1,c2,c3)local c4=c1-Vector3.Project(c1,c2)local c5=Vector3.Dot(c2,c1-c4)/c2.sqrMagnitude;local m,n=MathUtil.solveQuadratic(c5-c3*c3,2*c5,c4.sqrMagnitude+c5*c5)local c6=nil;if m and m>=0 then c6=m end;if n and n>=0 and n<m then c6=n end;return c6 and(c1+c6*c2).normalized or nil end;function Targeting.secondOrderTargeting(c1,c7,c8,c3,c9,ca)local m=-0.25*c8.sqrMagnitude;local n=-Vector3.Dot(c7,c8)local Q=-(c7.sqrMagnitude-c3*c3+Vector3.Dot(c1,c8))local h=-2*Vector3.Dot(c1,c7)local bM=-c1.sqrMagnitude;local g;local cb=c8.magnitude;local cc=c7.magnitude;local cd=c1.magnitude;local ce,cf=MathUtil.solveQuadratic(0.5*cb,cc+c3,-cd)local cg=math.max(ce,cf)local ch;local coeffs={0.5*cb,cc-c3,cd}if MathUtil.ruleOfSigns(coeffs,0)==2 then local ci,cj=MathUtil.solveQuadratic(coeffs[1],coeffs[2],coeffs[3])if ci then ch=math.min(ci,cj)end end;if not ch or ch<cg then local bg,bh,bm=MathUtil.solveCubic(4*m,3*n,2*Q,h)if not bm then if bg>cg then ch=bg end else local ci=math.min(bg,bm)local cj=math.max(bg,bm)if ci>cg then ch=ci elseif cj>cg then ch=cj end end;if not ch then return nil end end;local function ck(aF)return bM+aF*(h+aF*(Q+aF*(n+aF*m)))end;g=MathUtil.ITP(ck,cg,ch,1e-4,25)if not g then return nil end;local cl;if g and g>=cg and g<=ch then cl=c1/g+c7+0.5*c8*g end;if cl and cl.sqrMagnitude>=c9*c9 and cl.sqrMagnitude<=ca*ca then return cl,g end end;function Targeting.AIPPN(cm,c1,cn,c2,co)local c7=c2-cn;local cp=Vector3.Dot(-c7,c1.normalized)if cp<=0 then cp=10 end;local cq=c1.magnitude/cp;local cr=Vector3.Cross(c1,c7)/c1.sqrMagnitude;local cs=Vector3.Cross(c1,co)/c1.sqrMagnitude*cq/2;local ct=cr+cs;local cu=Vector3.Cross(ct,c1.normalized)local cv=Vector3.ProjectOnPlane(cu,cn).normalized;local cw=cm*cn.magnitude*ct.magnitude;return cw*cv end;function Targeting.ATPN(cm,c1,cn,c2,co)local c7=c2-cn;local cp=-Vector3.Dot(c7,c1.normalized)if cp<=0 then cp=10 end;local cr=Vector3.Cross(c1,c7)/c1.sqrMagnitude;local cu=Vector3.Cross(cr,c1.normalized)local cx=Vector3.ProjectOnPlane(co,c1)return cm*cp*cu+0.5*cm*co end;function BlockUtil.getWeaponsByName(cy,cz,b9,cA)if DEBUG then cy:Log("searching for "..cz)end;local cB=cy:GetAllSubConstructs()local cC={}b9=b9 or-1;local Q=b9;if not cA or cA==0 or cA==2 then for F=0,cy:GetWeaponCount()-1 do if Q==0 then break end;if cy:GetWeaponBlockInfo(F).CustomName==cz then table.insert(cC,{subIdx=nil,wpnIdx=F})if DEBUG then cy:Log("found weapon "..cz.." on hull, type "..cy:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end;if not cA or cA==1 or cA==2 then for p=1,#cB do local bo=cB[p]for F=0,cy:GetWeaponCountOnSubConstruct(bo)-1 do if Q==0 then break end;if cy:GetWeaponBlockInfoOnSubConstruct(bo,F).CustomName==cz then table.insert(cC,{subIdx=bo,wpnIdx=F})if DEBUG then cy:Log("found weapon "..cz.." on subobj "..bo..", type "..cy:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end end;if DEBUG then cy:Log("weapon count: "..#cC)end;return cC end;function BlockUtil.getSubConstructsByName(cy,cz,b9)if DEBUG then cy:Log("searching for "..cz)end;local cB=cy:GetAllSubConstructs()local cD={}b9=b9 or-1;local Q=b9;for p=1,#cB do local bo=cB[p]if Q==0 then break end;if cy:GetSubConstructInfo(bo).CustomName==cz then table.insert(cD,bo)if DEBUG then cy:Log("found subobj "..cz)end;Q=Q-1 end end;if DEBUG then cy:Log("subobj count: "..#cD)end;return cD end;function BlockUtil.getBlocksByName(cy,cz,type,b9)if DEBUG then cy:Log("searching for "..cz)end;local cE={}b9=b9 or-1;local Q=b9;for p=0,cy:Component_GetCount(type)-1 do if Q==0 then break end;if cy:Component_GetBlockInfo(type,p).CustomName==cz then table.insert(cE,p)if DEBUG then cy:Log("found component "..cz)end;Q=Q-1 end end;if DEBUG then cy:Log("component count: "..#cE)end;return cE end;function BlockUtil.getWeaponInfo(cy,cF)if cF.subIdx then return cy:GetWeaponInfoOnSubConstruct(cF.subIdx,cF.wpnIdx)end;return cy:GetWeaponInfo(cF.wpnIdx)end;function BlockUtil.getWeaponBlockInfo(cy,cF)if cF.subIdx then return cy:GetWeaponBlockInfoOnSubConstruct(cF.subIdx,cF.wpnIdx)end;return cy:GetWeaponBlockInfo(cF.wpnIdx)end;function BlockUtil.aimWeapon(cy,cF,cG,cH)if cF.subIdx then cy:AimWeaponInDirectionOnSubConstruct(cF.subIdx,cF.wpnIdx,cG.x,cG.y,cG.z,cH)else cy:AimWeaponInDirection(cF.wpnIdx,cG.x,cG.y,cG.z,cH)end end;function BlockUtil.fireWeapon(cy,cF,cH)if cF.subIdx then return cy:FireWeaponOnSubConstruct(cF.subIdx,cF.wpnIdx,cH)end;return cy:FireWeapon(cF.wpnIdx,cH)end;function Combat.pickTarget(cy,cI,cJ)cJ=cJ or function(U,cK)return cK.Priority end;local bP,cL;for F in MathUtil.range(cy:GetNumberOfTargets(cI))do local cK=cy:GetTargetInfo(cI,F)local cM=cJ(cy,cK)if not bP or cM>cL then bP=cK;cL=cM end end;return bP end;function Combat.CheckConstraints(cy,cN,cO,cP)local cQ;if cP then cQ=cy:GetWeaponConstraintsOnSubConstruct(cP,cO)else cQ=cy:GetWeaponConstraints(cO)end;local cR=cy:GetConstructForwardVector()local cS=cy:GetConstructUpVector()local cT=Quaternion.LookRotation(cR,cS)cN=Quaternion.Inverse(cT)*cN;if cQ.InParentConstructSpace and cP then local cU=cy:GetSubConstructInfo(cP).localRotation;cN=Quaternion.inverse(cU)*cN end;local cV=MathUtil.angleOnPlane(Vector3.forward,cN,Vector3.up)local cW=cN;cW.z=0;local O=Mathf.Atan2(cN.z,cW.magnitude)local cX=cV>cQ.MinAzimuth and cV<cQ.MaxAzimuth;local cY=O>cQ.MinElevation and O<cQ.MaxElevation;if cQ.FlipAzimuth then cX=not cX end;if cX and cY then return true end;cV=cV+180;O=180-O;if O>180 then O=O-360 end;if O<-180 then O=O+360 end;cX=cV>cQ.MinAzimuth and cV<cQ.MaxAzimuth;cY=O>cQ.MinElevation and O<cQ.MaxElevation;if cQ.FlipAzimuth then cX=not cX end;if cX and cY then return true end;return false end;function StringUtil.LogVector(cy,bD,cZ)cy:Log(cZ.."("..bD.x..", "..bD.y..", "..bD.z..")")end
