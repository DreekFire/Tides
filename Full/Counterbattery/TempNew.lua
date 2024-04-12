-- Settings
-- use pCall to avoid crash, retry if failed
local AllowErrorRecovery = false
local DebugLevel = 1
local DebugControlOutputs = false
-- seconds to record target movements for
local targetTrackTime = 15
-- number of locations to track per enemy (todo: support tracking multiple projectiles simultaneously)
local numOrigins = 1
-- time between switching targets
local originSwitchTime = 2
-- maximum time to remember origin points
local maxStaleness = 3

-- custom axis containing stability value (pass using breadboard)
local stabilityAxisName = "Stability"
-- will fire if any are true, set to 0 to ignore
-- will fire if stability exceeds this value
local minStabilityToFire = 0.87
-- will fire if stability exceeds the average stability
  -- calculated by an exponential filter with time constant of stabilityAveragingTime
local stabilityAvgTime = 3
-- will fire if stability is the highest its been in this many seconds
local stabilityMaxTime = 1

-- altitudes to engage (estimated altitude of firing piece, not target position)
  -- checked when adding an origin
local minAlt = -50
local maxAlt = math.huge
-- parameters to find weapons (Shift+N to name turret blocks)
  -- different turrets should have different names even if they have the same weapon
  -- if all weapons on a turret have the same muzzle velocity then just name the turret
  -- if multiple weapons on same turret with different muzzle velocity then name the turret,
    -- leave primary weapon unnamed and give secondary weapons unique names
    -- turret name should go before names of secondary weapons
local weaponDef = {
  { name = "CB1",
    velocity = 1649,
    minRange = 50,
    maxRange = 4000,
    minAlt = -50,
    maxAlt = math.huge, },
  { name = "CB2",
    velocity = 1605,
    minRange = 50,
    maxRange = 4000,
    minAlt = -50,
    maxAlt = math.huge, }
}
-- checkACB lists ACBs set to detect object presence and trigger custom axis
  -- will be checked before firing to make sure blocks actually exist at target distance
  -- this check is not performed when firing at aimpoint due to idle mode "fire" or "timer"
  -- may also have second ACB with inverted settings that trigger negative custom axis, in which case set zeroInvalid to true
local checkACBs = {
  { axis = "Check1", minRange = 200, maxRange = 4000, turretName = "check1Turret", offset = Vector3(0, 1, 0), zeroInvalid = false },
  { axis = "Check2", minRange = 200, maxRange = 4000, turretName = "check2Turret", offset = Vector3(0, 1, 0), zeroInvalid = false },
}
-- checkACBRequire is required ratio of sum of axis values to number of valid ACBs 
local checkACBRequire = 1
-- validACBRequire is required number of ACBs in range and pointing in the right direction
local validACBRequire = 1
-- indexes of mainframes to be used to track enemy rotations
  -- 3 is the bare minimum, but will fail if any of them target a subconstruct
  -- so more is preferred
  -- it will also fail if too many mainframes switch their aimpoint simultaneously
  -- this is unavoidable in the case of blocks being destroyed, but can be avoided
  -- in the case of the timer switching blocks. Todo: manage switch times to avoid
  -- synchronized aimpoint switching
local aimPointTrackers = {
  { idx = 0 },
  { idx = 1 },
  { idx = 2 },
  { idx = 3 },
  { idx = 4 },
  { idx = 5 },
  { idx = 6 },
  { idx = 7 },
  { idx = 8 },
}
-- which mainframe to use, will affect target prioritization
  -- currently only fires at the highest priority target
  -- can be the same as one of the aimPointTrackers
local mainframeIdx = 2
-- what to do when no firing origin detected
  -- timer: wait for waitTime seconds, then fire at current target if no origin found
  -- fire: aim and fire at current target
  -- enemy: aim at current target
  -- none: return to idle position
  -- last: continue aiming at last absolute bearing
local idleAim = "timer"
-- time to wait for a new origin before firing at the aimpoint
  -- only used in idle aim mode "timer"
local waitTime = 2
-- offset the aimpoint, i.e. for hitting the necks, tetris, or turret bases instead of the barrels
  -- is a function with the following properties:
  --[[
    Arguments:
      origin - if isCounter is true, then the original origin, a table containing:
          position - position of firing point relative to enemy position
          time - estimated time at which projectile was fired,
          relVel - velocity of projectile relative to enemy at firing time,
          detectPos - position at which the projectile was detected,
          detectTime - the time at which the projectile was detected
          type - 0 for APS, 1 for CRAM, 2 for missile
        otherwise just the aimpoint position relative to the enemy position
      target - the target position
      targetVel - the target velocity
      targetRotation - the target rotation relative to when the target was first detected
      isCounter - whether we are firing at an origin detected by the counterbattery script or just the AI aimpoint
    Returns:
      fpOffset - the offset of the adjusted aimpoint relative to the original aimpoint
  ]]
  -- to adjust depending on estimated turret location
-- todo: return list of locations to attempt, using the checkACBs to check each one
local function aimOffset(origin, target, targetVel, targetRot, isCounter)
  -- example: against APS, try to break tetris, against CRAM, try to hit firing piece (assumes barrel is about 8m long)
  --  against AI aimpoint or missile launcher, does not modify aimpoint
  if isCounter then
    if origin.type == 0 then
      return Vector3(0, -7, 0)
    elseif origin.type == 1 then
      return -8 * origin.relVel.normalized
    end
  end
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
local frameTime
local lastFrameTime
local inited
local prevTime
local lastOrigin
local lastOriginSwitchTime = 0
local currentTargetId
local originPopTime = 0
local lastAim
local nextRecordTime
local trackLossTime = 0
local continueLine = false
local stabilityAvg = 1
local stabilityWindow
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
  end

  for acbIdx, acb in ipairs(checkACBs) do
    acb.turretSub = BlockUtil.getSubConstructsByName(I, acb.turretName, 1)[1]
    acb.turret = BlockUtil.getWeaponsByName(I, acb.turretName, 1)[1]
  end

  nextRecordTime = I:GetTimeSinceSpawn()
  enemies = {}
  if stabilityMaxTime > 0 then
    stabilityWindow = RingBuffer.RingBuffer(TICKS_PER_S * stabilityMaxTime)
  end
  math.randomseed(I:GetTime())
  math.random()
  math.random()

  originPopTime = I:GetTimeSinceSpawn()
  inited = true
end

function Update(I)
  I:ClearLogs()
  I:Log(string.format("Game Time: %.2f", I:GetTimeSinceSpawn()))
  I:Log(string.format("Total rotation track loss time: %.3f", trackLossTime))
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
    local stability = I:GetCustomAxis(stabilityAxisName)
    if stability then
      --I:Log("Current stability: "..stability)
      if stabilityAvgTime > 0 then
        stabilityAvg = (1 - 1 / stabilityAvgTime) * stabilityAvg + (1 / stabilityAvgTime) * stability
        --[[if stability >= stabilityAvg then
          I:Log("Stability is at least average")
        end]]
      end
      if stabilityMaxTime > 0 then
        while stabilityWindow.size > 0 and RingBuffer.get(stabilityWindow, stabilityWindow.size) <= stability do
          RingBuffer.pop(stabilityWindow)
        end
        RingBuffer.push(stabilityWindow, stability)
        --[[if stabilityWindow.size == 1 then
          I:Log("Stability is the highest its been in the past "..stabilityMaxTime.." seconds")
        end]]
      end
    end
    nextRecordTime = nextRecordTime + 1 / TICKS_PER_S
  end
  currentTargetId = target.Id

  frameTime = lastFrameTime and t - lastFrameTime or 1 / TICKS_PER_S
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
            local origin = {
              position = lPos,
              time = t - warn.TimeSinceLaunch,
              relVel = warn.Velocity - target.Velocity,
              detectTime = t,
              detectPos = warn.Position,
              type = 2,
            }
            RingBuffer.push(enemy.origins, origin)
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
  local eps = 15 * frameTime * frameTime
  -- todo: maybe limit how often the line tracing runs to save processing power
  if lastProjectilePos and currentLine and CheckAndUpdateLine(I, currentLine, projectile, eps) then
    if not continueLine then
      local enemy = enemies[currentTargetId]
      local origin = GetOrigin(I, currentLine, enemy)
      if origin then
        RingBuffer.push(enemy.origins, origin)
      end
    end
    continueLine = true
  else
    continueLine = false
    if lastProjectilePos and dist > 0 then
      local start = lastProjectilePos
      currentLine = {
        start = start,
        tStart = t - frameTime,
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
  if I:GetAIFiringMode(mainframeIdx) == "off" then return end
  local enemy = enemies[currentTargetId]
  local eRot = enemy.rotation[enemy.rotation.size] or Quaternion.identity
  local origin, fp
  if lastOriginSwitchTime + originSwitchTime > t then
    origin = lastOrigin
  end
  if not origin then
    while enemy.origins.size > 0 and t - enemy.origins[1].detectTime > maxStaleness do
      RingBuffer.pop(enemy.origins)
      originPopTime = t
    end
    if enemy.origins.size == 0 then
      I:Log("No origins found.")
    else
      local candidate = enemy.origins[math.random(1, enemy.origins.size)]
      local originAlt = (eRot * candidate.position).y + target.Position.y
      if originAlt < minAlt or originAlt > maxAlt then
        -- do a linear search to find valid origin
        for i = 1, enemy.origins.size do
          candidate = enemy.origins[i]
          originAlt = (eRot * candidate.position).y + target.Position.y
          if originAlt > minAlt and originAlt < maxAlt then
            origin = candidate
            break
          end
        end
        if not origin then
          I:Log("No origin in bounds.")
        end
      else
        origin = candidate
      end
    end
    lastOriginSwitchTime = t
  end
  lastOrigin = origin

  if origin then
    I:Log("Aiming at origin")
    fp = eRot * origin.position + aimOffset(origin, target.Position, target.Velocity, eRot, true)
    -- check ACBs for object presence
    local valid = 0
    local score = 0
    for acbIdx, acb in ipairs(checkACBs) do
      local bInfo = I:GetSubConstructInfo(acb.turretSub)
      local r = fp + target.Position - (bInfo.Position + bInfo.Rotation * acb.offset)
      BlockUtil.aimWeapon(I, acb.turret, r, 0)
      if r.magnitude > acb.minRange and r.magnitude < acb.maxRange
          and I:IsAlive(acb.turretSub)
          and Vector3.Angle(r, bInfo.Forwards) < AIM_TOL then
        local val = I:GetCustomAxis(acb.axis)
        if val ~= 0 or not acb.zeroInvalid then
          score = score + I:GetCustomAxis(acb.axis)
          valid = valid + 1
        end
      end
    end
    if valid < validACBRequire or score < checkACBRequire * valid then
      I:Log(string.format("Origin ACB check failed: score %.2f of %.2f required with %d valid ACBs of %d required", score, checkACBRequire * valid, valid, validACBRequire))
      fp = nil
    else
      CallComplexControl(I, "T")
      I:Log("Origin ACB check passed")
    end
  end
  if not fp then
    I:Log("Aiming at AI aimpoint")
    fp = target.AimPointPosition - target.Position
    fp = fp + aimOffset(fp, target.Position, target.Velocity, eRot, false)
  end
  local stability = I:GetCustomAxis(stabilityAxisName)
  local stabilityOk = not stability or (stability >= minStabilityToFire or stability >= stabilityAvg or stabilityWindow.size == 1)

  for i, turret in ipairs(turrets) do
    local fpAlt = fp.y + target.Position.y
    if fpAlt >= weaponDef[i].minAlt and fpAlt <= weaponDef[i].maxAlt then
      for j, weapon in ipairs(turret) do
        if DebugLevel > 1 then I:Log(string.format("Controlling Turret:%0.0f Weapon:%0.0f", i, j)) end
        local aim
        local fire = false
        local ready = origin or (idleAim == "fire" or (idleAim == "timer" and t - originPopTime > waitTime))
        local wInfo = BlockUtil.getWeaponInfo(I, weapon)
        local mv = weaponDef[i].velocity or (wInfo.Speed > 1e5 and math.huge or wInfo.Speed)
        if mv == math.huge then
          local range = (fp + target.Position - I:GetConstructPosition()).magnitude
          if range > weaponDef[i].minRange and range < weaponDef[i].maxRange then
            aim = fp + target.Position - wInfo.GlobalFirePoint
            fire = ready
          end
        else
          local g = 0.5 * (I:GetGravityForAltitude(I:GetConstructPosition().y) + I:GetGravityForAltitude(target.Position.y))
          aim = Targeting.secondOrderTargeting(fp + target.Position - wInfo.GlobalFirePoint,
                      target.Velocity - I:GetVelocityVector(),
                      -g,
                      mv, weaponDef[i].minRange, weaponDef[i].maxRange)
          if aim then
            fire = ready
          else
            I:Log("Second order targeting failed")
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
              if stabilityOk then
                I:Log("Fire!")
                BlockUtil.fireWeapon(I, weapon, 0)
              else
                I:Log("Holding Fire for Stability.")
              end
            else
              I:Log("Holding fire for Aim.")
            end
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
        lastOriginSwitchTime = -originSwitchTime - 1
        currentTargetId = nil
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
    local dRot = GetRotation(en.oldAimpoints, en.aimpoints)
    if not dRot then
      dRot = Quaternion.identity
      trackLossTime = trackLossTime + 1 / TICKS_PER_S
    end
    RingBuffer.push(en.rotation, dRot * (en.rotation[en.rotation.size] or Quaternion.identity))
  end
end

function GetOrigin(I, line, enemy)
  if prevTime and RingBuffer.isFull(enemy.pos) then
    prevTime = prevTime - 1
  end
  local relVel = line.ev / frameTime - enemy.vel[enemy.vel.size]
  local relPos = line.ed - enemy.pos[enemy.pos.size]
  local time2d = math.sqrt((relPos.x ^ 2 + relPos.z ^ 2) / (relVel.x ^ 2 + relVel.z ^2))
  local g = I:GetGravityForAltitude(currentLine.start.y)
  local estimate2d = relPos - (relVel * time2d) + 0.5 * g * time2d * time2d
  -- todo: adjust threshold based on enemy size, also penalize vertical error greater than horizontal
  if estimate2d.sqrMagnitude < 150 * 150 then
    local t = I:GetTimeSinceSpawn()
    local launchPos, launchTime = CheckIfCram(line.ed, line.ev / frameTime, enemy.pos[enemy.pos.size], enemy.vel[enemy.vel.size], 150)
    if launchPos then
      I:Log("Projectile determined to be CRAM shell, fired "..launchTime.." seconds ago")
      local pIdx = enemy.pos.size + launchTime * TICKS_PER_S
      if pIdx < 1 then
        I:Log("CRAM direct guess has no target data")
        return
      end
      local enemyPosAtTime = enemy.pos[pIdx]
      local enemyRotAtTime = enemy.rotation[pIdx]
      if enemyPosAtTime then
        local origin = {
          position = Quaternion.Inverse(enemyRotAtTime) * (launchPos - enemyPosAtTime),
          time = t + launchTime,
          relVel = line.ev / frameTime + g * launchTime - enemy.vel[pIdx],
          detectTime = t,
          detectPos = line.ed,
          type = 1,
        }
        return origin
      end
    end
    local closest, closestTimeIdx = RunTrace(I, line, enemy, prevTime)
    if not closest then
      I:Log("Trace failed")
      prevTime = nil
      return
    end
    prevTime = closestTimeIdx
    if closest.sqrMagnitude < 100 * 100 then
      local closestAlt = closest.y + enemy.pos[closestTimeIdx].y
      if closestAlt > minAlt and closestAlt < maxAlt then
        local origin = {
          position = Quaternion.Inverse(enemy.rotation[closestTimeIdx]) * closest,
          time = t + (closestTimeIdx - enemy.pos.size) / TICKS_PER_S,
          relVel = line.ev / frameTime + g * t - enemy.vel[closestTimeIdx],
          detectTime = t,
          detectPos = line.ed,
          type = 0,
        }
        return origin
      end
    end
  end
end

function GetRotation(oldPts, newPts, iterLim)
  local nTrack = #newPts
  iterLim = iterLim or 25

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

function CheckIfCram(projPos, projVel, enemyPos, enemyVel, tolerance)
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
      p = CheckPosition(projPos, projVel, enemyPos, enemyVel, t2, tolerance)
      if p then return p, t2 end
    end
    if t1 <= 0 then
      p = CheckPosition(projPos, projVel, enemyPos, enemyVel, t1, tolerance)
      if p then return p, t1 end
    end
  end
  if maybeS then
    t1, t2 = GetCramLaunchTime(projVel, 240)
    if t2 <= 0 then
      p = CheckPosition(projPos, projVel, enemyPos, enemyVel, t2, tolerance)
      if p then return p, t2 end
    end
    if t1 <= 0 then
      p = CheckPosition(projPos, projVel, enemyPos, enemyVel, t1, tolerance)
      if p then return p, t1 end
    end
  end
end

function CheckPosition(projPos, projVel, enemyPos, enemyVel, t, tolerance)
  local p = projPos - enemyPos + (projVel - enemyVel) * t - 0.5 * Vector3(0, 9.81, 0) * t * t
  if p.sqrMagnitude < tolerance * tolerance then
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

function CheckAndUpdateLine(I, line, projectile, tolerance)
  -- the theoretical equation has a 0.5 before the quadratic term, but discrete integration results in no 0.5
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
  StringUtil.LogVector(I, expected - projectile, "line match error: ")
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

local stimTable = {
  T = 1,
  G = 2,
  Y = 3,
  H = 4,
  U = 5,
  J = 6,
  I = 7,
  K = 8,
  O = 9,
  L = 10,
  UP = 11,
  DOWN = 12,
  LEFT = 13,
  RIGHT = 14,
}

function CallComplexControl(I, keyRequested)
  --T G Y H U J I K O L up down left right

  if keyRequested == nil then return end
  if string.len(keyRequested) == 0 then return end

  keyRequested = string.upper(keyRequested)

  local stim = stimTable[keyRequested]

  if stim > 0 then
    I:RequestComplexControllerStimulus(stim)
    if DebugControlOutputs then
      I:Log("ComplexKey Output: " .. keyRequested)
    end
  else
    if DebugControlOutputs then
      I:Log("Requested Output '" .. keyRequested .. "' not recognized")
    end
  end
end

-- minified version of Tides library (not meant to be human-readable, see Tides.lua or individual class files for human-readable source)
function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function Heapq.Heapq(i,k)local l={}l.data=i;l.comp=k or function(m,n)return m<n end;local o=#l.data;l.size=o;for p=math.floor(o/2),1,-1 do Heapq.siftDown(l,p)end;return l end;function Heapq.siftDown(l,q)local r=false;local s=q;local o=#l.data;while not r do r=true;local t=2*s;local u=2*s+1;local v=s;if t<=o and l.comp(l.data[t],l.data[v])then v=t;r=false end;if u<=o and l.comp(l.data[u],l.data[v])then v=u;r=false end;if not r then local w=l.data[v]l.data[v]=l.data[s]l.data[s]=w;s=v end end end;function Heapq.siftUp(l,q)local r=false;local s=q;while not r do r=true;local x=math.floor(s/2)if l.comp(l.data[s],l.data[x])then local w=l.data[x]l.data[x]=l.data[s]l.data[s]=w;s=x;r=false end end end;function Heapq.insert(l,y)l.data[l.size+1]=y;l.size=l.size+1;Heapq.siftUp(l,l.size)end;function Heapq.pop(l)local z=l.data[1]l.data[1]=l.data[l.size]l.data[l.size]=nil;l.size=l.size-1;Heapq.siftDown(l,1)return z end;function Heapq.peek(l)return l.data[1]end;function Heapq.size(l)return l.size end;function LinkedList.LinkedList()local A={}A.value=nil;A.next=A;A.prev=A;return A end;function LinkedList.pushFront(B,d)local A={}A.value=d;LinkedList.connect(A,B.next)LinkedList.connect(B,A)end;function LinkedList.pushBack(B,d)local A={}A.value=d;LinkedList.connect(B.prev,A)LinkedList.connect(A,B)end;function LinkedList.popFront(B)local C=B.next.value;LinkedList.connect(B,B.next.next)return C end;function LinkedList.popBack(B)local C=B.prev.value;LinkedList.connect(B.prev.prev,B)return C end;function LinkedList.peekFront(B)return B.next.val end;function LinkedList.peekBack(B)return B.prev.val end;function LinkedList.connect(D,E)D.next=E;E.prev=D end;function LinkedList.toArray(B)local F=1;local G={}local A=B.next;while A~=B do G[F]=A.value;A=A.next end;return G end;function MathUtil.angleOnPlane(H,I,J)local K=Vector3.ProjectOnPlane(H,J)local g=Vector3.ProjectOnPlane(I,J)return Vector3.SignedAngle(K,g,J)end;function MathUtil.min(L,M)local N=nil;M=M or function(m,n)return m<n end;for O in L do if not N or M(O,N)then N=O end end;return N end;function MathUtil.max(L,M)local P=nil;M=M or function(m,n)return m<n end;for O in L do if not P or M(P,O)then P=O end end;return P end;function MathUtil.range(m,n,Q)local R,S=m,n;local T;if not m then return end;if not n then R=0;S=m;T=R<S and 1 or-1 elseif Q then T=Q end;return function(U,V)local W=V+T;if W==S then return nil end;return W end,nil,R-T end;function MathUtil.shuffle(B,X)local s=X and B or{}if not X then for F=1,#B do s[F]=B[F]end end;for F=#B,2,-1 do local Y=math.random(F)s[F],s[Y]=s[Y],s[F]end;return s end;function MathUtil.combine(m,n,Z)if#m==#n then local z={}for _,a0 in pairs(m)do z[_]=Z(_,a0,n[_])end;return z end end;function MathUtil.distribution()return{n=0}end;function MathUtil.updateDistribution(a1,a2)a1.n=a1.n+1;if a1.n==1 then a1.mean=a2;a1.covariance={}local h=#a2;for F=1,h do local a3={}for Y=1,h do a3[Y]=0 end;a1.covariance[F]=a3 end else a1.mean=a1.mean+1/(a1.n+1)*a2 end end;function MathUtil.mean(a1)return a1.mean end;function MathUtil.covariance(a1)return a1.cov end;function MathUtil.normal()local a4,a5=MathUtil.boxMuller()return a4 end;function MathUtil.normalPDF(a4)return math.exp(-0.5*a4*a4)/math.sqrt(2*math.pi)end;function MathUtil.normalCDF(a4)local a6=0.2316419;local a7=0.319381530;local a8=-0.356563782;local a9=1.781477937;local aa=-1.821255978;local ab=1.330274429;local g=1/(1+a6*a4)return 1-MathUtil.normalPDF(a4)*(a7*g+a8*g^2+a9*g^3+aa*g^4+ab*g^5)end;function MathUtil.inverseNorm(ac)local ad=ac>=0.5 and ac or-ac;local a4=5.55556*(1-((1-ad)/ad)^0.1186)if ac<0.5 then a4=-a4 end;return a4 end;function MathUtil.boxMuller()local ae=math.random()local af=math.random()af=math.random()af=math.random()local ag=math.sqrt(-2*math.log(ae))local ah=2*math.pi*af;return ag*math.cos(ah),ag*math.sin(ah)end;function MathUtil.angleSSS(m,n,Q)if m+n<Q or m+Q<n or n+Q<m then return nil end;local ai=math.deg(math.acos((n*n+Q*Q-m*m)/(2*n*Q)))local aj,ak=MathUtil.angleSAS(n,ai,Q)return ai,aj,ak end;function MathUtil.sideSAS(m,ak,n)local al=m*m+n*n-2*m*n*math.cos(math.rad(ak))return math.sqrt(al)end;function MathUtil.angleSAS(m,ak,n)local Q=MathUtil.sideSAS(m,ak,n)if MathUtil.isZero(Q)then return nil end;local ai,aj;if m<n then ai=MathUtil.angleLoSin(Q,m,ak)aj=180-ai-ak else aj=MathUtil.angleLoSin(Q,n,ak)ai=180-aj-ak end;return ai,aj end;function MathUtil.sideSSA(m,n,ai)local am=n*n-m*m;local an=-2*n*math.cos(math.rad(ai))local ao,ap=MathUtil.solveQuadratic(1,an,am)if not ap then return ao,ap end;if ao<ap then return ao,ap end;return ap,ao end;function MathUtil.angleSSA(m,n,ai)local ao,ap=MathUtil.sideSSA(m,n,ai)if not ao then return nil end;local aq,ar=MathUtil.angleSAS(n,ai,ao)if not ap then return aq,ar end;local as,at=MathUtil.angleSAS(n,ai,ap)return aq,ar,as,at end;function MathUtil.sideAAS(ai,aj,m)local ak=180-ai-aj;local n=MathUtil.sideLoSin(ai,aj,m)local Q=MathUtil.sideLoSin(ai,ak,m)return n,Q end;function MathUtil.sideLoSin(m,ai,aj)return m*math.sin(math.rad(aj))/math.sin(math.rad(ai))end;function MathUtil.angleLoSin(m,n,ai)return math.deg(math.asin(n*math.sin(math.rad(ai))/m))end;function MathUtil.clampCone(au,av,aw)local ax=math.min(aw,Vector3.Angle(au,av))local ay=Vector3.Cross(au,av)return Quaternion.AngleAxis(ax,ay)*au end;function MathUtil.newton(az,aA,aB,aC,aD,aE)aC=aC or 1e-5;aE=aE or 10*aC;aD=aD or 25;aA=aA or function(aF)return(az(aF+aE)-az(aF))/aE end;aB=aB or 0;local aG=aC+1;local aH=0;while aG>aC and aH<aD do local aI=az(aB)local aJ=aA(aB)if not aI or not aJ then return nil end;aG=-aI/aJ;aB=aB+aG;aH=aH+1 end;if aH<aD then return aB,false end;return aB,true end;function MathUtil.ITP(az,m,n,aC,aD)if az(m)*az(n)>0 then return nil end;local aK;if az(m)>az(n)then az=function(aF)return-az(aF)end end;aC=aC or 1e-5;aD=aD or 25;local aL=0.2/(n-m)local aM=2;local aN=1;local aO=math.ceil(math.log((n-m)/(2*aC),2))local aP=aO+aN;local Y=0;while n-m>2*aC and Y<aD do local aQ=0.5*(m+n)local aR=az(m)-az(n)if aR==0 then return m end;local aS=(n*az(m)-m*az(n))/aR;local aT=aQ-aS;local aU=aL*math.abs(n-m)^aM;local aV=aT>0 and 1 or(aT==0 and 0 or-1)local aW=aU<=math.abs(aT)and aS+aV*aU or aQ;local aX=aC*2^(aP-Y)-0.5*(n-m)local aY=math.abs(aW-aQ)<=aX and aW or aQ-aV*aX;local aZ=az(aY)if aZ>0 then n=aY elseif aZ<0 then m=aY else m=aY;n=aY end;Y=Y+1 end;local aR=az(m)-az(n)if aR~=0 then return(n*az(m)-m*az(n))/aR,Y==aD end;return m,Y==aD end;function MathUtil.binomCoeffs(a_,b0)if b0 then coeffs={}else coeffs={}coeffs[1]=1;for _=1,a_ do coeffs[_+1]=coeffs[_]*(a_-_)/(_+1)end;return coeffs end end;function MathUtil.ruleOfSigns(coeffs,b1)local b2={}local b3=#coeffs;for F=1,b3 do b2[F]=coeffs[b3-F+1]end;if b1~=0 then local b4={}for F=1,b3 do b4[F]=(F-1)*coeffs[b3-F+1]end;local b5=1;for F=2,b3 do local b6=b1^(F-1)for Y=1,b3-F+1 do local b7=F+Y-1;b2[Y]=b2[Y]+b5*b4[b7]*b6;b4[b7]=b4[b7]*(Y-1)end;b5=b5/F end end;local b8={}local o=1;for F,b9 in ipairs(b2)do if b9~=0 then b8[o]=b9;o=o+1 end end;local ba=0;for F=1,#b8-1 do if b8[F]*b8[F+1]<0 then ba=ba+1 end end;return ba end;function MathUtil.cache(az)local Q={}local bb=getmetatable(Q)or{}function bb.__index(bc,aF)local C=az(aF)bc[aF]=C;return C end;setmetatable(Q,bb)return function(m)return Q[m]end end;function MathUtil.lerp(az,R,S,T,bd)local be={}for F=1,math.floor((S-R)/T)+1 do be[F]=az(R+F*T)end;be.start=R;be.stop=S;be.step=T;be.lval=bd and be[1]or nil;be.rval=bd and be[#be]or nil;return function(aF)if aF>=be.stop then return be.rval end;if aF<=be.start then return be.lval end;local F=(aF-be.start)/be.step;local bf=F%1;F=math.floor(F)return(1-bf)*be[F]+bf*be[F+1]end end;function MathUtil._factorial(o)if o<2 then return 1 end;return MathUtil._factorial(o-1)end;MathUtil.factorial=MathUtil.cache(MathUtil._factorial)MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(aC)MathUtil.eps=aC end;function MathUtil.cuberoot(aF)return aF>0 and aF^(1/3)or-math.abs(aF)^(1/3)end;function MathUtil.solveQuadratic(bg,ao,ap)local bh,bi;local ac,bj,bk;ac=ao/(2*bg)bj=ap/bg;bk=ac*ac-bj;if MathUtil.isZero(bk)then bh=-ac;return bh elseif bk<0 then return else local bl=math.sqrt(bk)bh=bl-ac;bi=-bl-ac;return bh,bi end end;function MathUtil.solveCubic(bg,ao,ap,bm)local bh,bi,bn;local bo,bp;local ai,aj,ak;local bq,ac,bj;local br,bk;ai=ao/bg;aj=ap/bg;ak=bm/bg;bq=ai*ai;ac=1/3*(-(1/3)*bq+aj)bj=0.5*(2/27*ai*bq-1/3*ai*aj+ak)br=ac*ac*ac;bk=bj*bj+br;if MathUtil.isZero(bk)then if MathUtil.isZero(bj)then bh=0;bo=1 else local bs=MathUtil.cuberoot(-bj)bh=2*bs;bi=-bs;bo=2 end elseif bk<0 then local bt=1/3*math.acos(-bj/math.sqrt(-br))local g=2*math.sqrt(-ac)bh=g*math.cos(bt)bi=-g*math.cos(bt+math.pi/3)bn=-g*math.cos(bt-math.pi/3)bo=3 else local bl=math.sqrt(bk)local bs=MathUtil.cuberoot(bl-bj)local a0=-MathUtil.cuberoot(bl+bj)bh=bs+a0;bo=1 end;bp=1/3*ai;if bo>0 then bh=bh-bp end;if bo>1 then bi=bi-bp end;if bo>2 then bn=bn-bp end;return bh,bi,bn end;function MathUtil.solveQuartic(bg,ao,ap,bm,bu)local bh,bi,bn,bv;local coeffs={}local a4,bs,a0,bp;local ai,aj,ak,bk;local bq,ac,bj,ag;local bo=0;ai=ao/bg;aj=ap/bg;ak=bm/bg;bk=bu/bg;bq=ai*ai;ac=-0.375*bq+aj;bj=0.125*bq*ai-0.5*ai*aj+ak;ag=-(3/256)*bq*bq+0.0625*bq*aj-0.25*ai*ak+bk;if MathUtil.isZero(ag)then coeffs[3]=bj;coeffs[2]=ac;coeffs[1]=0;coeffs[0]=1;local bw={MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])}bo=#bw;bh,bi,bn=bw[1],bw[2],bw[3]elseif MathUtil.isZero(bj)then local bx={MathUtil.solveQuadratic(1,ac,ag)}if bx[1]>=0 then bh=-math.sqrt(bx[1])bi=math.sqrt(bx[1])bo=2 end;if bx[2]>=0 then if bo==0 then bh=-math.sqrt(bx[2])bi=math.sqrt(bx[2])bo=2 else bn=-math.sqrt(bx[2])bv=math.sqrt(bx[2])bo=4 end end else coeffs[3]=0.5*ag*ac-0.125*bj*bj;coeffs[2]=-ag;coeffs[1]=-0.5*ac;coeffs[0]=1;bh,bi,bn=MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])a4=bh;bs=a4*a4-ag;a0=2*a4-ac;if MathUtil.isZero(bs)then bs=0 elseif bs>0 then bs=math.sqrt(bs)else return end;if MathUtil.isZero(a0)then a0=0 elseif a0>0 then a0=math.sqrt(a0)else return end;coeffs[2]=a4-bs;coeffs[1]=bj<0 and-a0 or a0;coeffs[0]=1;do local bw={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bo=#bw;bh,bi=bw[1],bw[2]end;coeffs[2]=a4+bs;coeffs[1]=bj<0 and a0 or-a0;coeffs[0]=1;if bo==0 then local bw={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bo=bo+#bw;bh,bi=bw[1],bw[2]end;if bo==1 then local bw={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bo=bo+#bw;bi,bn=bw[1],bw[2]end;if bo==2 then local bw={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bo=bo+#bw;bn,bv=bw[1],bw[2]end end;bp=0.25*ai;if bo>0 then bh=bh-bp end;if bo>1 then bi=bi-bp end;if bo>2 then bn=bn-bp end;if bo>3 then bv=bv-bp end;return bh,bi,bn,bv end;function RingBuffer.RingBuffer(by)local bz={}bz.buf={}bz.capacity=by;bz.size=0;bz.head=1;local bb=getmetatable(bz)or{}bb.__index=RingBuffer.get;setmetatable(bz,bb)return bz end;function RingBuffer.isFull(bz)return bz.size>=bz.capacity end;function RingBuffer.push(bz,d)bz.buf[(bz.head+bz.size-1)%bz.capacity+1]=d;if bz.size==bz.capacity then bz.head=bz.head%bz.capacity+1 else bz.size=bz.size+1 end end;function RingBuffer.pop(bz)if bz.size==0 then return nil end;local C=bz.buf[bz.head]bz.buf[bz.head]=nil;bz.head=bz.head%bz.capacity+1;bz.size=bz.size-1;return C end;function RingBuffer.get(bz,p)if type(p)~="number"or math.floor(p)~=p then return nil end;if p<1 or p>bz.size then return nil end;return bz.buf[(bz.head+p-2)%bz.capacity+1]end;function InterpolatedSearch(bA,bB,t,u,bC,bD,bE)bE=bE or 50;local m,n,bF;local bG=0;while u>t do m=bB[t]if m==bC then return t end;if m>bC then return bD and t or nil end;n=bB[u]if n==bC then return u end;if n<bC then return bD and u or nil end;bF=math.floor((bC-m)/(n-m)*(u-t)+t)bF=math.min(math.max(bF,t+1),u-1)if bB[bF]==bC then return bF end;if bC<bB[bF]then if bD and math.abs(bB[bF-1]-bC)>math.abs(bB[bF]-bC)then return bF end;u=bF-1 else if bD and math.abs(bB[bF+1]-bC)>math.abs(bB[bF]-bC)then return bF end;t=bF+1 end;bG=bG+1;if bG>bE then break end end;return bD and t or nil end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(m,n)local bH=type(m)=="number"local bI=type(n)=="number"if not bH and bI then return n+m end;if bH and not bI then return MathUtil.combine(m,n,function(_,aF,bJ)return m+bJ end)else return MathUtil.combine(m,n,function(_,aF,bJ)return aF+bJ end)end end;VectorN.mt.__sub=function(m,n)return m+-n end;VectorN.mt.__mul=function(m,n)local bH=type(m)=="number"local bI=type(n)=="number"if not bH and bI then return n*m end;if bH and not bI then local z={}for _,a0 in pairs(n)do z[_]=m*a0 end;return z else return MathUtil.combine(m,n,function(_,aF,bJ)return aF*bJ end)end end;VectorN.mt.__div=function(m,n)local bH=type(m)=="number"local bI=type(n)=="number"if not bH and bI then return m*1/n end;if bH and not bI then local z={}for _,a0 in pairs(n)do z[_]=m/a0 end;return z else return MathUtil.combine(m,n,function(_,aF,bJ)return aF/bJ end)end end;VectorN.mt.__unm=function(m)local z={}for _,a0 in pairs(m)do z[_]=-a0 end;return z end;function VectorN.VectorN(B)local bK={}for _,a0 in pairs(B)do if type(a0)=="table"then bK[_]=VectorN.VectorN(a0)else bK[_]=a0 end end;setmetatable(bK,VectorN.mt)return bK end;function Control.PID(bL,bM,bN,bO,bP,bQ)local bR={}bR.kP=bL;bR.kI=bM;bR.kD=bN;bR.Iacc=Accumulator.Accumulator(bO,bP)if bQ and bQ~=0 then bR.period=bQ end;return bR end;function Control.processPID(bS,bT,e)bT=bS.period and(bT+bS.period/2)%bS.period-bS.period/2 or bT;local ac=bS.kP*bT;local F,bU=bS.kI*Accumulator.update(bS.Iacc,bT,e)F=F/bU;local h=bS.kD*(bT-(bS.lastError or bT))/e;bS.lastError=bT;return ac+F+h end;function Control.FF(coeffs,bQ)local bV={}bV.coeffs=coeffs;bV.degree=#coeffs-1;if bQ and bQ~=0 then bV.period=bQ end;bV.derivs={}return bV end;function Control.processFF(bS,bC,e)local bW=0*bC;local bX=bC;local bY=bC;for F=1,bS.degree+1 do bY=bS.derivs[F]bS.derivs[F]=bX;bW=bW+bS.coeffs[F]*bX;if bY then local aT=bX-bY;if F==1 and bS.period then aT=(aT+bS.period/2)%bS.period-bS.period/2 end;bX=aT/e else break end end;return bW end;function Nav.toLocal(bZ,b_,c0)local c1=bZ-b_;return Quaternion.Inverse(c0)*c1 end;function Nav.toGlobal(c2,b_,c0)local c1=c0*c2;return c1+b_ end;function Nav.cartToPol(c3)local ag=c3.magnitude;local ah=Vector3.SignedAngle(Vector3.forward,c3,Vector3.up)local bt=90-Vector3.Angle(Vector3.up,c3)return Vector3(ag,ah,bt)end;function Nav.cartToCyl(c3)local c4=Vector3(c3.x,0,c3.z)local c5=c4.magnitude;local bt=Vector3.SignedAngle(Vector3.forward,c3,Vector3.up)local a4=c3.y;return Vector3(c5,bt,a4)end;function Nav.polToCart(c3)local ag,ah,bt=c3.x,c3.y,c3.z;local aF=Mathf.Sin(ah)*Mathf.Cos(bt)local bJ=Mathf.Sin(bt)local a4=Mathf.Cos(ah)*Mathf.Cos(bt)return ag*Vector3(aF,bJ,a4)end;function Nav.cylToCart(c3)local c5,bt,c6=c3.x,c3.y,c3.z;local aF=c5*Mathf.Sin(bt)local bJ=c6;local a4=c5*Mathf.Cos(bt)return Vector3(aF,bJ,a4)end;function Targeting.firstOrderTargeting(c7,c8,c9)local ca=c7-Vector3.Project(c7,c8)local cb=Vector3.Dot(c8,c7-ca)/c8.sqrMagnitude;local m,n=MathUtil.solveQuadratic(cb-c9*c9,2*cb,ca.sqrMagnitude+cb*cb)local cc=nil;if m and m>=0 then cc=m end;if n and n>=0 and n<m then cc=n end;return cc and(c7+cc*c8).normalized or nil end;function Targeting.secondOrderTargeting(c7,cd,ce,c9,cf,cg)local m=-0.25*ce.sqrMagnitude;local n=-Vector3.Dot(cd,ce)local Q=-(cd.sqrMagnitude-c9*c9+Vector3.Dot(c7,ce))local h=-2*Vector3.Dot(c7,cd)local bT=-c7.sqrMagnitude;local g;local ch=ce.magnitude;local ci=cd.magnitude;local cj=c7.magnitude;local ck,cl=MathUtil.solveQuadratic(0.5*ch,ci+c9,-cj)local cm=math.max(ck,cl)local cn;local coeffs={0.5*ch,ci-c9,cj}if MathUtil.ruleOfSigns(coeffs,0)==2 then local co,cp=MathUtil.solveQuadratic(coeffs[1],coeffs[2],coeffs[3])if co then cn=math.min(co,cp)end end;if not cn or cn<cm then local bh,bi,bn=MathUtil.solveCubic(4*m,3*n,2*Q,h)if not bn then if bh>cm then cn=bh end else local co=math.min(bh,bn)local cp=math.max(bh,bn)if co>cm then cn=co elseif cp>cm then cn=cp end end;if not cn then return end end;local function cq(aF)return bT+aF*(h+aF*(Q+aF*(n+aF*m)))end;g=MathUtil.ITP(cq,cm,cn,1e-4,25)if not g then return end;if g>=cm and g<=cn then local cr=c7+cd*g+0.5*ce*g*g;if cr.sqrMagnitude>=cf*cf and cr.sqrMagnitude<=cg*cg then return cr,g end end end;function Targeting.AIPPN(cs,c7,ct,c8,cu)local cd=c8-ct;local cv=Vector3.Dot(-cd,c7.normalized)if cv<=0 then cv=10 end;local cw=c7.magnitude/cv;local cx=Vector3.Cross(c7,cd)/c7.sqrMagnitude;local cy=Vector3.Cross(c7,cu)/c7.sqrMagnitude*cw/2;local cz=cx+cy;local cA=Vector3.Cross(cz,c7.normalized)local cB=Vector3.ProjectOnPlane(cA,ct).normalized;local cC=cs*ct.magnitude*cz.magnitude;return cC*cB end;function Targeting.ATPN(cs,c7,ct,c8,cu)local cd=c8-ct;local cv=-Vector3.Dot(cd,c7.normalized)if cv<=0 then cv=10 end;local cx=Vector3.Cross(c7,cd)/c7.sqrMagnitude;local cA=Vector3.Cross(cx,c7.normalized)local cD=Vector3.ProjectOnPlane(cu,c7)return cs*cv*cA+0.5*cs*cu end;function BlockUtil.getWeaponsByName(bA,cE,ba,cF)if DEBUG then bA:Log("searching for "..cE)end;local cG=bA:GetAllSubConstructs()local cH={}ba=ba or-1;local Q=ba;if not cF or cF==0 or cF==2 then for F=0,bA:GetWeaponCount()-1 do if Q==0 then break end;if bA:GetWeaponBlockInfo(F).CustomName==cE then table.insert(cH,{subIdx=nil,wpnIdx=F})if DEBUG then bA:Log("found weapon "..cE.." on hull, type "..bA:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end;if not cF or cF==1 or cF==2 then for p=1,#cG do local bp=cG[p]for F=0,bA:GetWeaponCountOnSubConstruct(bp)-1 do if Q==0 then break end;if bA:GetWeaponBlockInfoOnSubConstruct(bp,F).CustomName==cE then table.insert(cH,{subIdx=bp,wpnIdx=F})if DEBUG then bA:Log("found weapon "..cE.." on subobj "..bp..", type "..bA:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end end;if DEBUG then bA:Log("weapon count: "..#cH)end;return cH end;function BlockUtil.getSubConstructsByName(bA,cE,ba)if DEBUG then bA:Log("searching for "..cE)end;local cG=bA:GetAllSubConstructs()local cI={}ba=ba or-1;local Q=ba;for p=1,#cG do local bp=cG[p]if Q==0 then break end;if bA:GetSubConstructInfo(bp).CustomName==cE then table.insert(cI,bp)if DEBUG then bA:Log("found subobj "..cE)end;Q=Q-1 end end;if DEBUG then bA:Log("subobj count: "..#cI)end;return cI end;function BlockUtil.getBlocksByName(bA,cE,type,ba)if DEBUG then bA:Log("searching for "..cE)end;local cJ={}ba=ba or-1;local Q=ba;for p=0,bA:Component_GetCount(type)-1 do if Q==0 then break end;if bA:Component_GetBlockInfo(type,p).CustomName==cE then table.insert(cJ,p)if DEBUG then bA:Log("found component "..cE)end;Q=Q-1 end end;if DEBUG then bA:Log("component count: "..#cJ)end;return cJ end;function BlockUtil.getWeaponInfo(bA,cK)if cK.subIdx then return bA:GetWeaponInfoOnSubConstruct(cK.subIdx,cK.wpnIdx)end;return bA:GetWeaponInfo(cK.wpnIdx)end;function BlockUtil.getWeaponBlockInfo(bA,cK)if cK.subIdx then return bA:GetWeaponBlockInfoOnSubConstruct(cK.subIdx,cK.wpnIdx)end;return bA:GetWeaponBlockInfo(cK.wpnIdx)end;function BlockUtil.aimWeapon(bA,cK,cL,cM)if cK.subIdx then bA:AimWeaponInDirectionOnSubConstruct(cK.subIdx,cK.wpnIdx,cL.x,cL.y,cL.z,cM)else bA:AimWeaponInDirection(cK.wpnIdx,cL.x,cL.y,cL.z,cM)end end;function BlockUtil.fireWeapon(bA,cK,cM)if cK.subIdx then return bA:FireWeaponOnSubConstruct(cK.subIdx,cK.wpnIdx,cM)end;return bA:FireWeapon(cK.wpnIdx,cM)end;function Combat.pickTarget(bA,cN,cO)cO=cO or function(U,cP)return cP.Priority end;local bC,cQ;for F in MathUtil.range(bA:GetNumberOfTargets(cN))do local cP=bA:GetTargetInfo(cN,F)local cR=cO(bA,cP)if not bC or cR>cQ then bC=cP;cQ=cR end end;return bC end;function Combat.CheckConstraints(bA,cS,cT,cU)local cV;if cU then cV=bA:GetWeaponConstraintsOnSubConstruct(cU,cT)else cV=bA:GetWeaponConstraints(cT)end;local cW=bA:GetConstructForwardVector()local cX=bA:GetConstructUpVector()local cY=Quaternion.LookRotation(cW,cX)cS=Quaternion.Inverse(cY)*cS;if cV.InParentConstructSpace and cU then local cZ=bA:GetSubConstructInfo(cU).localRotation;cS=Quaternion.inverse(cZ)*cS end;local c_=MathUtil.angleOnPlane(Vector3.forward,cS,Vector3.up)local d0=cS;d0.z=0;local O=Mathf.Atan2(cS.z,d0.magnitude)local d1=c_>cV.MinAzimuth and c_<cV.MaxAzimuth;local d2=O>cV.MinElevation and O<cV.MaxElevation;if cV.FlipAzimuth then d1=not d1 end;if d1 and d2 then return true end;c_=c_+180;O=180-O;if O>180 then O=O-360 end;if O<-180 then O=O+360 end;d1=c_>cV.MinAzimuth and c_<cV.MaxAzimuth;d2=O>cV.MinElevation and O<cV.MaxElevation;if cV.FlipAzimuth then d1=not d1 end;if d1 and d2 then return true end;return false end;function StringUtil.LogVector(bA,bK,d3)bA:Log(d3 .."("..bK.x..", "..bK.y..", "..bK.z..")")end
