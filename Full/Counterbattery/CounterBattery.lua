  -- Settings
  -- Each redundant Lua box should have a unique, positive luaIdx (i.e. 1, 2, 3, ...). They will come online if all lower index Lua boxes are destroyed.
  local luaIdx = 1

  -- use pCall to avoid crash, retry if failed
  local AllowErrorRecovery = false

  -- scaling for axes in projectile avoidance routine used to detect projectiles
    -- projectile avoidance should output value / scale into A, B, and D axes
  local altScale = 800
  local bearScale = 180
  local distScale = 800

  -- seconds to record target movements for
  local targetTrackTime = 15
  -- number of locations to track per enemy (todo: support tracking multiple projectiles simultaneously)
  local numOrigins = 3
  -- time between switching targets
  local originSwitchTime = 2
  -- maximum time to remember origin points
  local maxStaleness = 6
  -- altitudes to engage (estimated altitude of firing piece, not target position)
    -- checked when adding an origin
  local minAlt = 0
  local maxAlt = math.huge
  -- parameters to find weapons (Shift+N to name turret blocks)
    -- different turrets should have different names even if they have the same weapon
    -- both turrets and firing pieces should be named
    -- firing pieces can share the name of the turret they are mounted on
    -- if weapons on the same turret have different muzzle velocities, name them differently from each other
      -- only one should have the same name as the turret, usually the one with the narrowest aiming arc
  local weaponDef = {
    {
      name = "laser",
      -- Normally reads muzzle velocity from weapon info. Set a value here to override it, i.e. if you have multiple firing pieces on the same turret
      -- velocity = math.huge,
      minRange = 50,
      maxRange = 2000,
      -- per-weapon altitude settings checked when firing, not when adding origins
      minAlt = 0,
      maxAlt = math.huge,
    }
  }
  -- checkACB lists ACBs set to detect object presence and trigger custom axis
    -- will be checked before firing to make sure blocks actually exist at target distance
    -- this check is not performed when firing at aimpoint due to idle mode "fire" or "timer"
    -- may also have second ACB with inverted settings that trigger negative custom axis, in which case set inverted to true
  local checkACBs = {
    { axis = "check", minRange = 200, maxRange = 2400, turretName = "checkTurret", offset = Vector3(0, 1, 0), inverted = false },
  }
  -- checkACBRequire is required ratio of sum of axis values to number of valid ACBs 
  local checkACBRequire = 1
  -- validACBRequire is required number of ACBs in range and pointing in the right direction
  local validACBRequire = 0

  -- custom axis containing stability value (need breadboard to get this value)
  local stabilityAxisName = "Stability"
  -- will fire if any are true, set to 0 to ignore
  -- will fire if stability exceeds this value
  local minStabilityToFire = 0
  -- will fire if stability exceeds the average stability
    -- calculated by an exponential filter with time constant of stabilityAvgTime
  local stabilityAvgTime = 3
  -- will fire if stability is the highest its been in this many seconds
  local stabilityMaxTime = 1

  -- indexes of mainframes to be used to track enemy rotations
    -- 3 is the bare minimum, but will fail if any of them target a subconstruct
    -- so more is preferred
    -- it will also fail if too many mainframes switch their aimpoint simultaneously
    -- this is unavoidable in the case of blocks being destroyed, but can be avoided
    -- in the case of the timer switching blocks.

  -- todo: track aimpoint switching time
    -- invalidate track if aimpoint switching time has passed
    -- if aimpoint switching time set to allow script to control it,
      -- adjust switching time to avoid multiple mainframes switching at once
  local aimPointTrackers = {
    { idx = 0 },
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
        origin - if isCounter is true, then the original origin, a table containing:
            position - position of firing point relative to enemy position
            time - estimated time at which projectile was fired,
            relVel - velocity of projectile relative to enemy at firing time,
            detectPos - position at which the projectile was detected,
            detectTime - the time at which the projectile was detected
            type - 0 for APS, 1 for CRAM, 2 for missile
          otherwise just the aimpoint position relative to the enemy position
        targetPos - the target position
        targetVel - the target velocity
        targetRot - the target rotation relative to when the target was first detected
        isCounter - whether we are firing at an origin detected by the counterbattery script or just the AI aimpoint
      Returns:
        fpOffset - the offset of the adjusted aimpoint relative to the original aimpoint
    ]]
    -- to adjust depending on estimated turret location
  -- todo: return list of locations to attempt, using the checkACBs to check each one
  local function aimOffset(origin, targetPos, targetVel, targetRot, isCounter)
    return Vector3.zero
    -- example: against APS, try to break tetris, against CRAM, try to hit firing piece (assumes barrel is about 8m long)
    --  against AI aimpoint or missile launcher, does not modify aimpoint
    -- if isCounter then
    --   local ofs = Vector3.zero
    --   if origin.type == 0 then
    --     ofs = Vector3(0, -7, 0)
    --   elseif origin.type == 1 then
    --     local rot = targetRot * Quaternion.Inverse(origin.rot)
    --     ofs = -8 * (rot * origin.relVel.normalized)
    --     ofs.y = ofs.y - 1.5
    --   end
    --   ofs.y = math.max(ofs.y, minAlt - (targetPos.y + origin.position.y) + 0.1)
    --   return ofs
    -- end
    -- return Vector3.zero
  end
  -- when tracking rotation, reject any rotation estimates above this rate (in deg/s)
  local maxTurnRateTracking = 720
  -- whether or not to attempt to snipe missile launchers
    -- since missiles can turn, we can't trace back their trajectory
    -- however, if we detect a missile the instant it is launched (most feasible for huge missiles
      -- and large missiles with low ramp time, no ignition delay, and high thrust),
    -- we can approximate the missile as traveling in a straight line
  local missileCounter = true
  -- uses assumptions of 240/285/300m/s initial velocity and conservation of energy
    -- to more accurately determine CRAM launch point
    -- only reliable with automatic detection.
  local cramCounter = false
  -- degrees of inaccuracy allowed for ACB object checking
    -- NOT used for firing weapons because weaponInfo.CurrentDirection is not accurate
  local AIM_TOL = 0.1
  -- physics ticks per second (Lua runs in sync with game physics)
  local TICKS_PER_S = 40

  -- todo: Integrate with enemy identififcation script to get weapon distances from origin,
    -- and trace to that distance from origin instead of closest
  -- todo: Account for target acceleration

  -- Stored Variables
  -- one way to store previous values is in local variables outside of Update like this
    -- another way is to use global variables, which has the benefit of being able to be
    -- located near where they are used, but are much slower to access (requires a table lookup)
  local inited = false

  local continueLine = false
  local currentLine = {}
  local currentTargetId
  local enemies = {}
  local foundMissiles = {}
  local frameTime
  local lastAim
  local lastFrameTime
  local lastOrigin
  local lastOriginSwitchTime = 0
  local lastProjectilePos
  local nextRecordTime
  local originPopTime = 0
  local prevTime
  local stabilityAvg = 1
  local stabilityWindow
  local t
  local trackLossTime = 0
  local turrets = {}

  -- Tides header
  local BlockUtil = {}
  local Combat = {}
  local StringUtil = {}
  local Accumulator = {}
  local Differ = {}
  local Graph = {}
  local Heapq = {}
  local LinkedList = {}
  local MathUtil = {}
  local Matrix3 = {}
  local RingBuffer = {}
  local Search = {}
  local Stats = {}
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
    if stabilityMaxTime > 0 then
      stabilityWindow = LinkedList.LinkedList()
    end
    math.randomseed(I:GetTime())
    math.random()
    math.random()

    originPopTime = I:GetTimeSinceSpawn()
    inited = true
  end

  function Update(I)
    -- I:ClearLogs()
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
      return false -- This means we had an error, so just move on in the LUA.
    else
      I:Log("Ran update")
    end
  end

  function CoreUpdate(I)
    if not inited then
      Init(I)
    end
    -- Redundant Lua system. Prevents this box from running until higher priority boxes have all been destroyed.
    I:RequestCustomAxis("luaActive", 2 ^ (-luaIdx))
    if I:GetCustomAxis("luaActive") > 2 ^ (1 - luaIdx) then
        return
    end
    t = I:GetTimeSinceSpawn()
    frameTime = lastFrameTime and t - lastFrameTime or 0
    lastFrameTime = t

    local target = I:GetTargetInfo(mainframeIdx, 0)
    if not target or not target.Valid then
      return
    end

    if t >= nextRecordTime - 0.5 / TICKS_PER_S then
      UpdateEnemyData(I)
      local stability = I:GetCustomAxis(stabilityAxisName)
      if stability then
        CheckStability(stability)
      end
      nextRecordTime = nextRecordTime + 1 / TICKS_PER_S
    end
    currentTargetId = target.Id

    local enemy = enemies[currentTargetId]
    if not enemy then return end

    local alt = altScale * I:GetPropulsionRequest(9) -- A axis, set in projectile avoidance routine
    local relBear = bearScale * I:GetPropulsionRequest(10) -- B axis (is this guaranteed to be -180 to 180?)
    local dist = distScale * I:GetPropulsionRequest(12) -- D axis (maybe increase to account for large/huge missiles)
    local projectile = GetProjectile(I, alt, relBear, dist)

    if not projectile then
      currentLine = {}
      lastProjectilePos = nil
    end

    if missileCounter then
      local origins = GetMissileOrigin(I, enemy)
      if origins then
        for i, origin in ipairs(origins) do
          RingBuffer.push(enemies[target.Id].origins, origin)
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
    if lastProjectilePos and CheckAndUpdateLine(I, currentLine, projectile, eps) then
      if not continueLine then
        local origin = GetOrigin(I, currentLine, enemy)
        if origin then
          RingBuffer.push(enemy.origins, origin)
        end
      end
      continueLine = true
    else
      -- todo: maybe get origin again when line ends. Not necessary currently since munition detection is perfect
      continueLine = false
    end

    lastProjectilePos = projectile

    -- fire weapon at origins
    if I:GetAIFiringMode(mainframeIdx) == "off" then return end
    local origin
    if lastOriginSwitchTime + originSwitchTime > t then
      origin = lastOrigin
    end
    if not origin then
      origin = PickTargetOrigin(I, enemy)
      lastOriginSwitchTime = t
      lastOrigin = origin
    end

    local fp
    local eRot = enemy.rotation[enemy.rotation.size] or Quaternion.identity
    if origin then
      I:Log("Aiming at origin, type "..origin.type)
      fp = eRot * origin.position + aimOffset(origin, target.Position, target.Velocity, eRot, true)
      local acbPass = PerformACBCheck(I, fp, target)
      if not acbPass then
        origin = nil
        fp = nil
      end
    end
    if not fp then
      I:Log("Aiming at AI aimpoint")
      fp = target.AimPointPosition - target.Position
      fp = fp + aimOffset(fp, target.Position, target.Velocity, eRot, false)
    end

    local ready = origin ~= nil
    FireAtFirepoint(I, fp, target, ready)
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
          local newEnemy = {
                              pos = RingBuffer.RingBuffer(rbsize),
                              vel = RingBuffer.RingBuffer(rbsize),
                              rotation = RingBuffer.RingBuffer(rbsize),
                              origins = RingBuffer.RingBuffer(numOrigins),
                              density = Stats.Distribution({"x", "y", "z"}),
                              valid = true,
                              oldAimpoints = {},
                              aimpoints = {},
                            }
          newEnemy.density.n = 15
          -- todo: set initial size estimate based on block count
          newEnemy.density.cov = Matrix3.scalarmul(Matrix3.Identity(), 2500)
          enemies[target.Id] = newEnemy
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
      local dRot, consensusPoints = GetRotation(en.oldAimpoints, en.aimpoints)
      if dRot then
        RingBuffer.push(en.rotation, dRot * (en.rotation[en.rotation.size] or Quaternion.identity))
        local changedAimpoints = {}
        local nTrack = #en.aimpoints
        local refIdx = consensusPoints[1]
        local invRot = Quaternion.Inverse(en.rotation[en.rotation.size])
        for i=1, nTrack do
          local predicted = dRot * (en.oldAimpoints[i] - en.oldAimpoints[refIdx]) + en.aimpoints[refIdx]
          if (predicted - en.aimpoints[i]).sqrMagnitude > 3 * 3 then
            table.insert(changedAimpoints, invRot * (en.aimpoints[i] - en.pos[en.pos.size]))
          end
        end
        -- Only update distribution if aimpoint has changed
        Stats.updateDistributionBatched(en.density, changedAimpoints)
        -- reduce n so distribution is allowed to change faster
        en.density.n = math.min(en.density.n, 50)
      else
        RingBuffer.push(en.rotation, en.rotation[en.rotation.size] or Quaternion.identity)
        trackLossTime = trackLossTime + 1 / TICKS_PER_S
      end
    end
  end

  function CheckStability(stability)
    if stabilityAvgTime > 0 then
      stabilityAvg = (1 - 1 / (TICKS_PER_S * stabilityAvgTime)) * stabilityAvg + stability / (TICKS_PER_S * stabilityAvgTime)
    end
    if stabilityMaxTime > 0 then
      while LinkedList.peekBack(stabilityWindow) and LinkedList.peekBack(stabilityWindow).s <= stability do
        LinkedList.popBack(stabilityWindow)
      end
      while LinkedList.peekFront(stabilityWindow) and LinkedList.peekFront(stabilityWindow).t < t - stabilityMaxTime do
        LinkedList.popFront(stabilityWindow)
      end
      LinkedList.pushBack(stabilityWindow, { s=stability, t=t } )
    end
  end

  function GetProjectile(I, alt, relBear, dist)
    if dist == 0 then
      return nil
    end
    local com = I:GetConstructCenterOfMass()
    local relAlt = alt - com.y
    dist = math.sqrt(dist * dist - relAlt * relAlt)
    local projectile = Vector3(0, alt, dist)
    projectile = Quaternion.AngleAxis(I:GetConstructYaw() + relBear, Vector3.up) * projectile
    projectile = projectile + com
    projectile.y = alt

    return projectile
  end

  function GetOrigin(I, line, enemy)
    if prevTime and RingBuffer.isFull(enemy.pos) then
      prevTime = prevTime - 1
    end
    local relVel = line.ds / line.dt - Vector3(0, line.dv, 0) - enemy.vel[enemy.vel.size]
    local relPos = line.ed - enemy.pos[enemy.pos.size]
    local time2d = math.sqrt((relPos.x ^ 2 + relPos.z ^ 2) / (relVel.x ^ 2 + relVel.z ^2))
    local tIdx2d = enemy.pos.size - math.floor(time2d * TICKS_PER_S + 0.5)
    local g = I:GetGravityForAltitude(currentLine.start.y)
    local estimate2d = relPos - (relVel * time2d) + 0.5 * g * time2d * time2d
    if tIdx2d > 0 then
      estimate2d = Quaternion.Inverse(enemy.rotation[tIdx2d]) * estimate2d
    end
    local covInv = Matrix3.inverse(enemy.density.cov)
    if Matrix3.quadform(estimate2d, covInv, estimate2d) < 5 * 5 then
      local closest, closestTimeIdx = RunTrace(I, line, enemy, prevTime)
      if not closest then
        prevTime = nil
        return
      end
      prevTime = closestTimeIdx
      local launchPos, launchTime
      if cramCounter then
        launchPos, launchTime = CheckIfCram(I, line, enemy, closestTimeIdx)
      end
      if launchPos then
        I:Log(string.format("Projectile determined to be CRAM shell, fired %f seconds ago", launchTime))
        local pIdx = enemy.pos.size + math.floor(launchTime * TICKS_PER_S)
        if pIdx < 1 or pIdx > enemy.pos.size then
          I:Log("CRAM guess has no target data")
          return
        end
        local enemyPosAtTime = enemy.pos[pIdx]
        local enemyRotAtTime = enemy.rotation[pIdx]
        local launchAlt = (enemyPosAtTime + launchPos).y
        if launchAlt > minAlt and launchAlt < maxAlt then
          local origin = {
            position = Quaternion.Inverse(enemyRotAtTime) * launchPos,
            time = t + launchTime,
            relVel = line.ev * TICKS_PER_S + g * launchTime - enemy.vel[pIdx],
            rot = enemyRotAtTime,
            detectTime = t,
            detectPos = line.ed,
            type = 1,
          }
          return origin
        end
      end
      local eRot = enemy.rotation[closestTimeIdx]
      local closestRotated = Quaternion.Inverse(eRot) * closest
      if Matrix3.quadform(closestRotated, covInv, closestRotated) < 4 * 4 then
        local closestAlt = closest.y + enemy.pos[closestTimeIdx].y
        if closestAlt > minAlt and closestAlt < maxAlt then
          local origin = {
            position = closestRotated,
            time = t + (closestTimeIdx - enemy.pos.size) / TICKS_PER_S,
            relVel = line.ev / frameTime + g * t - enemy.vel[closestTimeIdx],
            rot = eRot,
            detectTime = t,
            detectPos = line.ed,
            type = 0,
          }
          return origin
        end
      end
    end
  end

  function GetMissileOrigin(I, enemy)
    local origins = {}
    for widx = 0, I:GetNumberOfWarnings(mainframeIdx) - 1 do
      local warn = I:GetMissileWarning(mainframeIdx, widx)
      if warn.Valid and not foundMissiles[warn.Id] and warn.TimeSinceLaunch < 0.1 then
        foundMissiles[warn.Id] = 1
        local launcherPos = warn.Position - warn.TimeSinceLaunch * warn.Velocity
        local tIdx = enemy.pos.size - warn.TimeSinceLaunch * TICKS_PER_S
        local targetPosAtLaunch = enemy.pos[tIdx]
        -- todo: set threshold by target size
        local diff = Quaternion.Inverse(enemy.rotation[tIdx]) * (launcherPos - targetPosAtLaunch)
        if Matrix3.quadform(diff, Matrix3.inverse(enemy.density.cov), diff) < 4 * 4 then
          local closestAlt = launcherPos.y
          if closestAlt > minAlt and closestAlt < maxAlt then
            local lPos = launcherPos - targetPosAtLaunch
            local origin = {
              position = lPos,
              time = t - warn.TimeSinceLaunch,
              relVel = warn.Velocity - enemy.vel[enemy.vel.size],
              rot = eRot,
              detectTime = t,
              detectPos = warn.Position,
              type = 2,
            }
            table.insert(origins, origin)
          end
        end
      end
    end
    return origins
  end

  function GetRotation(oldPts, newPts, iterLim)
    local nTrack = #newPts
    iterLim = iterLim or 25

    local indices = {}
    for i=1, nTrack do
      indices[i] = i
    end
    local consensusPoints = {}
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
              table.insert(consensusPoints, idx)
            end
            if 2 * votes - 3 >= nTrack then
              return deltaRot, consensusPoints
            end
            consensusPoints = {}
          end
        end
      end
    end
    return nil
  end

  function CheckIfCram(I, line, enemy, closestTimeIdx)
    local projPos = line.ed
    local projVel = line.ev * TICKS_PER_S
    local bestD, bestS, bestR
    local bestScoreD = -math.huge
    local bestScoreS = -math.huge
    local bestScoreR = -math.huge
    local t1, t2 = MathUtil.solveQuadratic(-9.81, projVel.y, projPos.y)
    if not t1 then return end
    for i = math.max(closestTimeIdx - 10, math.floor(t2 * TICKS_PER_S) + enemy.vel.size, 1), math.min(closestTimeIdx + 10, math.floor(t1 * TICKS_PER_S) + enemy.vel.size, enemy.vel.size) do
      if enemy.vel[i] then
        local iTime = (i - enemy.vel.size) / TICKS_PER_S
        local projPosAtTime = projPos + projVel * iTime - Vector3(0, 0.5 * 9.81 * iTime * iTime, 0)
        local relVelAtClosest = projVel
        relVelAtClosest = relVelAtClosest - enemy.vel[i]
        -- this line goes after because we don't want to modify the original projVel
        relVelAtClosest.y = relVelAtClosest.y - 9.81 * iTime
        local dScore = -1000 * math.abs(relVelAtClosest.magnitude - 300) - 0.01 * (projPosAtTime - enemy.pos[i]).magnitude
        local sScore = -1000 * math.abs(relVelAtClosest.magnitude - 240) - 0.01 * (projPosAtTime - enemy.pos[i]).magnitude
        local rScore = -1000 * math.abs(relVelAtClosest.magnitude - 285) - 0.01 * (projPosAtTime - enemy.pos[i]).magnitude
        if dScore > bestScoreD then
          bestD = i
          bestScoreD = dScore
        end
        if sScore > bestScoreS then
          bestS = i
          bestScoreS = sScore
        end
        if rScore > bestScoreR then
          bestR = i
          bestScoreR = rScore
        end
      end
    end
    local idx
    if bestScoreD > -3000 and bestScoreD > bestScoreS and bestScoreD > bestScoreR then
      idx = bestD
    elseif bestScoreS > -3000 and bestScoreS > bestScoreR then
      idx = bestS
    elseif bestScoreR > -3000 then
      idx = bestR
    else
      return
    end
    idx = math.max(idx - 1, 1)
    local launchTime = (idx - enemy.vel.size) / TICKS_PER_S
    local projPosAtTime = projPos + projVel * launchTime - Vector3(0, 0.5 * 9.81 * launchTime * launchTime, 0)
    local p = projPosAtTime - enemy.pos[idx]
    return p, launchTime
  end

  function PickTargetOrigin(I, enemy)
    local origin
    local eRot = enemy.rotation[enemy.rotation.size] or Quaternion.identity
    while enemy.origins.size > 0 and t - enemy.origins[1].detectTime > maxStaleness do
      RingBuffer.pop(enemy.origins)
      originPopTime = t
    end
    if enemy.origins.size == 0 then
      I:Log("No origins found.")
    else
      local candidate = enemy.origins[math.random(1, enemy.origins.size)]
      local originAlt = (eRot * candidate.position).y + enemy.pos[enemy.pos.size].y
      if originAlt < minAlt or originAlt > maxAlt then
        -- do a linear search to find valid origin
        for i = 1, enemy.origins.size do
          candidate = enemy.origins[i]
          originAlt = (eRot * candidate.position).y + enemy.pos[enemy.pos.size].y
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
    return origin
  end

  function PerformACBCheck(I, fp, target)
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
        if val ~= 0 or not acb.inverted then
          score = score + I:GetCustomAxis(acb.axis)
          valid = valid + 1
        end
      end
    end
    if valid < validACBRequire or score < checkACBRequire * valid then
      I:Log(string.format("Origin ACB check failed: score %.2f of %.2f required with %d valid ACBs of %d required", score, checkACBRequire * valid, valid, validACBRequire))
      return false
    else
      I:Log("Origin ACB check passed")
      return true
    end
  end

  function FireAtFirepoint(I, fp, target, ready)
    if not ready and idleAim == "none" then return end
    local fpAlt = fp.y + target.Position.y
    for i, turret in ipairs(turrets) do
      if fpAlt >= weaponDef[i].minAlt and fpAlt <= weaponDef[i].maxAlt then
        for j, weapon in ipairs(turret) do
          local aim
          local fire = false
          local wInfo = BlockUtil.getWeaponInfo(I, weapon)
          local mv = weaponDef[i].velocity or (wInfo.Speed > 1e5 and math.huge or wInfo.Speed)
          if not ready and idleAim == "last" then
            aim = lastAim
          elseif not wInfo.PlayerCurrentlyControllingIt then
            if mv == math.huge then
              local range = (fp + target.Position - I:GetConstructPosition()).magnitude
              if range > (weaponDef[i].minRange or 0) and range < (weaponDef[i].maxRange or math.huge) then
                aim = fp + target.Position - wInfo.GlobalFirePoint
                fire = ready
              end
            else
              local g = 0.5 * (I:GetGravityForAltitude(I:GetConstructPosition().y) + I:GetGravityForAltitude(target.Position.y))
              aim, interceptTime = Targeting.secondOrderTargeting(fp + target.Position - wInfo.GlobalFirePoint,
                          target.Velocity - I:GetVelocityVector(),
                          -g,
                          mv, weaponDef[i].minRange, weaponDef[i].maxRange, weapon.lastT or nil)
              if aim then fire = ready end
              weapon.lastT = interceptTime
            end
          else
            I:Log("Holding fire for player.")
          end
          if aim then 
            ready = ready or (idleAim == "fire" or (idleAim == "timer" and t - originPopTime > waitTime))
          end
          if not aim then
            if idleAim == "enemy" or idleAim == "fire" or idleAim == "timer" then
              aim = target.AimPointPosition - wInfo.GlobalFirePoint
            end
          end
          if aim then
            lastAim = aim
            if Combat.CheckConstraints(I, aim, weapon.wpnIdx, weapon.subIdx) then
              BlockUtil.aimWeapon(I, weapon, aim, 0)
              if fire then
                local stability = I:GetCustomAxis(stabilityAxisName)
                if stability and ((minStabilityToFire >= 0 and stability >= minStabilityToFire) or stability >= stabilityAvg or stabilityWindow.next.next == stabilityWindow) then
                  -- angle tolerance needs to be pretty big because wInfo.CurrentDirection is unreliable for projectile weapons
                  -- hitscan weapons will not fire if they can't aim in the desired direction anyways
                  if Vector3.Angle(aim, wInfo.CurrentDirection) > 5 or not BlockUtil.fireWeapon(I, weapon, 0) then
                    I:Log("Holding fire for aim or reloading.")
                  else
                    I:Log("Fire!")
                  end
                else
                  I:Log("Holding fire for stability.")
                end
              end
            else
              I:Log("Holding fire for constraints.")
            end
          end
        end
      else
        I:Log("Holding fire for altitude.")
      end
    end
  end

  function CheckAndUpdateLine(I, line, projectile, tolerance)
    if line.start then
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
    end
    line.start = lastProjectilePos
    line.tStart = t - frameTime
    line.ed = projectile
    line.dv = -frameTime * I:GetGravityForAltitude(line.start.y).y
    line.dy = frameTime * line.dv
    line.ds = projectile + line.dy * Vector3.up - lastProjectilePos
    line.dt = frameTime
    line.ev = projectile - lastProjectilePos
    prevTime = nil
    return false
  end

  function RunTrace(I, line, enemy, timeGuess)
    local totalIter = 0
    local targetPos = enemy.pos[enemy.pos.size]
    local targetVel = enemy.vel[enemy.vel.size]
    if timeGuess then
      if enemy.pos[timeGuess] then
        targetPos = enemy.pos[timeGuess]
        targetVel = enemy.vel[timeGuess]
      else
        I:Log("Initial guess has no target data")
      end
    end
    local tIdxClosest = timeGuess or enemy.pos.size
    local cov = enemy.density.cov
    local covInv = Matrix3.inverse(cov)
    -- due to bug in FtD's Lua interpreter, inverse is sometimes negated or has columns that are circularly shifted one to the right
    if Matrix3.determinant(covInv) < 0 then
      covInv = -covInv
    end
    if Matrix3.get(covInv, 1, 2) ~= Matrix3.get(covInv, 2, 1) then
      for row=1,3 do
        local temp = covInv[(row - 1) * 3 + 1]
        for col=1,2 do
          covInv[(row - 1) * 3 + col] = covInv[(row - 1) * 3 + col + 1]
        end
        covInv[(row - 1) * 3 + 3] = temp
      end
    end
    local iters = tIdxClosest and 2 or 3
    for i = 1, iters do
      -- find the point of closest approach based on current target position and velocity
      -- x(t) = x_i + v_x t
      -- z(t) = z_i + v_z t
      -- y(t) = y_i + v_y t + 0.5gt^2
      -- squared distance = x^2 + y^2 + z^2

      -- sqr Mahalanobis distance = x^T S^-1 x = (x_0 + x'_0 t + 0.5 g t^2)^T S^-1 x
      -- x'(t) = v_i + g t

      -- d/dt sqrDistance = 
      -- 2 x_i v_x + 2 v_x^2 t +
      -- 2 z_i v_z + 2 v_z^2 t +
      -- 2 y_i v_y + 2 v_y^2 t + 2 y_i g t + 3 v_y g t^2 + g^2 t^3
      -- this is a cubic polynomial in terms of t which we can find the roots of

      -- d/dt sqrMaha = 2 x(t)^T S^-1 x'(t) = 2 (x_i + v_i t + 0.5g t^2) S^-1 (v_i + g t)
      -- = 2 (x_i S^-1 v_i + x_i S^-1 g t + v_i t S^-1 v_i + v_i t S^-1 g t + 0.5g t^2 S^-1 v_i + 0.5 g t^2 S^-1 g t)
      -- = 2 (x_i S^-1 v_1 + (x_i S^-1 g + v_i S^-1 v_i) t + (v_i S^-1 g + 0.5 g S^-1 v_i) t^2 + 0.5 g S^-1 g t^3)

      local ti = line.tStart - (t - (enemy.pos.size - tIdxClosest) / TICKS_PER_S) -- time from guess to line start
      local di = line.start - (targetPos + ti * targetVel)
      local projRelVel = line.ds / line.dt - targetVel
      -- accounting exactly for gravity changes over altitude is difficult, just approximate and hope the enemy isn't using mortars
      local g = I:GetGravityForAltitude(line.start.y)
      -- local a, b, c = MathUtil.solveCubic(0.5 * g * g, 1.5 * projRelVel.y * g, projRelVel.sqrMagnitude + di.y * g, Vector3.Dot(di, projRelVel))
      -- when covariance matrix is the identity, this matches the case without using the covariance matrix
      local q3 = 0.5 * Matrix3.quadform(g, covInv, g)
      local q2 = Matrix3.quadform(projRelVel, covInv, g) + 0.5 * Matrix3.quadform(g, covInv, projRelVel)
      local q1 = Matrix3.quadform(di, covInv, g) + Matrix3.quadform(projRelVel, covInv, projRelVel)
      local q0 = Matrix3.quadform(di, covInv, projRelVel)
      local a, b, c = MathUtil.solveCubic(q3, q2, q1, q0)
      -- critical point is a minimum when derivative changes from negative to positive
      -- since leading term is always nonnegative (covariance is always positive semidefinite), if there are three roots, the first and third are minima
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
      local approachTime = minRoot + line.tStart - t
      tIdxClosest = math.floor(approachTime * TICKS_PER_S + 0.5) + enemy.pos.size
      if not tIdxClosest then return nil end
      targetPos = enemy.pos[tIdxClosest]
      targetVel = enemy.vel[tIdxClosest]
      if not targetPos then
        I:Log(string.format("Solved cubic %f x^3 + %f x^2 + %f x + %f", q3, q2, q1, q0))
        I:Log(string.format("Iterated guess has no target data. Shell estimated to have been fired %f seconds ago", approachTime))
        return nil
      end
    end

    -- linear search to find best point
    -- todo: account for target velocity
    function CalcSqrMahaDist(tIdx)
      local dt = t - (enemy.pos.size - tIdx) / TICKS_PER_S - line.tStart
      local diff = line.start - enemy.pos[tIdx] + dt * line.ds / line.dt + 0.5 * I:GetGravityForAltitude(line.start.y) * dt * dt
      diff = Quaternion.Inverse(enemy.rotation[tIdx]) * diff
      return Matrix3.quadform(diff, covInv, diff)
    end
    local currentSqrDist = CalcSqrMahaDist(tIdxClosest)
    if tIdxClosest < enemy.pos.size then
      local aftSqrDist = CalcSqrMahaDist(tIdxClosest + 1)
      while aftSqrDist < currentSqrDist and tIdxClosest < enemy.pos.size do
        currentSqrDist = aftSqrDist
        tIdxClosest = tIdxClosest + 1
        if tIdxClosest < enemy.pos.size then
          aftSqrDist = CalcSqrMahaDist(tIdxClosest + 1)
        end
        totalIter = totalIter + 1
        if totalIter >= 100 then
          I:Log("Max iterations exceeded on upwards search")
          break
        end
      end
    end
    --I:Log(totalIter.." iterations after upwards search")
    if tIdxClosest > 1 then
      local befSqrDist = CalcSqrMahaDist(tIdxClosest - 1)
      while befSqrDist < currentSqrDist and tIdxClosest > 1 do
        currentSqrDist = befSqrDist
        tIdxClosest = tIdxClosest - 1
        if tIdxClosest > 1 then
          befSqrDist = CalcSqrMahaDist(tIdxClosest -1)
        end
        totalIter = totalIter + 1
        if totalIter >= 100 then
          I:Log("Max iterations exceeded on downwards search")
          break
        end
      end
    end
    --I:Log(totalIter.." iterations after downwards search")
    -- time of closest approach relative to line.tStart
    local dt = t - (enemy.pos.size - tIdxClosest) / TICKS_PER_S - line.tStart
    return line.start - enemy.pos[tIdxClosest] + dt * (line.ds / line.dt) + 0.5 * I:GetGravityForAltitude(line.start.y) * dt * dt, tIdxClosest
  end

  function SqrDistance(I, line, targetAbsPos, targetAbsVel, dt)
    local di = line.start - targetAbsPos
    local viRel = line.ds / line.dt - targetAbsVel
    local diff = di + dt * viRel + 0.5 * I:GetGravityForAltitude(line.start.y) * dt * dt
    return diff.sqrMagnitude
  end

  -- minified version of Tides library (not meant to be human-readable, see Tides.lua or individual class files for human-readable source)
  function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function Heapq.Heapq(i,k)local l={}l.data=i;l.comp=k or function(m,n)return m<n end;local o=#l.data;l.size=o;for p=math.floor(o/2),1,-1 do Heapq.siftDown(l,p)end;return l end;function Heapq.siftDown(l,q)local r=false;local s=q;local o=#l.data;while not r do r=true;local t=2*s;local u=2*s+1;local v=s;if t<=o and l.comp(l.data[t],l.data[v])then v=t;r=false end;if u<=o and l.comp(l.data[u],l.data[v])then v=u;r=false end;if not r then local w=l.data[v]l.data[v]=l.data[s]l.data[s]=w;s=v end end end;function Heapq.siftUp(l,q)local r=false;local s=q;while not r do r=true;local x=math.floor(s/2)if l.comp(l.data[s],l.data[x])then local w=l.data[x]l.data[x]=l.data[s]l.data[s]=w;s=x;r=false end end end;function Heapq.insert(l,y)l.data[l.size+1]=y;l.size=l.size+1;Heapq.siftUp(l,l.size)end;function Heapq.pop(l)local z=l.data[1]l.data[1]=l.data[l.size]l.data[l.size]=nil;l.size=l.size-1;Heapq.siftDown(l,1)return z end;function Heapq.peek(l)return l.data[1]end;function Heapq.size(l)return l.size end;function LinkedList.LinkedList()local A={}A.value=nil;A.next=A;A.prev=A;return A end;function LinkedList.pushFront(B,d)local A={}A.value=d;LinkedList.connect(A,B.next)LinkedList.connect(B,A)end;function LinkedList.pushBack(B,d)local A={}A.value=d;LinkedList.connect(B.prev,A)LinkedList.connect(A,B)end;function LinkedList.popFront(B)local C=B.next.value;LinkedList.connect(B,B.next.next)return C end;function LinkedList.popBack(B)local C=B.prev.value;LinkedList.connect(B.prev.prev,B)return C end;function LinkedList.peekFront(B)return B.next.val end;function LinkedList.peekBack(B)return B.prev.val end;function LinkedList.connect(D,E)D.next=E;E.prev=D end;function LinkedList.toArray(B)local F=1;local G={}local A=B.next;while A~=B do G[F]=A.value;A=A.next end;return G end;function MathUtil.angleOnPlane(H,I,J)local K=Vector3.ProjectOnPlane(H,J)local g=Vector3.ProjectOnPlane(I,J)return Vector3.SignedAngle(K,g,J)end;function MathUtil.min(L,M)local N=nil;M=M or function(m,n)return m<n end;for O in L do if not N or M(O,N)then N=O end end;return N end;function MathUtil.max(L,M)local P=nil;M=M or function(m,n)return m<n end;for O in L do if not P or M(P,O)then P=O end end;return P end;function MathUtil.range(m,n,Q)local R,S=m,n;local T;if not m then return end;if not n then R=0;S=m;T=R<S and 1 or-1 elseif Q then T=Q end;return function(U,V)local W=V+T;if W==S then return nil end;return W end,nil,R-T end;function MathUtil.shuffle(B,X)local s=X and B or{}if not X then for F=1,#B do s[F]=B[F]end end;for F=#B,2,-1 do local Y=math.random(F)s[F],s[Y]=s[Y],s[F]end;return s end;function MathUtil.combine(m,n,Z)if#m==#n then local z={}for _,a0 in pairs(m)do z[_]=Z(_,a0,n[_])end;return z end end;function MathUtil.angleSSS(m,n,Q)if m+n<Q or m+Q<n or n+Q<m then return nil end;local a1=math.deg(math.acos((n*n+Q*Q-m*m)/(2*n*Q)))local a2,a3=MathUtil.angleSAS(n,a1,Q)return a1,a2,a3 end;function MathUtil.sideSAS(m,a3,n)local a4=m*m+n*n-2*m*n*math.cos(math.rad(a3))return math.sqrt(a4)end;function MathUtil.angleSAS(m,a3,n)local Q=MathUtil.sideSAS(m,a3,n)if MathUtil.isZero(Q)then return nil end;local a1,a2;if m<n then a1=MathUtil.angleLoSin(Q,m,a3)a2=180-a1-a3 else a2=MathUtil.angleLoSin(Q,n,a3)a1=180-a2-a3 end;return a1,a2 end;function MathUtil.sideSSA(m,n,a1)local a5=n*n-m*m;local a6=-2*n*math.cos(math.rad(a1))local a7,a8=MathUtil.solveQuadratic(1,a6,a5)if not a8 then return a7,a8 end;if a7<a8 then return a7,a8 end;return a8,a7 end;function MathUtil.angleSSA(m,n,a1)local a7,a8=MathUtil.sideSSA(m,n,a1)if not a7 then return nil end;local a9,aa=MathUtil.angleSAS(n,a1,a7)if not a8 then return a9,aa end;local ab,ac=MathUtil.angleSAS(n,a1,a8)return a9,aa,ab,ac end;function MathUtil.sideAAS(a1,a2,m)local a3=180-a1-a2;local n=MathUtil.sideLoSin(a1,a2,m)local Q=MathUtil.sideLoSin(a1,a3,m)return n,Q end;function MathUtil.sideLoSin(m,a1,a2)return m*math.sin(math.rad(a2))/math.sin(math.rad(a1))end;function MathUtil.angleLoSin(m,n,a1)return math.deg(math.asin(n*math.sin(math.rad(a1))/m))end;function MathUtil.clampCone(ad,ae,af)local ag=math.min(af,Vector3.Angle(ad,ae))local ah=Vector3.Cross(ad,ae)return Quaternion.AngleAxis(ag,ah)*ad end;function MathUtil.newton(ai,aj,ak,al,am,an)al=al or 1e-5;an=an or 10*al;am=am or 25;aj=aj or function(ao)return(ai(ao+an)-ai(ao))/an end;ak=ak or 0;local ap=al+1;local aq=0;while ap>al and aq<am do local ar=ai(ak)local as=aj(ak)if not ar or not as then return nil end;ap=-ar/as;ak=ak+ap;aq=aq+1 end;if aq<am then return ak,false end;return ak,true end;function MathUtil.ITP(ai,m,n,al,am)if ai(m)*ai(n)>0 then return nil end;if ai(m)>ai(n)then ai=function(ao)return-ai(ao)end end;al=al or 1e-5;am=am or 25;local at=0.2/(n-m)local au=2;local av=1;local aw=math.ceil(math.log((n-m)/(2*al),2))local ax=aw+av;local aq=am;for Y=1,am do local ay=ai(m)local az=ai(n)local aA=ay-az;if aA==0 then return m end;local aB=0.5*(m+n)local aC=(n*ay+m*az)/aA;if aB<m or aB>n then aB=0.5*(m+n)end;local aD=aB-aC;local aE=at*math.abs(n-m)^au;local aF=aD>0 and 1 or(aD==0 and 0 or-1)local aG=aE<=math.abs(aD)and aC+aF*aE or aB;local aH=al*2^(ax-Y)-0.5*(n-m)local aI=math.abs(aG-aB)<=aH and aG or aB-aF*aH;local aJ=ai(aI)if aJ>0 then n=aI elseif aJ<0 then m=aI else return aI,Y==am end;if n-m<2*al then aq=Y;break end end;local ay=ai(m)local az=ai(n)local aA=az-ay;if aA~=0 then return(m*az-n*ay)/aA,aq==am end;return m,aq==am end;function MathUtil.binomCoeffs(aK,aL)if aL then local aM={}else local aM={}aM[1]=1;for _=1,aK do aM[_+1]=aM[_]*(aK-_)/(_+1)end;return aM end end;function MathUtil.ruleOfSigns(aM,aN)local aO={}local aP=#aM;for F=1,aP do aO[F]=aM[aP-F+1]end;if aN~=0 then local aQ={}for F=1,aP do aQ[F]=(F-1)*aM[aP-F+1]end;local aR=1;for F=2,aP do local aS=aN^(F-1)for Y=1,aP-F+1 do local aT=F+Y-1;aO[Y]=aO[Y]+aR*aQ[aT]*aS;aQ[aT]=aQ[aT]*(Y-1)end;aR=aR/F end end;local aU={}local o=1;for F,aV in ipairs(aO)do if aV~=0 then aU[o]=aV;o=o+1 end end;local aW=0;for F=1,#aU-1 do if aU[F]*aU[F+1]<0 then aW=aW+1 end end;return aW end;function MathUtil.cache(ai)local Q={}local aX=getmetatable(Q)or{}function aX.__index(aY,ao)local C=ai(ao)aY[ao]=C;return C end;setmetatable(Q,aX)return function(m)return Q[m]end end;function MathUtil.lerp(ai,R,S,T,aZ)local a_={}for F=1,math.floor((S-R)/T)+1 do a_[F]=ai(R+F*T)end;a_.start=R;a_.stop=S;a_.step=T;a_.lval=aZ and a_[1]or nil;a_.rval=aZ and a_[#a_]or nil;return function(ao)if ao>=a_.stop then return a_.rval end;if ao<=a_.start then return a_.lval end;local F=(ao-a_.start)/a_.step;local b0=F%1;F=math.floor(F)return(1-b0)*a_[F]+b0*a_[F+1]end end;function MathUtil._factorial(o)if o<2 then return 1 end;return MathUtil._factorial(o-1)end;MathUtil.factorial=MathUtil.cache(MathUtil._factorial)MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(al)MathUtil.eps=al end;function MathUtil.cuberoot(ao)return ao>0 and ao^(1/3)or-math.abs(ao)^(1/3)end;function MathUtil.solveQuadratic(b1,a7,a8)local b2,b3;local b4,b5,b6;b4=a7/(2*b1)b5=a8/b1;b6=b4*b4-b5;if MathUtil.isZero(b6)then b2=-b4;return b2 elseif b6<0 then return else local b7=math.sqrt(b6)b2=b7-b4;b3=-b7-b4;return b2,b3 end end;function MathUtil.solveCubic(b1,a7,a8,b8)local b2,b3,b9;local ba,bb;local a1,a2,a3;local bc,b4,b5;local bd,b6;a1=a7/b1;a2=a8/b1;a3=b8/b1;bc=a1*a1;b4=1/3*(-(1/3)*bc+a2)b5=0.5*(2/27*a1*bc-1/3*a1*a2+a3)bd=b4*b4*b4;b6=b5*b5+bd;if MathUtil.isZero(b6)then if MathUtil.isZero(b5)then b2=0;ba=1 else local be=MathUtil.cuberoot(-b5)b2=2*be;b3=-be;ba=2 end elseif b6<0 then local bf=1/3*math.acos(-b5/math.sqrt(-bd))local g=2*math.sqrt(-b4)b2=g*math.cos(bf)b3=-g*math.cos(bf+math.pi/3)b9=-g*math.cos(bf-math.pi/3)ba=3 else local b7=math.sqrt(b6)local be=MathUtil.cuberoot(b7-b5)local a0=-MathUtil.cuberoot(b7+b5)b2=be+a0;ba=1 end;bb=1/3*a1;if ba>0 then b2=b2-bb end;if ba>1 then b3=b3-bb end;if ba>2 then b9=b9-bb end;return b2,b3,b9 end;function MathUtil.solveQuartic(b1,a7,a8,b8,bg)local b2,b3,b9,bh;local aM={}local bi,be,a0,bb;local a1,a2,a3,b6;local bc,b4,b5,bj;local ba=0;a1=a7/b1;a2=a8/b1;a3=b8/b1;b6=bg/b1;bc=a1*a1;b4=-0.375*bc+a2;b5=0.125*bc*a1-0.5*a1*a2+a3;bj=-(3/256)*bc*bc+0.0625*bc*a2-0.25*a1*a3+b6;if MathUtil.isZero(bj)then aM[3]=b5;aM[2]=b4;aM[1]=0;aM[0]=1;local bk={MathUtil.solveCubic(aM[0],aM[1],aM[2],aM[3])}ba=#bk;b2,b3,b9=bk[1],bk[2],bk[3]elseif MathUtil.isZero(b5)then local bl={MathUtil.solveQuadratic(1,b4,bj)}if bl[1]>=0 then b2=-math.sqrt(bl[1])b3=math.sqrt(bl[1])ba=2 end;if bl[2]>=0 then if ba==0 then b2=-math.sqrt(bl[2])b3=math.sqrt(bl[2])ba=2 else b9=-math.sqrt(bl[2])bh=math.sqrt(bl[2])ba=4 end end else aM[3]=0.5*bj*b4-0.125*b5*b5;aM[2]=-bj;aM[1]=-0.5*b4;aM[0]=1;b2,b3,b9=MathUtil.solveCubic(aM[0],aM[1],aM[2],aM[3])bi=b2;be=bi*bi-bj;a0=2*bi-b4;if MathUtil.isZero(be)then be=0 elseif be>0 then be=math.sqrt(be)else return end;if MathUtil.isZero(a0)then a0=0 elseif a0>0 then a0=math.sqrt(a0)else return end;aM[2]=bi-be;aM[1]=b5<0 and-a0 or a0;aM[0]=1;do local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=#bk;b2,b3=bk[1],bk[2]end;aM[2]=bi+be;aM[1]=b5<0 and a0 or-a0;aM[0]=1;if ba==0 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b2,b3=bk[1],bk[2]end;if ba==1 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b3,b9=bk[1],bk[2]end;if ba==2 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b9,bh=bk[1],bk[2]end end;bb=0.25*a1;if ba>0 then b2=b2-bb end;if ba>1 then b3=b3-bb end;if ba>2 then b9=b9-bb end;if ba>3 then bh=bh-bb end;return b2,b3,b9,bh end;function Matrix3.Matrix3(a_)local bm={}for F=1,9 do bm[F]=a_[F]end;setmetatable(bm,Matrix3.meta)return bm end;function Matrix3.get(bm,bn,bo)return bm[(bn-1)*3+bo]end;function Matrix3.set(bm,bn,bo,C)bm[(bn-1)*3+bo]=C end;function Matrix3.scalarmul(bm,s)local bp={}for F=1,9 do bp[F]=s*bm[F]end;return Matrix3.Matrix3(bp)end;function Matrix3.vecmul(bm,bq)local bp=Vector3.zero;for bn=1,3 do local C=0;for bo=1,3 do C=C+bq[bo]*bm[(bn-1)*3+bo]end;bp[bn]=C end;return bp end;function Matrix3.matmul(br,bs)local bp={}for F=1,3 do for Y=1,3 do local C=0;for _=1,3 do C=C+br[(F-1)*3+_]*bs[(_-1)*3+Y]end;bp[(F-1)*3+Y]=C end end;return Matrix3.Matrix3(bp)end;function Matrix3.mul(m,n)if getmetatable(m)~=Matrix3.meta then if type(m)=="table"and m.x then return Matrix3.vecmul(Matrix3.transpose(n),m)end;return Matrix3.mul(n,m)end;if getmetatable(n)==Matrix3.meta then return Matrix3.matmul(m,n)end;if type(n)=="table"and n.x then return Matrix3.vecmul(m,n)end;return Matrix3.scalarmul(m,n)end;function Matrix3.quadform(bt,bm,bu)return Vector3.Dot(bt,bm*bu)end;function Matrix3.Identity()return Matrix3.Matrix3({1,0,0,0,1,0,0,0,1})end;function Matrix3.Zero()return Matrix3.Matrix3({0,0,0,0,0,0,0,0,0})end;function Matrix3.pow(bm,bv)local bw=bm;local bx=bm;while true do bv=math.floor(bv/2)if bv%2==1 then bw=Matrix3.matmul(bm,bx)end;if bv>=2 then bx=Matrix3.matmul(bx,bx)else break end end;return bw end;function Matrix3.add(br,bs)local bp={}for F=1,9 do bp[F]=br[F]+bs[F]end;return Matrix3.Matrix3(bp)end;function Matrix3.hadamard(br,bs)local bp={}for F=1,9 do bp[F]=br[F]*bs[F]end;return Matrix3.Matrix3(bp)end;function Matrix3.transpose(bm)local by={}for bn=1,3 do for bo=1,3 do by[(bo-1)*3+bn]=bm[(bn-1)*3+bo]end end;return Matrix3.Matrix3(by)end;function Matrix3.determinant(bm)local bz=0;local bA=0;for h=1,3 do local bB=1;local bC=1;for s=1,3 do bB=bB*bm[(s-1)*3+(s+h-2)%3+1]bC=bC*bm[(s-1)*3+(-s+h)%3+1]end;bz=bz+bB;bA=bA+bC end;return bz-bA end;function Matrix3.adjugate(bm)local bD={}for bn=1,3 do for bo=1,3 do local bB=1;local bC=1;for F=1,2 do bB=bB*bm[(bn+F-1)%3*3+(bo+F-1)%3+1]bC=bC*bm[(bn+F-1)%3*3+(bo-F-1)%3+1]end;bD[(bo-1)*3+bn]=bB-bC end end;return Matrix3.Matrix3(bD)end;function Matrix3.inverse(bm)local bE=Matrix3.determinant(bm)if MathUtil.isZero(bE)then return end;local bD=Matrix3.adjugate(bm)return 1/bE*bD end;function Matrix3.tostring(bm)local bF=string.format("%f, %f, %f\n%f, %f, %f\n%f, %f, %f",unpack(bm))return bF end;Matrix3.meta={__add=Matrix3.add,__mul=Matrix3.mul,__unm=function(bG)return Matrix3.scalarmul(bG,-1)end,__pow=Matrix3.pow,__tostring=Matrix3.tostring}function RingBuffer.RingBuffer(bH)local bI={}bI.buf={}bI.capacity=bH;bI.size=0;bI.head=1;local aX=getmetatable(bI)or{}aX.__index=RingBuffer.get;setmetatable(bI,aX)return bI end;function RingBuffer.isFull(bI)return bI.size>=bI.capacity end;function RingBuffer.push(bI,d)bI.buf[(bI.head+bI.size-1)%bI.capacity+1]=d;if bI.size==bI.capacity then bI.head=bI.head%bI.capacity+1 else bI.size=bI.size+1 end end;function RingBuffer.pop(bI)if bI.size==0 then return nil end;local C=bI.buf[bI.head]bI.buf[bI.head]=nil;bI.head=bI.head%bI.capacity+1;bI.size=bI.size-1;return C end;function RingBuffer.get(bI,p)if type(p)~="number"or math.floor(p)~=p then return nil end;if p<1 or p>bI.size then return nil end;return bI.buf[(bI.head+p-2)%bI.capacity+1]end;function Search.interpolatedSearch(bJ,t,u,bK,bL,bM)bM=bM or 50;local m,n,bN;local bO=0;while u>t do m=bJ[t]if m==bK then return t end;if m>bK then return bL and t or nil end;n=bJ[u]if n==bK then return u end;if n<bK then return bL and u or nil end;bN=math.floor((bK-m)/(n-m)*(u-t)+t)bN=math.min(math.max(bN,t+1),u-1)if bJ[bN]==bK then return bN end;if bK<bJ[bN]then if bL and math.abs(bJ[bN-1]-bK)>math.abs(bJ[bN]-bK)then return bN end;u=bN-1 else if bL and math.abs(bJ[bN+1]-bK)>math.abs(bJ[bN]-bK)then return bN end;t=bN+1 end;bO=bO+1;if bO>bM then break end end;return bL and t or nil end;function Stats.Distribution(bP)local bQ={n=0,vars=bP}if bP then local bR={}local bS={}local bT=#bP;for F,a0 in ipairs(bP)do bR[a0]=0;for Y=1,bT do bS[(F-1)*bT+Y]=0 end end;bQ.mean=bR;bQ.cov=Matrix3.Matrix3(bS)else bQ.mean=0;bQ.cov=0 end;return bQ end;function Stats.updateDistribution(bQ,bU,bV)local bW=bQ.n;bV=bV or 1;bQ.n=bQ.n+bV;if bQ.vars then local bX={}local bT=bQ.vars and#bQ.vars or 1;for F,a0 in ipairs(bQ.vars)do bX[F]=bQ.mean[a0]local bY=bX[F]+bU[a0]*bV/bQ.n;for Y=F,bT do local ae=bQ.vars[Y]local bZ=bQ.mean[ae]local b_=(bV or 1)*(bU[a0]-bY)*(bU[ae]-bZ)bQ.cov[(F-1)*bT+Y]=(bQ.cov[(F-1)*bT+Y]*bW+b_)/bQ.n;bQ.cov[(Y-1)*bT+F]=bQ.cov[(F-1)*bT+Y]end;bQ.mean[a0]=bY end else local bY=bQ.mean+bU*bV/bQ.n;bQ.cov=(bQ.cov*bW+bV*(bU-bY)*(bU-bQ.mean))/bQ.n;bQ.mean=bY end end;function Stats.updateDistributionBatched(bQ,c0,c1)if#c0==0 then return end;local c2={}local bT=bQ.vars and#bQ.vars or 1;local c3=0;for Y=1,#c0 do c3=c3+(c1 and c1[Y]or 1)end;bQ.n=bQ.n+c3;local bW=bQ.n;if bQ.vars then for F,c4 in ipairs(bQ.vars)do local bo={}for Y=1,#c0 do bo[Y]=c0[Y][c4]end;c2[F]=bo end;for F,a0 in ipairs(bQ.vars)do local c5=0;for Y,s in ipairs(c2[F])do c5=c5+s*(c1 and c1[Y]or 1)end;local bY=bQ.mean[a0]+c5/bQ.n;for Y=F,bT do local bZ=bQ.mean[bQ.vars[Y]]c5=0;for s=1,#c0 do c5=c5+(c1 and c1[s]or 1)*(c2[F][s]-bY)*(c2[Y][s]-bZ)end;bQ.cov[(F-1)*bT+Y]=(bQ.cov[(F-1)*bT+Y]*bW+c5)/bQ.n;bQ.cov[(Y-1)*bT+F]=bQ.cov[(F-1)*bT+Y]end;bQ.mean[a0]=bY end else local c5=0;for F,s in ipairs(c0)do c5=c5+s*(c1 and c1[F]or 1)end;local bY=bQ.mean+c5/bQ.n;c5=0;for F,s in ipairs(c0)do c5=c5+(c1 and c1[F]or 1)*(s-bY)*(s-bQ.mean)end;bQ.cov=(bQ.cov*bW+c5)/bQ.n end end;function Stats.mean(bQ)return bQ.mean end;function Stats.covariance(bQ)return bQ.cov end;function Stats.namedCovariance(bQ,c6,c7)if not bQ.vars then return bQ.cov end;local bT=bQ.vars and#bQ.vars or 1;for F=1,bT do if bQ.vars[F]==c6 then for Y=1,bT do if bQ.vars[Y]==c7 then return bQ.cov[(F-1)*bT+Y]end end end end end;function Stats.normal()local bi,c8=Stats.boxMuller()return bi end;function Stats.normalPDF(bi)return math.exp(-0.5*bi*bi)/math.sqrt(2*math.pi)end;function Stats.normalCDF(bi)local c9=0.2316419;local ca=0.319381530;local cb=-0.356563782;local cc=1.781477937;local cd=-1.821255978;local ce=1.330274429;local g=1/(1+c9*bi)return 1-Stats.normalPDF(bi)*(ca*g+cb*g^2+cc*g^3+cd*g^4+ce*g^5)end;function Stats.inverseNorm(b4)local cf=b4>=0.5 and b4 or-b4;local bi=5.55556*(1-((1-cf)/cf)^0.1186)if b4<0.5 then bi=-bi end;return bi end;function Stats.boxMuller()local cg=math.random()local ch=math.random()ch=math.random()ch=math.random()local bj=math.sqrt(-2*math.log(cg))local ci=2*math.pi*ch;return bj*math.cos(ci),bj*math.sin(ci)end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(m,n)local cj=type(m)=="number"local ck=type(n)=="number"if not cj and ck then return n+m end;if cj and not ck then return Stats.combine(m,n,function(_,ao,cl)return m+cl end)else return Stats.combine(m,n,function(_,ao,cl)return ao+cl end)end end;VectorN.mt.__sub=function(m,n)return m+-n end;VectorN.mt.__mul=function(m,n)local cj=type(m)=="number"local ck=type(n)=="number"if not cj and ck then return n*m end;if cj and not ck then local z={}for _,a0 in pairs(n)do z[_]=m*a0 end;return z else return Stats.combine(m,n,function(_,ao,cl)return ao*cl end)end end;VectorN.mt.__div=function(m,n)local cj=type(m)=="number"local ck=type(n)=="number"if not cj and ck then return m*1/n end;if cj and not ck then local z={}for _,a0 in pairs(n)do z[_]=m/a0 end;return z else return Stats.combine(m,n,function(_,ao,cl)return ao/cl end)end end;VectorN.mt.__unm=function(m)local z={}for _,a0 in pairs(m)do z[_]=-a0 end;return z end;function VectorN.VectorN(B)local bq={}for _,a0 in pairs(B)do if type(a0)=="table"then bq[_]=VectorN.VectorN(a0)else bq[_]=a0 end end;setmetatable(bq,VectorN.mt)return bq end;function Control.PID(cm,cn,co,cp,cq,cr)local cs={}cs.kP=cm;cs.kI=cn;cs.kD=co;cs.Iacc=Accumulator.Accumulator(cp,cq)if cr and cr~=0 then cs.period=cr end;return cs end;function Control.processPID(ct,cu,e)cu=ct.period and(cu+ct.period/2)%ct.period-ct.period/2 or cu;local b4=ct.kP*cu;local F,cv=Accumulator.update(ct.Iacc,cu,e)F=ct.kI*F/cv;local h=ct.kD*(cu-(ct.lastError or cu))/e;ct.lastError=cu;return b4+F+h end;function Control.FF(aM,cr)local cw={}cw.coeffs=aM;cw.degree=#aM-1;if cr and cr~=0 then cw.period=cr end;cw.derivs={}return cw end;function Control.processFF(ct,bK,e)local cx=0*bK;local cy=bK;local cz=bK;for F=1,ct.degree+1 do cz=ct.derivs[F]ct.derivs[F]=cy;cx=cx+ct.coeffs[F]*cy;if cz then local aD=cy-cz;if F==1 and ct.period then aD=(aD+ct.period/2)%ct.period-ct.period/2 end;cy=aD/e else break end end;return cx end;function Nav.toLocal(cA,cB,cC)local cD=cA-cB;return Quaternion.Inverse(cC)*cD end;function Nav.toGlobal(cE,cB,cC)local cD=cC*cE;return cD+cB end;function Nav.cartToPol(cF)local bj=cF.magnitude;local ci=Vector3.SignedAngle(Vector3.forward,cF,Vector3.up)local bf=90-Vector3.Angle(Vector3.up,cF)return Vector3(bj,ci,bf)end;function Nav.cartToCyl(cF)local cG=Vector3(cF.x,0,cF.z)local cH=cG.magnitude;local bf=Vector3.SignedAngle(Vector3.forward,cF,Vector3.up)local bi=cF.y;return Vector3(cH,bf,bi)end;function Nav.polToCart(cF)local bj,ci,bf=cF.x,cF.y,cF.z;local ao=Mathf.Sin(ci)*Mathf.Cos(bf)local cl=Mathf.Sin(bf)local bi=Mathf.Cos(ci)*Mathf.Cos(bf)return bj*Vector3(ao,cl,bi)end;function Nav.cylToCart(cF)local cH,bf,cI=cF.x,cF.y,cF.z;local ao=cH*Mathf.Sin(bf)local cl=cI;local bi=cH*Mathf.Cos(bf)return Vector3(ao,cl,bi)end;function Targeting.firstOrderTargeting(cJ,cK,cL)if cK.sqrMagnitude==0 then return cJ.normalized end;local cM=cJ-Vector3.Project(cJ,cK)local cN=Vector3.Dot(cK,cJ-cM)/cK.sqrMagnitude;local m,n=MathUtil.solveQuadratic(cK.sqrMagnitude-cL*cL,2*cN*cK.sqrMagnitude,cM.sqrMagnitude+cN*cN*cK.sqrMagnitude)local cO=nil;if m and m>=0 then cO=m end;if n and n>=0 and n<m then cO=n end;if cO then return cJ+cO*cK,cO end end;function Targeting.secondOrderTargetingNewton(cJ,cP,cQ,cL,cR,cS,ak)local aD=10000;local cT=0;local aq=0;if not ak then ak=0 end;local cU=cJ+ak*cP+0.5*ak*ak*cQ;while math.abs(aD)>0.001 and aq<10 do local g=cU.magnitude/cL;aD=g-cT;cT=g;aq=aq+1;cU=cJ+g*cP+0.5*g*g*cQ end;return cU,cT,aq end;Targeting.secondOrderTargeting=Targeting.secondOrderTargetingNewton;function Targeting.secondOrderTargetingITP(cJ,cP,cQ,cL,cR,cS,ak)if not ak then ak=0 end;local m=-0.25*cQ.sqrMagnitude;local n=-Vector3.Dot(cP,cQ)local Q=-(cP.sqrMagnitude-cL*cL+Vector3.Dot(cJ,cQ))local h=-2*Vector3.Dot(cJ,cP)local cu=-cJ.sqrMagnitude;local g;local cV=cQ.magnitude;local cW=cP.magnitude;local cX=cJ.magnitude;local cY,cZ=MathUtil.solveQuadratic(0.5*cV,cW+cL,-cX)local c_=math.max(cY,cZ)local d0;local aM={0.5*cV,cW-cL,cX}if MathUtil.ruleOfSigns(aM,0)==2 then local d1,d2=MathUtil.solveQuadratic(aM[1],aM[2],aM[3])if d1 then d0=math.min(d1,d2)end end;if not d0 or d0<c_ then local b2,b3,b9=MathUtil.solveCubic(4*m,3*n,2*Q,h)if not b9 then if b2>c_ then d0=b2 end else local d1=math.min(b2,b9)local d2=math.max(b2,b9)if d1>c_ then d0=d1 elseif d2>c_ then d0=d2 end end;if not d0 then return end end;local function d3(ao)return cu+ao*(h+ao*(Q+ao*(n+ao*m)))end;if ak>c_ and d0>ak then if d3(c_)*d3(g)<0 then d0=ak else c_=ak end end;g=MathUtil.ITP(d3,c_,d0,1e-4,25)if not g then return end;if g>=c_ and g<=d0 then local d4=cJ+cP*g+0.5*cQ*g*g;if d4.magnitude>=cR and d4.magnitude<=cS then return d4,g end end end;function Targeting.AIPPN(d5,cJ,d6,cK,d7)local cP=cK-d6;local d8=Vector3.Dot(-cP,cJ.normalized)if d8<=0 then d8=10 end;local d9=cJ.magnitude/d8;local da=Vector3.Cross(cJ,cP)/cJ.sqrMagnitude;local db=Vector3.Cross(cJ,d7)/cJ.sqrMagnitude*d9/2;local dc=da+db;local dd=Vector3.Cross(dc,cJ.normalized)local de=Vector3.ProjectOnPlane(dd,d6).normalized;local df=d5*d6.magnitude*dc.magnitude;return df*de end;function Targeting.ATPN(d5,cJ,d6,cK,d7)local cP=cK-d6;local d8=-Vector3.Dot(cP,cJ.normalized)if d8<=0 then d8=10 end;local da=Vector3.Cross(cJ,cP)/cJ.sqrMagnitude;local dd=Vector3.Cross(da,cJ.normalized)local dg=Vector3.ProjectOnPlane(d7,cJ)return d5*d8*dd+0.5*d5*d7 end;function BlockUtil.getWeaponsByName(dh,di,aW,dj)if DEBUG then dh:Log("searching for "..di)end;local dk={}aW=aW or-1;local Q=aW;if not dj or dj==0 or dj==2 then for F=0,dh:GetWeaponCount()-1 do if Q==0 then break end;if dh:GetWeaponBlockInfo(F).CustomName==di then table.insert(dk,{subIdx=nil,wpnIdx=F})if DEBUG then dh:Log("found weapon "..di.." on hull, type "..dh:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end;if not dj or dj==1 or dj==2 then local dl=dh:GetAllSubConstructs()for p=1,#dl do local bb=dl[p]for F=0,dh:GetWeaponCountOnSubConstruct(bb)-1 do if Q==0 then break end;if dh:GetWeaponBlockInfoOnSubConstruct(bb,F).CustomName==di then table.insert(dk,{subIdx=bb,wpnIdx=F})if DEBUG then dh:Log("found weapon "..di.." on subobj "..bb..", type "..dh:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end;Q=Q-1 end end end end;if DEBUG then dh:Log("weapon count: "..#dk)end;return dk end;function BlockUtil.getSubConstructsByName(dh,di,aW)if DEBUG then dh:Log("searching for "..di)end;local dl=dh:GetAllSubConstructs()local dm={}aW=aW or-1;local Q=aW;for p=1,#dl do local bb=dl[p]if Q==0 then break end;if dh:GetSubConstructInfo(bb).CustomName==di then table.insert(dm,bb)if DEBUG then dh:Log("found subobj "..di)end;Q=Q-1 end end;if DEBUG then dh:Log("subobj count: "..#dm)end;return dm end;function BlockUtil.getBlocksByName(dh,di,dn,aW)if DEBUG then dh:Log("searching for "..di)end;local dp={}aW=aW or-1;local Q=aW;for p=0,dh:Component_GetCount(dn)-1 do if Q==0 then break end;if dh:Component_GetBlockInfo(dn,p).CustomName==di then table.insert(dp,p)if DEBUG then dh:Log("found component "..di)end;Q=Q-1 end end;if DEBUG then dh:Log("component count: "..#dp)end;return dp end;function BlockUtil.populateWeaponsByName(dh,dj)if DEBUG then dh:Log("populating all weapons, mode "..dj)end;local dk={}for p=0,dh:GetWeaponCount()-1 do local di=dh:Component_GetBlockInfo(type,p).CustomName;if di and di~=''then dk[di]=dk[di]or{}table.insert(dk[di],{subIdx=nil,wpnIdx=p})if DEBUG then dh:Log("found weapon "..di.." on hull, type "..dh:GetWeaponInfo(p).WeaponType)end else table.insert(dk,{subIdx=nil,wpnIdx=p})if DEBUG then dh:Log("found unnamed weapon on hull, type "..dh:GetWeaponInfo(p).WeaponType)end end end;if not dj or dj==1 or dj==2 then local dl=dh:GetAllSubConstructs()for p=1,#dl do local bb=dl[p]for F=0,dh:GetWeaponCountOnSubConstruct(bb)-1 do local di=dh:Component_GetBlockInfo(type,F).CustomName;if di and di~=''then dk[di]=dk[di]or{}table.insert(dk[di],{subIdx=bb,wpnIdx=F})if DEBUG then dh:Log("found weapon "..di.." on subobj "..bb..", type "..dh:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end else table.insert(dk,{subIdx=bb,wpnIdx=F})if DEBUG then dh:Log("found unnamed weapon on subobj "..bb..", type "..dh:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end end end end end;if DEBUG then local aW=0;for _,a0 in pairs(dk)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;dh:Log("weapon count: "..aW)end;return dk end;function BlockUtil.populateSubConstructsByName(dh)if DEBUG then dh:Log("populating all subconstructs")end;local dl=dh:GetAllSubConstructs()local dm={}for p=1,#dl do local bb=dl[p]local di=dh:GetSubConstructInfo(bb).CustomName;if di and di~=''then dm[di]=dm[di]or{}table.insert(dm[di],bb)if DEBUG then dh:Log("found subobj "..di)end else table.insert(dm,bb)if DEBUG then dh:Log("found unnamed subobj")end end end;if DEBUG then local aW=0;for _,a0 in pairs(dm)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;dh:Log("subobject count: "..aW)end;return dm end;function BlockUtil.populateBlocksByName(dh,dn)if DEBUG then dh:Log("populating all blocks of type "..dn)end;local dp={}for p=0,dh:Component_GetCount(dn)-1 do local di=dh:Component_GetBlockInfo(dn,p).CustomName;if di and di~=''then dp[di]=dp[di]or{}table.insert(dp[di],p)if DEBUG then dh:Log("found component "..di)end else table.insert(dp,p)if DEBUG then dh:Log("found unnamed component of type "..dn)end end end;if DEBUG then local aW=0;for _,a0 in pairs(dp)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;dh:Log("component count: "..aW)end;return dp end;function BlockUtil.getWeaponInfo(dh,dq)if dq.subIdx then return dh:GetWeaponInfoOnSubConstruct(dq.subIdx,dq.wpnIdx)end;return dh:GetWeaponInfo(dq.wpnIdx)end;function BlockUtil.getWeaponBlockInfo(dh,dq)if dq.subIdx then return dh:GetWeaponBlockInfoOnSubConstruct(dq.subIdx,dq.wpnIdx)end;return dh:GetWeaponBlockInfo(dq.wpnIdx)end;function BlockUtil.aimWeapon(dh,dq,dr,ds)if dq.subIdx then dh:AimWeaponInDirectionOnSubConstruct(dq.subIdx,dq.wpnIdx,dr.x,dr.y,dr.z,ds)else dh:AimWeaponInDirection(dq.wpnIdx,dr.x,dr.y,dr.z,ds)end end;function BlockUtil.fireWeapon(dh,dq,ds)if dq.subIdx then return dh:FireWeaponOnSubConstruct(dq.subIdx,dq.wpnIdx,ds)end;return dh:FireWeapon(dq.wpnIdx,ds)end;function Combat.pickTarget(dh,dt,du)du=du or function(U,dv)return dv.Priority end;local bK,dw;for F in MathUtil.range(dh:GetNumberOfTargets(dt))do local dv=dh:GetTargetInfo(dt,F)local dx=du(dh,dv)if not bK or dx>dw then bK=dv;dw=dx end end;return bK end;function Combat.CheckConstraints(dh,dy,dz,dA)local dB;if dA then dB=dh:GetWeaponConstraintsOnSubConstruct(dA,dz)else dB=dh:GetWeaponConstraints(dz)end;if not dB or not dB.Valid then return true end;local dC=dh:GetConstructForwardVector()local dD=dh:GetConstructUpVector()local dE=Quaternion.LookRotation(dC,dD)dy=Quaternion.Inverse(dE)*dy;if dB.InParentConstructSpace and dA then local dF=dh:GetSubConstructInfo(dA).localRotation;dy=Quaternion.inverse(dF)*dy end;local dG=MathUtil.angleOnPlane(Vector3.forward,dy,Vector3.up)local dH=dy;dH.y=0;local O=Mathf.Atan2(dy.y,dH.magnitude)local dI=dG>dB.MinAzimuth and dG<dB.MaxAzimuth;local dJ=O>dB.MinElevation and O<dB.MaxElevation;if dB.FlipAzimuth then dI=not dI end;if dI and dJ then return true end;dG=dG+180;O=180-O;if O>180 then O=O-360 end;if O<-180 then O=O+360 end;dI=dG>dB.MinAzimuth and dG<dB.MaxAzimuth;dJ=O>dB.MinElevation and O<dB.MaxElevation;if dB.FlipAzimuth then dI=not dI end;if dI and dJ then return true end;return false end;function StringUtil.LogVector(dh,bq,dK)dh:Log(dK.."("..bq.x..", "..bq.y..", "..bq.z..")")end
