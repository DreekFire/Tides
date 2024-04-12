-- Settings

AllowErrorRecovery = true

-- seconds to record target movements for
local targetTrackTime = 5
-- number of locations to track per enemy (todo: support tracking multiple projectiles)
local numOrigins = 3
-- time between switching targets
local originSwitchTime = 2.5
-- maximum time to remember origin points
local maxStaleness = 6
-- ranges to engage
local minRange = 50
local maxRange = 4000
-- altitudes to engage (estimated altitude of firing piece, not target origin)
  -- checked when adding an origin and when firing
local minAlt = -2
local maxAlt = math.huge
-- parameters to find weapons (Shift+N to name turret blocks)
  -- different turrets should have different names even if they have the same weapon
  -- both turrets and firing pieces should be named
  -- firing pieces can share the name of the turret they are mounted on
  -- if weapons on the same turret have different muzzle velocities, name them differently from each other
    -- only one should have the same name as the turret, usually the one with the narrowest aiming arc
local weaponDef = {
    { name = "CB1", velocity = 1712 },
    { name = "CB2", velocity = 1666 },
	{ name = 'check1Turret', velocity = math.huge }
}
-- which mainframe to use
local mainframeIdx = 0
-- what to do when no firing origin detected
  -- timer: wait for waitTime seconds, then fire at current target if no origin found
  -- fire: aim and fire at current target
  -- enemy: aim at current target
  -- none: return to idle position
  -- last: continue aiming at last absolute bearing
local idleAim = "fire"
  -- time to wait for a new origin before firing at the aimpoint
    -- only used in idle aim mode "timer"
local waitTime = 3
-- offset the aimpoint, i.e. for hitting the necks, tetris, or turret bases instead of the barrels
  -- is a function which accepts the target point relative to the enemy origin to adjust depending
  -- on estimated turret location
-- todo: take in origin and aimpoint, return list of locations to attempt
local function aimOffset(fp)
  return Vector3(0, -6, 0)
end
local function aimPointScalingOffset(ap)
  return Vector3(ap.x * 0.75, ap.y * 0.5, ap.z * 0.75)
end


local checkACB = {

}
-- whether or not to attempt to snipe missile launchers
  -- (WIP) since missiles can turn, we can't trace back their trajectory
  -- however, if we detect a missile the instant it is launched (most feasible for huge missiles
    -- and large missiles with low ramp time, low ignition delay, and high thrust),
  -- we can approximate the missile as traveling in a straight line
local missileCounter = true
-- degrees of inaccuracy allowed when firing
-- weapon will start firing within this angle
-- but will always try to obtain perfect accuracy
local AIM_TOL = 1
-- physics ticks per second (Lua runs in sync with game physics)
local TICKS_PER_S = 40

-- todo: Integrate with enemy identififcation script to get weapon distances from origin,
    -- and trace to that distance from origin instead of closest
-- todo: Account for target acceleration

local lastProjectilePos
local times
local enemies
local currentLine
local lastFrameTime
local inited
local prevTime
local lastOrigin
local lastOriginSwitchTime = 0
local originPopTime = 0
local lastAim
local turrets = {}
local velocities = {}


local BlockUtil = {}
local Combat = {}
local StringUtil = {}
local Accumulator = {}
local Differ = {}
local Graph = {}
local LinkedList = {}
local Heapq = {}
local MathUtil = {}
local RingBuffer = {}
local VectorN = {}
local Control = {}
local Nav = {}
local Targeting = {}

local errors = ""

function Init(I)
    for idx, weapon in ipairs(weaponDef) do
    velocities[idx] = weapon.velocity
    turrets[idx] = BlockUtil.getWeaponsByName(I, weapon.name, 1, 2)
    end
    times = RingBuffer.RingBuffer(targetTrackTime * TICKS_PER_S)
    enemies = {}
    math.randomseed(I:GetTime())
    math.random()
    math.random()

    originPopTime = I:GetTimeSinceSpawn()
    inited = true
end

function Update(I)
  I:ClearLogs()
  I:Log(string.format("Game Time: %.2f", I:GetTimeSinceSpawn()))
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
  for tarIdx = 0, I:GetNumberOfTargets(mainframeIdx) do
    local target = I:GetTargetInfo(mainframeIdx, tarIdx)
    if not enemies[target.Id] then
      enemies[target.Id] = { pos = RingBuffer.RingBuffer(targetTrackTime * TICKS_PER_S),
                             vel = RingBuffer.RingBuffer(targetTrackTime * TICKS_PER_S),
                             origins = RingBuffer.RingBuffer(numOrigins),
                             originTimes = RingBuffer.RingBuffer(numOrigins),
                             valid = true }
      RingBuffer.setSize(enemies[target.Id].pos, times.size)
      RingBuffer.setSize(enemies[target.Id].vel, times.size)
    end
    enemies[target.Id].valid = true
    RingBuffer.push(enemies[target.Id].pos, target.Position)
    RingBuffer.push(enemies[target.Id].vel, target.Velocity)
  end
  for id, tar in pairs(enemies) do
    if not tar.valid then
        enemies[id] = nil
    else
        -- set valid to false. This will be set back to true the next frame if the target still exists
        -- otherwise the target no longer exists and we can clean up its data.
        enemies[id].valid = false
    end
  end

  local frameTime = lastFrameTime and I:GetTimeSinceSpawn() - lastFrameTime or 0
  lastFrameTime = I:GetTimeSinceSpawn()
  -- calculate projectile location
  local alt = 800 * I:GetPropulsionRequest(9) -- A axis, set in projectile avoidance routine
  local relBear = 180 * I:GetPropulsionRequest(10) -- B axis
  local dist = 800 * I:GetPropulsionRequest(12) -- D axis

  local projectile = dist * Vector3.forward
  projectile = Quaternion.AngleAxis(I:GetConstructYaw() + relBear, Vector3.up) * projectile
  projectile = projectile + I:GetConstructCenterOfMass()
  projectile.y = alt

  if prevTime and RingBuffer.isFull(times) then
    prevTime = prevTime - 1
  end
  RingBuffer.push(times, I:GetTimeSinceSpawn())
  if dist == 0 then
    currentLine = nil
    lastProjectilePos = nil
  end

  if missileCounter then
    for widx = 0, I:GetNumberOfWarnings(0) - 1 do
      local warn = I:GetMissileWarning(0, widx)
      if warn.Valid and warn.TimeSinceLaunch < 0.5 then
        local launcherPos = warn.Position - warn.TimeSinceLaunch * warn.Velocity
        local target = I:GetTargetInfo(mainframeIdx, 0)
        local targetPosAtLaunch = target.Position - warn.TimeSinceLaunch * target.Velocity
        if (launcherPos - targetPosAtLaunch).sqrMagnitude < 100 * 100 then
          local closestAlt = launcherPos.y
          if closestAlt > minAlt and closestAlt < maxAlt then
            local enemy = enemies[target.Id]
            RingBuffer.push(enemy.origins, launcherPos - targetPosAtLaunch)
            RingBuffer.push(enemy.originTimes, I:GetTimeSinceSpawn())
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
    local target = I:GetTargetInfo(mainframeIdx, 0)
    local enemy = enemies[target.Id]
    local relVel = (currentLine.ds / currentLine.dt) - target.Velocity
    local relPos = projectile - target.Position
    local time2d = math.sqrt((relPos.x ^ 2 + relPos.z ^ 2) / (relVel.x ^ 2 + relVel.z ^2))
    local estimate2d = relPos - (relVel * time2d) + 0.5 * I:GetGravityForAltitude(currentLine.start.y) * time2d * time2d
    if estimate2d.sqrMagnitude < 150 * 150 then
      local closest, closestTimeIdx = RunTrace(I, currentLine, enemy, prevTime)
      if not closest then
        I:Log("No solution found")
        prevTime = nil
        return
      end
      prevTime = closestTimeIdx
      if closest.sqrMagnitude < 100 * 100 then
        local closestAlt = closest.y + enemy.pos[closestTimeIdx].y
        if closestAlt > minAlt and closestAlt < maxAlt then
          RingBuffer.push(enemy.origins, closest)
          RingBuffer.push(enemy.originTimes, I:GetTimeSinceSpawn())
        end
      end
    end
  elseif lastProjectilePos and dist > 0 then
    local start = lastProjectilePos
    currentLine = {
      start = start,
      tStart = times[times.size - 1],
      ed = projectile
    }
    currentLine.dv = -frameTime * I:GetGravityForAltitude(start.y).y
    currentLine.dy = frameTime * currentLine.dv
    currentLine.ds = projectile + currentLine.dy * Vector3.up - start
    currentLine.dt = frameTime
    currentLine.ev = projectile - start
    prevTime = nil
  end

  lastProjectilePos = projectile

  -- fire weapon at origins
  local target = I:GetTargetInfo(mainframeIdx, 0)
  if target and target.Valid then
    local enemy = enemies[target.Id]
    local fp = lastOrigin
    local t = I:GetTimeSinceSpawn()
    if not fp or not lastOriginSwitchTime or t - lastOriginSwitchTime > originSwitchTime then
      while enemy.origins.size > 0 and t - enemy.originTimes[1] > maxStaleness do
        RingBuffer.pop(enemy.origins)
        RingBuffer.pop(enemy.originTimes)
        originPopTime = t
      end
      if enemy.origins.size == 0 then
        I:Log("No origins found.")
        fp = nil
      else
        fp = enemy.origins[math.random(1, enemy.origins.size)]
        if fp.y + target.Position.y < minAlt or fp.y + target.Position.y > maxAlt then
          -- do a linear search to find valid origin
          local found = false
          for i = 1, enemy.origins.size do
            fp = enemy.origins[i]
            if fp.y + target.Position.y > minAlt and fp.y + target.Position.y < maxAlt then
              found = true
              break
            end
          end
          if not found then
            fp = nil
            I:Log("No origin in bounds.")
          end
        end
      end
      lastOriginSwitchTime = t
      lastOrigin = fp
    else
        I:Log(tostring(enemy.origins.size) .." origins found.")
    end
    local aim
    local aimt
    for i, turret in ipairs(turrets) do
      for j, weapon in ipairs(turret) do
        I:Log(string.format("Controlling Turret:%0.0f Weapon:%0.0f", i, j))
        local fire = false
        local wInfo = BlockUtil.getWeaponInfo(I, weapon)
        local wfp
        if not fp and (idleAim == "fire" or (idleAim == "timer" and t - originPopTime > waitTime)) then
          wfp = aimPointScalingOffset(target.AimPointPosition - target.Position)
        elseif fp then
          wfp = fp + aimOffset(fp)
          CallComplexControl(I, "T")
          I:Log("Solution Found")
        end
        if wfp then
          if velocities[i] == math.huge then
            local range = (wfp + target.Position - I:GetConstructPosition()).magnitude
            if range > minRange and range < maxRange then
              aim = wfp + target.Position - wInfo.GlobalFirePoint
              fire = true
            end
          else
            local g = 0.5 * (I:GetGravityForAltitude(I:GetConstructPosition().y) + I:GetGravityForAltitude(target.Position.y))
            aim, aimt = Targeting.secondOrderTargeting(wfp + target.Position - wInfo.GlobalFirePoint,
                        target.Velocity - I:GetVelocityVector(),
                        -g,
                        velocities[i], minRange, maxRange)
            if aim then fire = true
            I:Log("Time to target: "..aimt)
          end
        end
        if not aim then
          fire = false
          if idleAim == "enemy" or idleAim == "fire" or idleAim == "timer" then
            aim = target.Position - wInfo.GlobalFirePoint
          elseif idleAim == "last" then
            aim = lastAim
          end
        end
        if aim then
          if Combat.CheckConstraints(I, aim, weapon.wpnIdx, weapon.subIdx) then
            BlockUtil.aimWeapon(I, weapon, aim, 0)
            if fire and Vector3.Angle(wInfo.CurrentDirection, aim) < AIM_TOL then
              if BlockUtil.fireWeapon(I, weapon, 0) then
                I:LogToHud("Firing weapon "..weapon.wpnIdx)
              end
            else
              I:LogToHud("Weapon still turning: "..weapon.wpnIdx..", error "..Vector3.angle(wInfo.CurrentDirection, aim))
            end
          else
            I:LogToHud("Weapon out of arc: "..weapon.wpnIdx)
          end
          if weapon.wpnIdx == 7 then lastAim = aim end
        end
      end
    end
  end
end

function CheckAndUpdateLine(I, line, projectile, frameTime, tolerance)
  local expected = line.ed + line.ev + frameTime * frameTime * I:GetGravityForAltitude(line.ed.y)
  local dv = line.dv - frameTime * I:GetGravityForAltitude(line.ed.y).y
  local dy = line.dy + frameTime * line.dv
  if (expected - projectile).sqrMagnitude <= tolerance * tolerance then
    line.ds = projectile + line.dy * Vector3.up - line.start
    line.dv = dv
    line.dy = dy
    line.dt = times[times.size] - line.tStart
    line.ev = projectile - line.ed
    line.ed = projectile
    return true
  end
  return false
end

function RunTrace(I, line, enemy, timeGuess)
  local totalIter = 0
  local targetPos = enemy.pos[enemy.pos.size]
  local targetVel = enemy.vel[enemy.vel.size]
  if timeGuess then
    targetPos = enemy.pos[timeGuess]
    targetVel = enemy.vel[timeGuess]
    if not targetPos then
      I:Log("initial guess has no target data")
      return nil
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
    local di = line.ed - targetPos
    local projRelVel = line.ds / line.dt - line.dv * Vector3.up - targetVel
    -- accounting exactly for gravity changes over altitude is difficult, just approximate and hope the enemy isn't using mortars
    local g = I:GetGravityForAltitude(line.ed.y).y
    local a, b, c = MathUtil.solveCubic(0.125 * g * g, 1.5 * projRelVel.y * g, projRelVel.sqrMagnitude + di.y * g, Vector3.Dot(di, projRelVel))
    -- critical point is a minimum when derivative changes from negative to positive
    -- since leading term is always positive (0.125g^2 = 12.2), if there are three roots, the first and third are minima
    -- if there is one root, it is a minimum
    local minRoot, minimum
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
    elseif a then
      minRoot = a
    end
    -- get target position and velocity at estimated time of closest approach
    tIdxClosest = InterpolatedSearch(I, times, 1, times.size, minRoot + line.tStart + line.dt, true)
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
    local dt = times[tIdx] - line.tStart
    return (line.start - enemy.pos[tIdx] + dt * (line.ds / line.dt) + 0.5 * I:GetGravityForAltitude(line.start.y) * dt * dt).sqrMagnitude
  end
  local currentSqrDist = CalcSqrDist(tIdxClosest)
  if tIdxClosest < enemy.pos.size then
    local aftSqrDist = CalcSqrDist(tIdxClosest + 1)
    while aftSqrDist < currentSqrDist and tIdxClosest < enemy.pos.size do
      currentSqrDist = aftSqrDist
      tIdxClosest = tIdxClosest + 1
      targetPos = enemy.pos[tIdxClosest]
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
      targetPos = enemy.pos[tIdxClosest]
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
  local dt = times[tIdxClosest] - line.tStart
  return line.start - targetPos + dt * (line.ds / line.dt) + 0.5 * I:GetGravityForAltitude(line.start.y) * dt * dt, tIdxClosest
end

function SqrDistance(I, line, targetAbsPos, targetAbsVel, t)
  local di = line.ed - targetAbsPos
  local viRel = line.ds / line.dt - line.dv * Vector3.up - targetAbsVel
  local diff = di + t * viRel + 0.5 * I:GetGravityForAltitude(line.ed.y) * t * t
  return diff.sqrMagnitude
end

function InterpolatedSearch(I, list, left, right, target, findClosest)
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
    if totalIter > 50 then
      I:Log("max iterations exceeded on InterpolatedSearch")
      break
    end
  end
  return findClosest and left or nil
end

function CallComplexControl(I, keyRequested)

    --T G Y H U J I K O L up down left right

    if keyRequested == nil then return end
    if string.len(keyRequested) == 0 then return end

    keyRequested = string.upper(keyRequested)

    stim = 0
    if     keyRequested == "T" then stim = 1 
    elseif keyRequested == "G" then stim = 2 
    elseif keyRequested == "Y" then stim = 3 
    elseif keyRequested == "H" then stim = 4 
    elseif keyRequested == "U" then stim = 5 
    elseif keyRequested == "J" then stim = 6 
    elseif keyRequested == "I" then stim = 7 
    elseif keyRequested == "K" then stim = 8 
    elseif keyRequested == "O" then stim = 9 
    elseif keyRequested == "L" then stim = 10 
    elseif keyRequested == "UP" then stim = 11 
    elseif keyRequested == "DOWN" then stim = 12 
    elseif keyRequested == "LEFT" then stim = 13 
    elseif keyRequested == "RIGHT" then stim = 14 end

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
function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function Heapq.Heapq(i,k)local l={}l.data=i;l.comp=k or function(m,n)return m<n end;local o=#l.data;l.size=o;for p=math.floor(o/2),1,-1 do Heapq.siftDown(l,p)end;return l end;function Heapq.siftDown(l,q)local r=false;local s=q;local o=#l.data;while not r do r=true;local t=2*s;local u=2*s+1;local v=s;if t<=o and l.comp(l.data[t],l.data[v])then v=t;r=false end;if u<=o and l.comp(l.data[u],l.data[v])then v=u;r=false end;if not r then local w=l.data[v]l.data[v]=l.data[s]l.data[s]=w;s=v end end end;function Heapq.siftUp(l,q)local r=false;local s=q;while not r do r=true;local x=math.floor(s/2)if l.comp(l.data[s],l.data[x])then local w=l.data[x]l.data[x]=l.data[s]l.data[s]=w;s=x;r=false end end end;function Heapq.insert(l,y)l.data[l.size+1]=y;l.size=l.size+1;Heapq.siftUp(l,l.size)end;function Heapq.pop(l)local z=l.data[1]l.data[1]=l.data[l.size]l.data[l.size]=nil;l.size=l.size-1;Heapq.siftDown(l,1)return z end;function Heapq.peek(l)return l.data[1]end;function Heapq.size(l)return l.size end;function LinkedList.LinkedList()local A={}A.value=nil;A.next=A;A.prev=A;return A end;function LinkedList.pushFront(B,d)local A={}A.value=d;LinkedList.connect(A,B.next)LinkedList.connect(B,A)end;function LinkedList.pushBack(B,d)local A={}A.value=d;LinkedList.connect(B.prev,A)LinkedList.connect(A,B)end;function LinkedList.popFront(B)local C=B.next.value;LinkedList.connect(B,B.next.next)return C end;function LinkedList.popBack(B)local C=B.prev.value;LinkedList.connect(B.prev.prev,B)return C end;function LinkedList.peekFront(B)return B.next.val end;function LinkedList.peekBack(B)return B.prev.val end;function LinkedList.connect(D,E)D.next=E;E.prev=D end;function LinkedList.toArray(B)local F=1;local G={}local A=B.next;while A~=B do G[F]=A.value;A=A.next end;return G end;function MathUtil.angleOnPlane(H,I,J)local K=Vector3.ProjectOnPlane(H,J)local g=Vector3.ProjectOnPlane(I,J)return Vector3.SignedAngle(K,g,J)end;function MathUtil.min(L,M)local N=nil;M=M or function(m,n)return m<n end;for O in L do if not N or M(O,N)then N=O end end;return N end;function MathUtil.max(L,M)local P=nil;M=M or function(m,n)return m<n end;for O in L do if not P or M(P,O)then P=O end end;return P end;function MathUtil.range(m,n,Q)local R,S=m,n;local T;if not m then return end;if not n then R=0;S=m;T=R<S and 1 or-1 elseif Q then T=Q end;return function(U,V)local W=V+T;if W==S then return nil end;return W end,nil,R-T end;function MathUtil.shuffle(B)local s={}for F=1,#B do s[F]=B[F]end;for F=#B,2,-1 do local X=math.random(F)s[F],s[X]=s[X],s[F]end;return s end;function MathUtil.combine(m,n,Y)if#m==#n then local z={}for Z,_ in pairs(m)do z[Z]=Y(Z,_,n[Z])end;return z end end;function MathUtil.distribution()return{n=0}end;function MathUtil.updateDistribution(a0,a1)a0.n=a0.n+1;if a0.n==1 then a0.mean=a1;a0.covariance={}local h=#a1;for F=1,h do local a2={}for X=1,h do a2[X]=0 end;a0.covariance[F]=a2 end else a0.mean=a0.mean+1/(a0.n+1)*a1 end end;function MathUtil.mean(a0)return a0.mean end;function MathUtil.covariance(a0)return a0.cov end;function MathUtil.normal()local a3,a4=MathUtil.boxMuller()return a3 end;function MathUtil.normalPDF(a3)return math.exp(-0.5*a3*a3)/math.sqrt(2*math.pi)end;function MathUtil.normalCDF(a3)local a5=0.2316419;local a6=0.319381530;local a7=-0.356563782;local a8=1.781477937;local a9=-1.821255978;local aa=1.330274429;local g=1/(1+a5*a3)return 1-MathUtil.normalPDF(a3)*(a6*g+a7*g^2+a8*g^3+a9*g^4+aa*g^5)end;function MathUtil.inverseNorm(ab)local ac=ab>=0.5 and ab or-ab;local a3=5.55556*(1-((1-ac)/ac)^0.1186)if ab<0.5 then a3=-a3 end;return a3 end;function MathUtil.boxMuller()local ad=math.random()local ae=math.random()ae=math.random()ae=math.random()local af=math.sqrt(-2*math.log(ad))local ag=2*math.pi*ae;return af*math.cos(ag),af*math.sin(ag)end;function MathUtil.angleSSS(m,n,Q)if m+n<Q or m+Q<n or n+Q<m then return nil end;local ah=math.deg(math.acos((n*n+Q*Q-m*m)/(2*n*Q)))local ai,aj=MathUtil.angleSAS(n,ah,Q)return ah,ai,aj end;function MathUtil.sideSAS(m,aj,n)local ak=m*m+n*n-2*m*n*math.cos(math.rad(aj))return math.sqrt(ak)end;function MathUtil.angleSAS(m,aj,n)local Q=MathUtil.sideSAS(m,aj,n)if MathUtil.isZero(Q)then return nil end;local ah,ai;if m<n then ah=MathUtil.angleLoSin(Q,m,aj)ai=180-ah-aj else ai=MathUtil.angleLoSin(Q,n,aj)ah=180-ai-aj end;return ah,ai end;function MathUtil.sideSSA(m,n,ah)local al=n*n-m*m;local am=-2*n*math.cos(math.rad(ah))local an,ao=MathUtil.solveQuadratic(1,am,al)if not ao then return an,ao end;if an<ao then return an,ao end;return ao,an end;function MathUtil.angleSSA(m,n,ah)local an,ao=MathUtil.sideSSA(m,n,ah)if not an then return nil end;local ap,aq=MathUtil.angleSAS(n,ah,an)if not ao then return ap,aq end;local ar,as=MathUtil.angleSAS(n,ah,ao)return ap,aq,ar,as end;function MathUtil.sideAAS(ah,ai,m)local aj=180-ah-ai;local n=MathUtil.sideLoSin(ah,ai,m)local Q=MathUtil.sideLoSin(ah,aj,m)return n,Q end;function MathUtil.sideLoSin(m,ah,ai)return m*math.sin(math.rad(ai))/math.sin(math.rad(ah))end;function MathUtil.angleLoSin(m,n,ah)return math.deg(math.asin(n*math.sin(math.rad(ah))/m))end;function MathUtil.clampCone(at,au,av)local aw=math.min(av,Vector3.Angle(at,au))local ax=Vector3.Cross(at,au)return Quaternion.AngleAxis(aw,ax)*at end;function MathUtil.newton(ay,az,aA,aB,aC,aD)aB=aB or 1e-5;aD=aD or 10*aB;aC=aC or 25;az=az or function(aE)return(ay(aE+aD)-ay(aE))/aD end;aA=aA or 0;local aF=aB+1;local aG=0;while aF>aB and aG<aC do local aH=ay(aA)local aI=az(aA)if not aH or not aI then return nil end;aF=-aH/aI;aA=aA+aF;aG=aG+1 end;if aG<aC then return aA,false end;return aA,true end;function MathUtil.ITP(ay,m,n,aB,aC)if ay(m)*ay(n)>0 then return nil end;local aJ;if ay(m)>ay(n)then aJ=function(aE)return-ay(aE)end else aJ=ay end;aB=aB or 1e-5;aC=aC or 25;local aK=0.2/(n-m)local aL=2;local aM=1;local aN=math.ceil(math.log((n-m)/(2*aB),2))local aO=aN+aM;local X=0;while n-m>2*aB and X<aC do local aP=(m+n)/2;local aQ=(n*ay(m)-m*ay(n))/(ay(m)-ay(n))local aR=aP-aQ;local aS=aK*math.abs(n-m)^aL;local aT=aR>0 and 1 or(aR==0 and 0 or-1)local aU=aS<=math.abs(aR)and aQ+aT*aS or aP;local aV=aB*2^(aO-X)-(n-m)/2;local aW=math.abs(aU-aP)<=aV and aU or aP-aT*aV;local aX=ay(aW)if aX>0 then n=aW elseif aX<0 then m=aW else m=aW;n=aW end;X=X+1 end;return(m+n)/2,X==aC end;function MathUtil.binomCoeffs(aY,aZ)if aZ then coeffs={}else coeffs={}coeffs[1]=1;for Z=1,aY do coeffs[Z+1]=coeffs[Z]*(aY-Z)/(Z+1)end;return coeffs end end;function MathUtil.ruleOfSigns(coeffs,a_)local b0={}local b1=#coeffs;for F=1,b1 do b0[F]=coeffs[b1-F+1]end;if a_~=0 then local b2={}for F=1,b1 do b2[F]=(F-1)*coeffs[b1-F+1]end;local b3=1;for F=2,b1 do local b4=a_^(F-1)for X=1,b1-F+1 do local b5=F+X-1;b0[X]=b0[X]+b3*b2[b5]*b4;b2[b5]=b2[b5]*(X-1)end;b3=b3/F end end;local b6={}local o=1;for F,b7 in ipairs(b0)do if b7~=0 then b6[o]=b7;o=o+1 end end;local b8=0;for F=1,#b6-1 do if b6[F]*b6[F+1]<0 then b8=b8+1 end end;return b8 end;function MathUtil.cache(ay)local Q={}local b9=getmetatable(Q)or{}function b9.__index(ba,aE)local C=ay(aE)ba[aE]=C;return C end;setmetatable(Q,b9)return function(m)return Q[m]end end;function MathUtil.lerp(ay,R,S,T,bb)local bc={}for F=1,math.floor((S-R)/T)+1 do bc[F]=ay(R+F*T)end;bc.start=R;bc.stop=S;bc.step=T;bc.lval=bb and bc[1]or nil;bc.rval=bb and bc[#bc]or nil;return function(aE)if aE>=bc.stop then return bc.rval end;if aE<=bc.start then return bc.lval end;local F=(aE-bc.start)/bc.step;local bd=F%1;F=math.floor(F)return(1-bd)*bc[F]+bd*bc[F+1]end end;function MathUtil._factorial(o)if o<2 then return 1 end;return MathUtil._factorial(o-1)end;MathUtil.factorial=MathUtil.cache(MathUtil._factorial)function MathUtil.rfft(be,bf)local bg;if bf==nil then bf=0.05 elseif type(bf)=="function"then bg=bf else bg=MathUtil.lerp(function(aE)return math.cos(aE*math.pi)end,0,2,bf)end;local o=#be;if be%2==0 and o>32 then local bh={}local bi={}local bj=MathUtil.rfft(bh,bg)local bk=MathUtil.rfft(bi,bg)else local z={}for bl=1,o do z[bl]=0 end;for F=1,o do for bl=1,o do local ag=(bl-1)*(F-1)/o%2;z[bl]=z[bl]+be[F]*bg(ag)end end end end;MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(aB)MathUtil.eps=aB end;function MathUtil.cuberoot(aE)return aE>0 and aE^(1/3)or-math.abs(aE)^(1/3)end;function MathUtil.solveQuadratic(bm,an,ao)local bn,bo;local ab,bp,bq;ab=an/(2*bm)bp=ao/bm;bq=ab*ab-bp;if MathUtil.isZero(bq)then bn=-ab;return bn elseif bq<0 then return else local br=math.sqrt(bq)bn=br-ab;bo=-br-ab;return bn,bo end end;function MathUtil.solveCubic(bm,an,ao,bs)local bn,bo,bt;local bu,bv;local ah,ai,aj;local bw,ab,bp;local bx,bq;ah=an/bm;ai=ao/bm;aj=bs/bm;bw=ah*ah;ab=1/3*(-(1/3)*bw+ai)bp=0.5*(2/27*ah*bw-1/3*ah*ai+aj)bx=ab*ab*ab;bq=bp*bp+bx;if MathUtil.isZero(bq)then if MathUtil.isZero(bp)then bn=0;bu=1 else local by=MathUtil.cuberoot(-bp)bn=2*by;bo=-by;bu=2 end elseif bq<0 then local bz=1/3*math.acos(-bp/math.sqrt(-bx))local g=2*math.sqrt(-ab)bn=g*math.cos(bz)bo=-g*math.cos(bz+math.pi/3)bt=-g*math.cos(bz-math.pi/3)bu=3 else local br=math.sqrt(bq)local by=MathUtil.cuberoot(br-bp)local _=-MathUtil.cuberoot(br+bp)bn=by+_;bu=1 end;bv=1/3*ah;if bu>0 then bn=bn-bv end;if bu>1 then bo=bo-bv end;if bu>2 then bt=bt-bv end;return bn,bo,bt end;function MathUtil.solveQuartic(bm,an,ao,bs,bA)local bn,bo,bt,bB;local coeffs={}local a3,by,_,bv;local ah,ai,aj,bq;local bw,ab,bp,af;local bu=0;ah=an/bm;ai=ao/bm;aj=bs/bm;bq=bA/bm;bw=ah*ah;ab=-0.375*bw+ai;bp=0.125*bw*ah-0.5*ah*ai+aj;af=-(3/256)*bw*bw+0.0625*bw*ai-0.25*ah*aj+bq;if MathUtil.isZero(af)then coeffs[3]=bp;coeffs[2]=ab;coeffs[1]=0;coeffs[0]=1;local bC={MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])}bu=#bC;bn,bo,bt=bC[1],bC[2],bC[3]elseif MathUtil.isZero(bp)then local bD={MathUtil.solveQuadratic(1,ab,af)}if bD[1]>=0 then bn=-math.sqrt(bD[1])bo=math.sqrt(bD[1])bu=2 end;if bD[2]>=0 then if bu==0 then bn=-math.sqrt(bD[2])bo=math.sqrt(bD[2])bu=2 else bt=-math.sqrt(bD[2])bB=math.sqrt(bD[2])bu=4 end end else coeffs[3]=0.5*af*ab-0.125*bp*bp;coeffs[2]=-af;coeffs[1]=-0.5*ab;coeffs[0]=1;bn,bo,bt=MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])a3=bn;by=a3*a3-af;_=2*a3-ab;if MathUtil.isZero(by)then by=0 elseif by>0 then by=math.sqrt(by)else return end;if MathUtil.isZero(_)then _=0 elseif _>0 then _=math.sqrt(_)else return end;coeffs[2]=a3-by;coeffs[1]=bp<0 and-_ or _;coeffs[0]=1;do local bC={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bu=#bC;bn,bo=bC[1],bC[2]end;coeffs[2]=a3+by;coeffs[1]=bp<0 and _ or-_;coeffs[0]=1;if bu==0 then local bC={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bu=bu+#bC;bn,bo=bC[1],bC[2]end;if bu==1 then local bC={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bu=bu+#bC;bo,bt=bC[1],bC[2]end;if bu==2 then local bC={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}bu=bu+#bC;bt,bB=bC[1],bC[2]end end;bv=0.25*ah;if bu>0 then bn=bn-bv end;if bu>1 then bo=bo-bv end;if bu>2 then bt=bt-bv end;if bu>3 then bB=bB-bv end;return bn,bo,bt,bB end;function RingBuffer.RingBuffer(bE)local bF={}bF.buf={}bF.capacity=bE;bF.size=0;bF.head=1;local b9=getmetatable(bF)or{}b9.__index=RingBuffer.get;setmetatable(bF,b9)return bF end;function RingBuffer.isFull(bF)return bF.size>=bF.capacity end;function RingBuffer.setSize(bF,bG)bF.size=bG end;function RingBuffer.push(bF,d)bF.buf[(bF.head+bF.size-1)%bF.capacity+1]=d;if bF.size==bF.capacity then bF.head=bF.head%bF.capacity+1 else bF.size=bF.size+1 end end;function RingBuffer.pop(bF)if bF.size==0 then return nil end;local C=bF.buf[bF.head]bF.buf[bF.head]=nil;bF.head=bF.head%bF.capacity+1;bF.size=bF.size-1;return C end;function RingBuffer.get(bF,p)if type(p)~="number"or math.floor(p)~=p then return nil end;if p<1 or p>bF.size then return nil end;return bF.buf[(bF.head+p-2)%bF.capacity+1]end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(m,n)local bH=type(m)=="number"local bI=type(n)=="number"if not bH and bI then return n+m end;if bH and not bI then return MathUtil.combine(m,n,function(Z,aE,bJ)return m+bJ end)else return MathUtil.combine(m,n,function(Z,aE,bJ)return aE+bJ end)end end;VectorN.mt.__sub=function(m,n)return m+-n end;VectorN.mt.__mul=function(m,n)local bH=type(m)=="number"local bI=type(n)=="number"if not bH and bI then return n*m end;if bH and not bI then local z={}for Z,_ in pairs(n)do z[Z]=m*_ end;return z else return MathUtil.combine(m,n,function(Z,aE,bJ)return aE*bJ end)end end;VectorN.mt.__div=function(m,n)local bH=type(m)=="number"local bI=type(n)=="number"if not bH and bI then return m*1/n end;if bH and not bI then local z={}for Z,_ in pairs(n)do z[Z]=m/_ end;return z else return MathUtil.combine(m,n,function(Z,aE,bJ)return aE/bJ end)end end;VectorN.mt.__unm=function(m)local z={}for Z,_ in pairs(m)do z[Z]=-_ end;return z end;function VectorN.VectorN(B)local bK={}for Z,_ in pairs(B)do if type(_)=="table"then bK[Z]=VectorN.VectorN(_)else bK[Z]=_ end end;setmetatable(bK,VectorN.mt)return bK end;function Control.PID(bL,bM,bN,bO,bP,bQ)local bR={}bR.kP=bL;bR.kI=bM;bR.kD=bN;bR.Iacc=Accumulator.Accumulator(bO,bP)if bQ and bQ~=0 then bR.period=bQ end;return bR end;function Control.processPID(bS,bT,e)bT=bS.period and(bT+bS.period/2)%bS.period-bS.period/2 or bT;local ab=bS.kP*bT;local F,bU=bS.kI*Accumulator.update(bS.Iacc,bT,e)F=F/bU;local h=bS.kD*(bT-(bS.lastError or bT))/e;bS.lastError=bT;return ab+F+h end;function Control.FF(coeffs,bQ)local bV={}bV.coeffs=coeffs;bV.degree=#coeffs-1;if bQ and bQ~=0 then bV.period=bQ end;bV.derivs={}return bV end;function Control.processFF(bS,bW,e)local bX=0*bW;local bY=bW;local bZ=bW;for F=1,bS.degree+1 do bZ=bS.derivs[F]bS.derivs[F]=bY;bX=bX+bS.coeffs[F]*bY;if bZ then local aR=bY-bZ;if F==1 and bS.period then aR=(aR+bS.period/2)%bS.period-bS.period/2 end;bY=aR/e else break end end;return bX end;function Nav.toLocal(b_,c0,c1)local c2=b_-c0;return Quaternion.Inverse(c1)*c2 end;function Nav.toGlobal(c3,c0,c1)local c2=c1*c3;return c2+c0 end;function Nav.cartToPol(c4)local af=c4.magnitude;local ag=Vector3.SignedAngle(Vector3.forward,c4,Vector3.up)local bz=90-Vector3.Angle(Vector3.up,c4)return Vector3(af,ag,bz)end;function Nav.cartToCyl(c4)local c5=Vector3(c4.x,0,c4.z)local c6=c5.magnitude;local bz=Vector3.SignedAngle(Vector3.forward,c4,Vector3.up)local a3=c4.y;return Vector3(c6,bz,a3)end;function Nav.polToCart(c4)local af,ag,bz=c4.x,c4.y,c4.z;local aE=Mathf.Sin(ag)*Mathf.Cos(bz)local bJ=Mathf.Sin(bz)local a3=Mathf.Cos(ag)*Mathf.Cos(bz)return af*Vector3(aE,bJ,a3)end;function Nav.cylToCart(c4)local c6,bz,c7=c4.x,c4.y,c4.z;local aE=c6*Mathf.Sin(bz)local bJ=c7;local a3=c6*Mathf.Cos(bz)return Vector3(aE,bJ,a3)end;function Targeting.firstOrderTargeting(c8,c9,ca)local cb=c8-Vector3.Project(c8,c9)local cc=Vector3.Dot(c9,c8-cb)/c9.sqrMagnitude;local m,n=MathUtil.solveQuadratic(cc-ca*ca,2*cc,cb.sqrMagnitude+cc*cc)local cd=nil;if m and m>=0 then cd=m end;if n and n>=0 and n<m then cd=n end;return cd and(c8+cd*c9).normalized or nil end;function Targeting.secondOrderTargeting(c8,ce,cf,ca,cg,ch)local m=-0.25*cf.sqrMagnitude;local n=-Vector3.Dot(ce,cf)local Q=-(ce.sqrMagnitude-ca*ca+Vector3.Dot(c8,cf))local h=-2*Vector3.Dot(c8,ce)local bT=-c8.sqrMagnitude;local g;local ci=cf.magnitude;local cj=ce.magnitude;local ck=c8.magnitude;local cl,cm=MathUtil.solveQuadratic(0.5*ci,cj+ca,-ck)local cn=math.max(cl,cm)local co;local coeffs={0.5*ci,cj-ca,ck}if MathUtil.ruleOfSigns(coeffs,0)==2 then local cp,cq=MathUtil.solveQuadratic(coeffs[1],coeffs[2],coeffs[3])if cp then co=math.min(cp,cq)end end;if not co or co<cn then local bn,bo,bt=MathUtil.solveCubic(4*m,3*n,2*Q,h)if not bt then if bn>cn then co=bn end else local cp=math.min(bn,bt)local cq=math.max(bn,bt)if cp>cn then co=cp elseif cq>cn then co=cq end end;if not co then return nil end end;local function cr(aE)return bT+aE*(h+aE*(Q+aE*(n+aE*m)))end;g=MathUtil.ITP(cr,cn,co,1e-4,25)if not g then return nil end;local cs;if g and g>=cn and g<=co then cs=c8/g+ce+0.5*cf*g end;if cs and cs.sqrMagnitude>=cg*cg and cs.sqrMagnitude<=ch*ch then return cs,g end end;function Targeting.AIPPN(ct,c8,cu,c9,cv)local ce=c9-cu;local cw=Vector3.Dot(-ce,c8.normalized)if cw<=0 then cw=10 end;local cx=c8.magnitude/cw;local cy=Vector3.Cross(c8,ce)/c8.sqrMagnitude;local cz=Vector3.Cross(c8,cv)/c8.sqrMagnitude*cx/2;local cA=cy+cz;local cB=Vector3.Cross(cA,c8.normalized)local cC=Vector3.ProjectOnPlane(cB,cu).normalized;local cD=ct*cu.magnitude*cA.magnitude;return cD*cC end;function Targeting.ATPN(ct,c8,cu,c9,cv)local ce=c9-cu;local cw=-Vector3.Dot(ce,c8.normalized)if cw<=0 then cw=10 end;local cy=Vector3.Cross(c8,ce)/c8.sqrMagnitude;local cB=Vector3.Cross(cy,c8.normalized)local cE=Vector3.ProjectOnPlane(cv,c8)return ct*cw*cB+0.5*ct*cv end;function BlockUtil.getWeaponsByName(cF,cG,b8,cH)if DEBUG then cF:Log("searching for "..cG)end;local cI=cF:GetAllSubConstructs()local cJ={}b8=b8 or-1;local Q=b8;if not cH or cH==0 or cH==2 then for F=0,cF:GetWeaponCount()-1 do if Q==0 then break end;if cF:GetWeaponBlockInfo(F).CustomName==cG then table.insert(cJ,{subIdx=nil,wpnIdx=F})if DEBUG then cF:Log("found weapon "..cG.." on hull, type "..cF:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end;if not cH or cH==1 or cH==2 then for p=1,#cI do local bv=cI[p]for F=0,cF:GetWeaponCountOnSubConstruct(bv)-1 do if Q==0 then break end;if cF:GetWeaponBlockInfoOnSubConstruct(bv,F).CustomName==cG then table.insert(cJ,{subIdx=bv,wpnIdx=F})if DEBUG then cF:Log("found weapon "..cG.." on subobj "..bv..", type "..cF:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end end;if DEBUG then cF:Log("weapon count: "..#cJ)end;return cJ end;function BlockUtil.getSubConstructsByName(cF,cG,b8)if DEBUG then cF:Log("searching for "..cG)end;local cI=cF:GetAllSubConstructs()local cK={}b8=b8 or-1;local Q=b8;for p=1,#cI do local bv=cI[p]if Q==0 then break end;if cF:GetSubConstructInfo(bv).CustomName==cG then table.insert(cK,bv)if DEBUG then cF:Log("found subobj "..cG)end;Q=Q-1 end end;if DEBUG then cF:Log("subobj count: "..#cK)end;return cK end;function BlockUtil.getBlocksByName(cF,cG,type,b8)if DEBUG then cF:Log("searching for "..cG)end;local cL={}b8=b8 or-1;local Q=b8;for p=0,cF:Component_GetCount(type)-1 do if Q==0 then break end;if cF:Component_GetBlockInfo(type,p).CustomName==cG then table.insert(cL,p)if DEBUG then cF:Log("found component "..cG)end;Q=Q-1 end end;if DEBUG then cF:Log("component count: "..#cL)end;return cL end;function BlockUtil.getWeaponInfo(cF,cM)local cN;if cM.subIdx then cN=cF:GetWeaponInfoOnSubConstruct(cM.subIdx,cM.wpnIdx)else cN=cF:GetWeaponInfo(cM.wpnIdx)end;return cN end;function BlockUtil.aimWeapon(cF,cM,cO,cP)if cM.subIdx then cF:AimWeaponInDirectionOnSubConstruct(cM.subIdx,cM.wpnIdx,cO.x,cO.y,cO.z,cP)else cF:AimWeaponInDirection(cM.wpnIdx,cO.x,cO.y,cO.z,cP)end end;function BlockUtil.fireWeapon(cF,cM,cP)if cM.subIdx then cF:FireWeaponOnSubConstruct(cM.subIdx,cM.wpnIdx,cP)else cF:FireWeapon(cM.wpnIdx,cP)end end;function Combat.pickTarget(cF,cQ,cR)cR=cR or function(U,cS)return cS.Priority end;local bW,cT;for F in MathUtil.range(cF:GetNumberOfTargets(cQ))do local cS=cF:GetTargetInfo(cQ,F)local cU=cR(cF,cS)if not bW or cU>cT then bW=cS;cT=cU end end;return bW end;function Combat.CheckConstraints(cF,cV,cW,cX)local cY;if cX then cY=cF:GetWeaponConstraintsOnSubConstruct(cX,cW)else cY=cF:GetWeaponConstraints(cW)end;local cZ=cF:GetConstructForwardVector()local c_=cF:GetConstructUpVector()local d0=Quaternion.LookRotation(cZ,c_)cV=Quaternion.Inverse(d0)*cV;if cY.InParentConstructSpace and cX then local d1=cF:GetSubConstructInfo(cX).localRotation;cV=Quaternion.inverse(d1)*cV end;local d2=MathUtil.angleOnPlane(Vector3.forward,cV,Vector3.up)local d3=cV;d3.z=0;local O=Mathf.Atan2(cV.z,d3.magnitude)local d4=d2>cY.MinAzimuth and d2<cY.MaxAzimuth;local d5=O>cY.MinElevation and O<cY.MaxElevation;if cY.FlipAzimuth then d4=not d4 end;if d4 and d5 then return true end;d2=d2+180;O=180-O;if O>180 then O=O-360 end;if O<-180 then O=O+360 end;d4=d2>cY.MinAzimuth and d2<cY.MaxAzimuth;d5=O>cY.MinElevation and O<cY.MaxElevation;if cY.FlipAzimuth then d4=not d4 end;if d4 and d5 then return true end;return false end;function StringUtil.LogVector(cF,bK,d6)cF:Log(d6 .."("..bK.x..", "..bK.y..", "..bK.z..")")end

function Targeting.stationaryTargetingTime(relPos, muzzle, gravity)
  local a = 0.25 * gravity.sqrMagnitude
  local b = Vector3.Dot(relPos, gravity) - muzzle * muzzle
  local c = relPos.sqrMagnitude
  local t1, t2 = MathUtil.solveQuadratic(a, b, c)
  if t2 and t2 > 0 and t2 < t1 then
    return math.sqrt(t2)
  elseif t1 and t1 > 0 then
    return math.sqrt(t1)
  end
end

function Targeting.secondOrderTargeting(relPos, relVel, accel, muzzle, minRange, maxRange)
  local diff = 10000
  local lastT = 0
  local iters = 0
  while math.abs(diff) > 0.001 and iters < 10 do
    local newPos = relPos + lastT * relVel
    local t = Targeting.stationaryTargetingTime(newPos, muzzle, accel)
    diff = t - lastT
    lastT = t
    iters = iters + 1
  end
  local aimPos = relPos + lastT * relVel + 0.5 * lastT * lastT * accel
  return aimPos, lastT, iters
end