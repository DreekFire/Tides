-- Settings
-- seconds to record target movements for
local targetTrackTime = 10
-- number of locations to track per enemy (todo: support tracking multiple projectiles)
local numOrigins = 1
-- time between switching targets
local originSwitchTime = 0.25
-- maximum time to remember origin points
local maxStaleness = 3
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
local weaponDef = {
  { name = "laser", velocity = math.huge }
}
-- which mainframe to use
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
  -- is a function which accepts the target point relative to the enemy origin to adjust depending
  -- on estimated turret location
local function aimOffset(fp)
  return Vector3(0, 0, 0)
end
-- whether or not to attempt to snipe missile launchers
  -- (WIP) since missiles can turn, we can't trace back their trajectory
  -- however, if we detect a missile the instant it is launched (most feasible for huge missiles
    -- and large missiles with low ramp time, low ignition delay, and high thrust),
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

local projectilePos
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
local MathUtil = {}
local RingBuffer = {}
local VectorN = {}
local Control = {}
local Nav = {}
local Targeting = {}


function Init(I)
  for idx, weapon in ipairs(weaponDef) do
    velocities[idx] = weapon.velocity
    turrets[idx] = BlockUtil.getWeaponsByName(I, weapon.name, 1, 2)
  end
  projectilePos = RingBuffer.RingBuffer(targetTrackTime * TICKS_PER_S)
  times = RingBuffer.RingBuffer(targetTrackTime * TICKS_PER_S)
  enemies = {}
  math.randomseed(I:GetTime())
  math.random()
  math.random()
  originPopTime = I:GetTimeSinceSpawn()
  inited = true
end

function Update(I)
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
    RingBuffer.push(projectilePos, Vector3.zero)
  else
    RingBuffer.push(projectilePos, projectile)
  end

  if missileCounter then
    -- missile warning info seems to be most reliable on mainframe 0
      -- but I'm still getting invalid missiles sometimes
    for widx = 0, I:GetNumberOfWarnings(0) - 1 do
      local warn = I:GetMissileWarning(0, widx)
      if warn.Valid and warn.TimeSinceLaunch < 0.5 then
        local launcherPos = warn.Position - warn.Velocity * warn.TimeSinceLaunch
        local target = I:GetTargetInfo(0, 0)
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

  if projectilePos.size == 1 then return end
  -- see if it matches current line
  -- todo: store multiple lines and find match

  -- 2/3rds the estimated drop in two frames due to gravity
  -- inconsistent with theoretical formula due to discrete integration
  -- powered missiles have no gravity so their expected error
  -- is the negative of the drop due to gravity
  -- todo: use missile warning info to filter out missiles
  local eps = 20 * frameTime * frameTime
  -- todo: maybe limit how often the line tracing runs to save processing power
  if currentLine and CheckAndUpdateLine(I, currentLine, projectile, frameTime, eps) then
    local target = I:GetTargetInfo(mainframeIdx, 0)
    local enemy = enemies[target.Id]
    local relVel = (currentLine.ds / currentLine.dt) - target.Velocity
    local relPos = projectile - target.Position
    local time2d = math.sqrt((relPos.x ^ 2 + relPos.z ^ 2) / (relVel.x ^ 2 + relVel.z ^2))
    local estimate2d = relPos - (relVel * time2d) + 0.5 * I:GetGravityForAltitude(currentLine.start.y) * time2d * time2d
    if estimate2d.sqrMagnitude < 150 * 150 then
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
          RingBuffer.push(enemy.origins, closest)
          RingBuffer.push(enemy.originTimes, I:GetTimeSinceSpawn())
        end
      end
    end
  elseif dist > 0 then
    local start = projectilePos[projectilePos.size - 1]
    currentLine = {
      start = start,
      tStart = times[times.size - 1],
      ed = projectile
    }
    currentLine.dv = -frameTime * I:GetGravityForAltitude(start.y).y
    currentLine.dy = frameTime * currentLine.dv
    currentLine.ds = projectile + currentLine.dy * Vector3.up - start
    currentLine.dt = frameTime
    prevTime = nil
  end

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
          if not found then fp = nil end
        end
      end
      lastOriginSwitchTime = t
      lastOrigin = fp
    end
    local aim
    for i, turret in ipairs(turrets) do
      for j, weapon in ipairs(turret) do
        local fire = false
        local wInfo = BlockUtil.getWeaponInfo(I, weapon)
        if not fp and idleAim == "fire" or idleAim == "timer" and t - originPopTime > waitTime then
          fp = target.AimPointPosition - target.Position
        elseif fp then
          fp = fp + aimOffset(fp)
        end
        if fp then
          if velocities[i] == math.huge then
            local range = (fp + target.Position - I:GetConstructPosition()).magnitude
            if range > minRange and range < maxRange then
              aim = fp + target.Position - wInfo.GlobalFirePoint
              fire = true
            end
          else
            local g = 0.5 * (I:GetGravityForAltitude(I:GetConstructPosition().y) + I:GetGravityForAltitude(target.Position.y))
            aim = Targeting.secondOrderTargeting(fp + target.Position - wInfo.GlobalFirePoint,
                        target.Velocity - I:GetVelocityVector(),
                        -g,
                        velocities[i], minRange, maxRange)
            if aim then fire = true end
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
end

function CheckAndUpdateLine(I, line, projectile, frameTime, tolerance)
  local expected = line.ed + (line.ds * frameTime / line.dt)
  local dv = line.dv - frameTime * I:GetGravityForAltitude(line.ed.y).y
  local dy = line.dy + frameTime * dv
  local projectileZeroG = projectile + dy * Vector3.up
  if (expected - projectileZeroG).sqrMagnitude <= tolerance * tolerance then
    line.ds = projectileZeroG - line.start
    line.dv = dv
    line.dy = dy
    line.dt = times[times.size] - line.tStart
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
  for i = 1, timeGuess and 1 or 2 do
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

-- minified version of Tides library (not meant to be human-readable, see Tides.lua or individual class files for human-readable source)
function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function LinkedList.LinkedList()local k={}k.value=nil;k.next=k;k.prev=k;return k end;function LinkedList.pushFront(l,d)local k={}k.value=d;LinkedList.connect(k,l.next)LinkedList.connect(l,k)end;function LinkedList.pushBack(l,d)local k={}k.value=d;LinkedList.connect(l.prev,k)LinkedList.connect(k,l)end;function LinkedList.popFront(l)local m=l.next.value;LinkedList.connect(l,l.next.next)return m end;function LinkedList.popBack(l)local m=l.prev.value;LinkedList.connect(l.prev.prev,l)return m end;function LinkedList.peekFront(l)return l.next.val end;function LinkedList.peekBack(l)return l.prev.val end;function LinkedList.connect(n,o)n.next=o;o.prev=n end;function LinkedList.toArray(l)local p=1;local q={}local k=l.next;while k~=l do q[p]=k.value;k=k.next end;return q end;function MathUtil.angleOnPlane(r,s,t)local u=Vector3.ProjectOnPlane(r,t)local g=Vector3.ProjectOnPlane(s,t)return Vector3.SignedAngle(u,g,t)end;function MathUtil.min(v,w)local x=nil;w=w or function(y,z)return y<z end;for A in v do if not x or w(A,x)then x=A end end;return x end;function MathUtil.max(v,w)local B=nil;w=w or function(y,z)return y<z end;for A in v do if not B or w(B,A)then B=A end end;return B end;function MathUtil.range(y,z,C)local D,E=y,z;local F;if not y then return end;if not z then D=0;E=y;F=D<E and 1 or-1 elseif C then F=C end;return function(G,H)local I=H+F;if I==E then return nil end;return I end,nil,D-F end;function MathUtil.shuffle(l)local J={}for p=1,#l do J[p]=l[p]end;for p=#l,2,-1 do local K=math.random(p)J[p],J[K]=J[K],J[p]end;return J end;function MathUtil.combine(y,z,L)if#y==#z then local M={}for N,O in pairs(y)do M[N]=L(N,O,z[N])end;return M end end;function MathUtil.distribution()return{n=0}end;function MathUtil.updateDistribution(P,Q)P.n=P.n+1;if P.n==1 then P.mean=Q;P.covariance={}local h=#Q;for p=1,h do local R={}for K=1,h do R[K]=0 end;P.covariance[p]=R end else P.mean=P.mean+1/(P.n+1)*Q end end;function MathUtil.mean(P)return P.mean end;function MathUtil.covariance(P)return P.cov end;function MathUtil.normal()local S,T=MathUtil.boxMuller()return S end;function MathUtil.normalPDF(S)return math.exp(-0.5*S*S)/math.sqrt(2*math.pi)end;function MathUtil.normalCDF(S)local U=0.2316419;local V=0.319381530;local W=-0.356563782;local X=1.781477937;local Y=-1.821255978;local Z=1.330274429;local g=1/(1+U*S)return 1-MathUtil.normalPDF(S)*(V*g+W*g^2+X*g^3+Y*g^4+Z*g^5)end;function MathUtil.inverseNorm(_)local a0=_>=0.5 and _ or-_;local S=5.55556*(1-((1-a0)/a0)^0.1186)if _<0.5 then S=-S end;return S end;function MathUtil.boxMuller()local a1=math.random()local a2=math.random()a2=math.random()a2=math.random()local a3=math.sqrt(-2*math.log(a1))local a4=2*math.pi*a2;return a3*math.cos(a4),a3*math.sin(a4)end;function MathUtil.angleSSS(y,z,C)if y+z<C or y+C<z or z+C<y then return nil end;local a5=math.deg(math.acos((z*z+C*C-y*y)/(2*z*C)))local a6,a7=MathUtil.angleSAS(z,a5,C)return a5,a6,a7 end;function MathUtil.sideSAS(y,a7,z)local a8=y*y+z*z-2*y*z*math.cos(math.rad(a7))return math.sqrt(a8)end;function MathUtil.angleSAS(y,a7,z)local C=MathUtil.sideSAS(y,a7,z)if MathUtil.isZero(C)then return nil end;local a5,a6;if y<z then a5=MathUtil.angleLoSin(C,y,a7)a6=180-a5-a7 else a6=MathUtil.angleLoSin(C,z,a7)a5=180-a6-a7 end;return a5,a6 end;function MathUtil.sideSSA(y,z,a5)local a9=z*z-y*y;local aa=-2*z*math.cos(math.rad(a5))local ab,ac=MathUtil.solveQuadratic(1,aa,a9)if not ac then return ab,ac end;if ab<ac then return ab,ac end;return ac,ab end;function MathUtil.angleSSA(y,z,a5)local ab,ac=MathUtil.sideSSA(y,z,a5)if not ab then return nil end;local ad,ae=MathUtil.angleSAS(z,a5,ab)if not ac then return ad,ae end;local af,ag=MathUtil.angleSAS(z,a5,ac)return ad,ae,af,ag end;function MathUtil.sideAAS(a5,a6,y)local a7=180-a5-a6;local z=MathUtil.sideLoSin(a5,a6,y)local C=MathUtil.sideLoSin(a5,a7,y)return z,C end;function MathUtil.sideLoSin(y,a5,a6)return y*math.sin(math.rad(a6))/math.sin(math.rad(a5))end;function MathUtil.angleLoSin(y,z,a5)return math.deg(math.asin(z*math.sin(math.rad(a5))/y))end;function MathUtil.clampCone(ah,ai,aj)local ak=math.min(aj,Vector3.Angle(ah,ai))local al=Vector3.Cross(ah,ai)return Quaternion.AngleAxis(ak,al)*ah end;function MathUtil.fourier(am)return"Work in progress"end;function MathUtil.newton(an,ao,ap,aq,ar,as)aq=aq or 1e-5;as=as or 10*aq;ar=ar or 100;ao=ao or function(at)return(an(at+as)-an(at))/as end;ap=ap or 0;local au=aq+1;local av=0;while au>aq and av<ar do local aw=an(ap)local ax=ao(ap)if not aw or not ax then return nil end;au=-aw/ax;ap=ap+au;av=av+1 end;if av<ar then return ap,false end;return ap,true end;MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(aq)MathUtil.eps=aq end;function MathUtil.cuberoot(at)return at>0 and at^(1/3)or-math.abs(at)^(1/3)end;function MathUtil.solveQuadratic(ay,ab,ac)local az,aA;local _,aB,aC;_=ab/(2*ay)aB=ac/ay;aC=_*_-aB;if MathUtil.isZero(aC)then az=-_;return az elseif aC<0 then return else local aD=math.sqrt(aC)az=aD-_;aA=-aD-_;return az,aA end end;function MathUtil.solveCubic(ay,ab,ac,aE)local az,aA,aF;local aG,aH;local a5,a6,a7;local aI,_,aB;local aJ,aC;a5=ab/ay;a6=ac/ay;a7=aE/ay;aI=a5*a5;_=1/3*(-(1/3)*aI+a6)aB=0.5*(2/27*a5*aI-1/3*a5*a6+a7)aJ=_*_*_;aC=aB*aB+aJ;if MathUtil.isZero(aC)then if MathUtil.isZero(aB)then az=0;aG=1 else local aK=MathUtil.cuberoot(-aB)az=2*aK;aA=-aK;aG=2 end elseif aC<0 then local aL=1/3*math.acos(-aB/math.sqrt(-aJ))local g=2*math.sqrt(-_)az=g*math.cos(aL)aA=-g*math.cos(aL+math.pi/3)aF=-g*math.cos(aL-math.pi/3)aG=3 else local aD=math.sqrt(aC)local aK=MathUtil.cuberoot(aD-aB)local O=-MathUtil.cuberoot(aD+aB)az=aK+O;aG=1 end;aH=1/3*a5;if aG>0 then az=az-aH end;if aG>1 then aA=aA-aH end;if aG>2 then aF=aF-aH end;return az,aA,aF end;function MathUtil.solveQuartic(ay,ab,ac,aE,aM)local az,aA,aF,aN;local aO={}local S,aK,O,aH;local a5,a6,a7,aC;local aI,_,aB,a3;local aG=0;a5=ab/ay;a6=ac/ay;a7=aE/ay;aC=aM/ay;aI=a5*a5;_=-0.375*aI+a6;aB=0.125*aI*a5-0.5*a5*a6+a7;a3=-(3/256)*aI*aI+0.0625*aI*a6-0.25*a5*a7+aC;if MathUtil.isZero(a3)then aO[3]=aB;aO[2]=_;aO[1]=0;aO[0]=1;local aP={MathUtil.solveCubic(aO[0],aO[1],aO[2],aO[3])}aG=#aP;az,aA,aF=aP[1],aP[2],aP[3]elseif MathUtil.isZero(aB)then local aQ={MathUtil.solveQuadratic(1,_,a3)}if aQ[1]>=0 then az=-math.sqrt(aQ[1])aA=math.sqrt(aQ[1])aG=2 end;if aQ[2]>=0 then if aG==0 then az=-math.sqrt(aQ[2])aA=math.sqrt(aQ[2])aG=2 else aF=-math.sqrt(aQ[2])aN=math.sqrt(aQ[2])aG=4 end end else aO[3]=0.5*a3*_-0.125*aB*aB;aO[2]=-a3;aO[1]=-0.5*_;aO[0]=1;az,aA,aF=MathUtil.solveCubic(aO[0],aO[1],aO[2],aO[3])S=az;aK=S*S-a3;O=2*S-_;if MathUtil.isZero(aK)then aK=0 elseif aK>0 then aK=math.sqrt(aK)else return end;if MathUtil.isZero(O)then O=0 elseif O>0 then O=math.sqrt(O)else return end;aO[2]=S-aK;aO[1]=aB<0 and-O or O;aO[0]=1;do local aP={MathUtil.solveQuadratic(aO[0],aO[1],aO[2])}aG=#aP;az,aA=aP[1],aP[2]end;aO[2]=S+aK;aO[1]=aB<0 and O or-O;aO[0]=1;if aG==0 then local aP={MathUtil.solveQuadratic(aO[0],aO[1],aO[2])}aG=aG+#aP;az,aA=aP[1],aP[2]end;if aG==1 then local aP={MathUtil.solveQuadratic(aO[0],aO[1],aO[2])}aG=aG+#aP;aA,aF=aP[1],aP[2]end;if aG==2 then local aP={MathUtil.solveQuadratic(aO[0],aO[1],aO[2])}aG=aG+#aP;aF,aN=aP[1],aP[2]end end;aH=0.25*a5;if aG>0 then az=az-aH end;if aG>1 then aA=aA-aH end;if aG>2 then aF=aF-aH end;if aG>3 then aN=aN-aH end;return az,aA,aF,aN end;function RingBuffer.RingBuffer(aR)local aS={}aS.buf={}aS.capacity=aR;aS.size=0;aS.head=1;local aT=getmetatable(aS)or{}aT.__index=RingBuffer.get;setmetatable(aS,aT)return aS end;function RingBuffer.isFull(aS)return aS.size>=aS.capacity end;function RingBuffer.setSize(aS,aU)aS.size=aU end;function RingBuffer.push(aS,d)aS.buf[(aS.head+aS.size-1)%aS.capacity+1]=d;if aS.size==aS.capacity then aS.head=aS.head%aS.capacity+1 else aS.size=aS.size+1 end end;function RingBuffer.pop(aS)if aS.size==0 then return nil end;local m=aS.buf[aS.head]aS.buf[aS.head]=nil;aS.head=aS.head%aS.capacity+1;aS.size=aS.size-1;return m end;function RingBuffer.get(aS,aV)if type(aV)~="number"or math.floor(aV)~=aV then return nil end;if aV<1 or aV>aS.size then return nil end;return aS.buf[(aS.head+aV-2)%aS.capacity+1]end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(y,z)local aW=type(y)=="int"local aX=type(z)=="int"if not aW and aX then return z+y end;if aW and not aX then return MathUtil.combine(y,z,function(N,at,aY)return y+aY end)else return MathUtil.combine(y,z,function(N,at,aY)return at+aY end)end end;VectorN.mt.__sub=function(y,z)return y+-z end;VectorN.mt.__mul=function(y,z)local aW=type(y)=="int"local aX=type(z)=="int"if not aW and aX then return z*y end;if aW and not aX then local M={}for N,O in pairs(z)do M[N]=y*O end;return M else return MathUtil.combine(y,z,function(N,at,aY)return at*aY end)end end;VectorN.mt.__div=function(y,z)local aW=type(y)=="int"local aX=type(z)=="int"if not aW and aX then return y*1/z end;if aW and not aX then local M={}for N,O in pairs(z)do M[N]=y/O end;return M else return MathUtil.combine(y,z,function(N,at,aY)return at/aY end)end end;VectorN.mt.__unm=function(y)local M={}for N,O in pairs(y)do M[N]=-O end;return M end;function VectorN.VectorN(l)local aZ={}for N,O in pairs(l)do if type(O)=="table"then aZ[N]=VectorN.VectorN(O)else aZ[N]=O end end;setmetatable(aZ,VectorN.mt)return aZ end;function Control.PID(a_,b0,b1,b2,b3,b4)local b5={}b5.kP=a_;b5.kI=b0;b5.kD=b1;b5.Iacc=Accumulator.Accumulator(b2,b3)if b4 and b4~=0 then b5.period=b4 end;return b5 end;function Control.processPID(b6,b7,e)b7=b6.period and(b7+b6.period/2)%b6.period-b6.period/2 or b7;local _=b6.kP*b7;local p,b8=b6.kI*Accumulator.update(b6.Iacc,b7,e)p=p/b8;local h=b6.kD*(b7-(b6.lastError or b7))/e;b6.lastError=b7;return _+p+h end;function Control.FF(aO,b4)local b9={}b9.coeffs=aO;b9.degree=#aO-1;if b4 and b4~=0 then b9.period=b4 end;b9.derivs={}return b9 end;function Control.processFF(b6,ba,e)local bb=0*ba;local bc=ba;local bd=ba;for p=1,b6.degree+1 do bd=b6.derivs[p]b6.derivs[p]=bc;bb=bb+b6.coeffs[p]*bc;if bd then local be=bc-bd;if p==1 and b6.period then be=(be+b6.period/2)%b6.period-b6.period/2 end;bc=be/e else break end end;return bb end;function Nav.toLocal(bf,bg,bh)local bi=bf-bg;return Quaternion.Inverse(bh)*bi end;function Nav.toGlobal(bj,bg,bh)local bi=bh*bj;return bi+bg end;function Nav.cartToPol(bk)local a3=bk.magnitude;local a4=Vector3.SignedAngle(Vector3.forward,bk,Vector3.up)local aL=90-Vector3.Angle(Vector3.up,bk)return Vector3(a3,a4,aL)end;function Nav.cartToCyl(bk)local bl=Vector3(bk.x,0,bk.z)local bm=bl.magnitude;local aL=Vector3.SignedAngle(Vector3.forward,bk,Vector3.up)local S=bk.y;return Vector3(bm,aL,S)end;function Nav.polToCart(bk)local a3,a4,aL=bk.x,bk.y,bk.z;local at=Mathf.Sin(a4)*Mathf.Cos(aL)local aY=Mathf.Sin(aL)local S=Mathf.Cos(a4)*Mathf.Cos(aL)return a3*Vector3(at,aY,S)end;function Nav.cylToCart(bk)local bm,aL,bn=bk.x,bk.y,bk.z;local at=bm*Mathf.Sin(aL)local aY=bn;local S=bm*Mathf.Cos(aL)return Vector3(at,aY,S)end;function Targeting.firstOrderTargeting(bo,bp,bq)local br=bo-Vector3.Project(bo,bp)local bs=Vector3.Dot(bp,bo-br)/bp.sqrMagnitude;local y,z=MathUtil.solveQuadratic(bs-bq*bq,2*bs,br.sqrMagnitude+bs*bs)local bt=nil;if y and y>=0 then bt=y end;if z and z>=0 and z<y then bt=z end;return bt and(bo+bt*bp).normalized or nil end;function Targeting.secondOrderTargeting(bo,bu,bv,bq,bw,bx)local g=Targeting.secondOrderTargetingTime(bo,bu,bv,bq,bw/bq,bx/bq)if g and g>0 then return(bo/g+bu+0.5*bv*g).normalized end;return nil end;function Targeting.secondOrderTargetingTime(bo,bu,bv,bq,by,bz)local y=0.25*bv.sqrMagnitude;local z=Vector3.Dot(bu,bv)local C=bu.sqrMagnitude-bq*bq+Vector3.Dot(bo,bv)local h=2*Vector3.Dot(bo,bu)local b7=bo.sqrMagnitude;local function bA(at)local bB=at*at;return b7+h*at+C*bB+z*bB*at+y*bB*bB end;local function bC(at)local bB=at*at;return h+2*C*at+3*z*bB+4*y*bB*at end;local bD=bo.magnitude;local bE=bq-h/(2*bD)local g=MathUtil.newton(bA,bC,bD/bE,0.1,10)if g and g>by and g<bz then return g end end;function Targeting.AIPPN(bF,bo,bG,bp,bH)local bu=bp-bG;local bI=Vector3.Dot(-bu,bo.normalized)if bI<=0 then bI=10 end;local bJ=bo.magnitude/bI;local bK=Vector3.Cross(bo,bu)/bo.sqrMagnitude;local bL=Vector3.Cross(bo,bH)/bo.sqrMagnitude*bJ/2;local bM=bK+bL;local bN=Vector3.Cross(bM,bo.normalized)local bO=Vector3.ProjectOnPlane(bN,bG).normalized;local bP=bF*bG.magnitude*bM.magnitude;return bP*bO end;function Targeting.ATPN(bF,bo,bG,bp,bH)local bu=bp-bG;local bI=-Vector3.Dot(bu,bo.normalized)if bI<=0 then bI=10 end;local bK=Vector3.Cross(bo,bu)/bo.sqrMagnitude;local bN=Vector3.Cross(bK,bo.normalized)local bQ=Vector3.ProjectOnPlane(bH,bo)return bF*bI*bN+0.5*bF*bH end;function BlockUtil.getWeaponsByName(bR,bS,bT,bU)if DEBUG then bR:Log("searching for "..bS)end;local bV=bR:GetAllSubConstructs()local bW={}bT=bT or-1;local C=bT;if not bU or bU==0 or bU==2 then for p=0,bR:GetWeaponCount()-1 do if C==0 then break end;if bR:GetWeaponBlockInfo(p).CustomName==bS then table.insert(bW,{subIdx=nil,wpnIdx=p})if DEBUG then bR:Log("found weapon "..bS.." on hull, type "..bR:GetWeaponInfo(p).WeaponType)end;C=C-1 end end end;if not bU or bU==1 or bU==2 then for aV=1,#bV do local aH=bV[aV]for p=0,bR:GetWeaponCountOnSubConstruct(aH)-1 do if C==0 then break end;if bR:GetWeaponBlockInfoOnSubConstruct(aH,p).CustomName==bS then table.insert(bW,{subIdx=aH,wpnIdx=p})if DEBUG then bR:Log("found weapon "..bS.." on subobj "..aH..", type "..bR:GetWeaponInfo(p).WeaponType)end;C=C-1 end end end end;if DEBUG then bR:Log("weapon count: "..#bW)end;return bW end;function BlockUtil.getSubConstructsByName(bR,bS,bT)if DEBUG then bR:Log("searching for "..bS)end;local bV=bR:GetAllSubConstructs()local bX={}bT=bT or-1;local C=bT;for aV=1,#bV do local aH=bV[aV]if C==0 then break end;if bR:GetSubConstructInfo(aH).CustomName==bS then table.insert(bX,aH)if DEBUG then bR:Log("found subobj "..bS)end;C=C-1 end end;if DEBUG then bR:Log("subobj count: "..#bX)end;return bX end;function BlockUtil.getBlocksByName(bR,bS,type,bT)if DEBUG then bR:Log("searching for "..bS)end;local bY={}bT=bT or-1;local C=bT;for aV=0,bR:Component_GetCount(type)-1 do if C==0 then break end;if bR:Component_GetBlockInfo(type,aV).CustomName==bS then table.insert(bY,aV)if DEBUG then bR:Log("found component "..bS)end;C=C-1 end end;if DEBUG then bR:Log("component count: "..#bY)end;return bY end;function BlockUtil.getWeaponInfo(bR,bZ)local b_;if bZ.subIdx then b_=bR:GetWeaponInfoOnSubConstruct(bZ.subIdx,bZ.wpnIdx)else b_=bR:GetWeaponInfo(bZ.wpnIdx)end;return b_ end;function BlockUtil.aimWeapon(bR,bZ,c0,c1)if bZ.subIdx then bR:AimWeaponInDirectionOnSubConstruct(bZ.subIdx,bZ.wpnIdx,c0.x,c0.y,c0.z,c1)else bR:AimWeaponInDirection(bZ.wpnIdx,c0.x,c0.y,c0.z,c1)end end;function BlockUtil.fireWeapon(bR,bZ,c1)if bZ.subIdx then bR:FireWeaponOnSubConstruct(bZ.subIdx,bZ.wpnIdx,c1)else bR:FireWeapon(bZ.wpnIdx,c1)end end;function Combat.pickTarget(bR,c2,c3)c3=c3 or function(G,c4)return c4.Priority end;local ba,c5;for p in MathUtil.range(bR:GetNumberOfTargets(c2))do local c4=bR:GetTargetInfo(c2,p)local c6=c3(bR,c4)if not ba or c6>c5 then ba=c4;c5=c6 end end;return ba end;function Combat.CheckConstraints(bR,c7,c8,c9)local ca;if c9 then ca=bR:GetWeaponConstraintsOnSubConstruct(c9,c8)else ca=bR:GetWeaponConstraints(c8)end;local cb=bR:GetConstructForwardVector()local cc=bR:GetConstructUpVector()local cd=Quaternion.LookRotation(cb,cc)c7=Quaternion.Inverse(cd)*c7;if ca.InParentConstructSpace and c9 then local ce=bR:GetSubConstructInfo(c9).localRotation;c7=Quaternion.inverse(ce)*c7 end;local cf=MathUtil.angleOnPlane(Vector3.forward,c7,Vector3.up)local cg=c7;cg.z=0;local A=Mathf.Atan2(c7.z,cg.magnitude)local ch=cf>ca.MinAzimuth and cf<ca.MaxAzimuth;local ci=A>ca.MinElevation and A<ca.MaxElevation;if ca.FlipAzimuth then ch=not ch end;if ch and ci then return true end;cf=cf+180;A=180-A;if A>180 then A=A-360 end;if A<-180 then A=A+360 end;ch=cf>ca.MinAzimuth and cf<ca.MaxAzimuth;ci=A>ca.MinElevation and A<ca.MaxElevation;if ca.FlipAzimuth then ch=not ch end;if ch and ci then return true end;return false end;function StringUtil.LogVector(bR,aZ,cj)bR:Log(cj.."("..aZ.x..", "..aZ.y..", "..aZ.z..")")end
