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
-- todo: take in origin and aimpoint, return list of locations to attempt
local function aimOffset(fp)
  return Vector3(0, 0, 0)
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
local AIM_TOL = 0.1
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

-- minified version of Tides library (not meant to be human-readable, see Tides.lua or individual class files for human-readable source)
function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function LinkedList.LinkedList()local k={}k.value=nil;k.next=k;k.prev=k;return k end;function LinkedList.pushFront(l,d)local k={}k.value=d;LinkedList.connect(k,l.next)LinkedList.connect(l,k)end;function LinkedList.pushBack(l,d)local k={}k.value=d;LinkedList.connect(l.prev,k)LinkedList.connect(k,l)end;function LinkedList.popFront(l)local m=l.next.value;LinkedList.connect(l,l.next.next)return m end;function LinkedList.popBack(l)local m=l.prev.value;LinkedList.connect(l.prev.prev,l)return m end;function LinkedList.peekFront(l)return l.next.val end;function LinkedList.peekBack(l)return l.prev.val end;function LinkedList.connect(n,o)n.next=o;o.prev=n end;function LinkedList.toArray(l)local p=1;local q={}local k=l.next;while k~=l do q[p]=k.value;k=k.next end;return q end;function MathUtil.angleOnPlane(r,s,t)local u=Vector3.ProjectOnPlane(r,t)local g=Vector3.ProjectOnPlane(s,t)return Vector3.SignedAngle(u,g,t)end;function MathUtil.min(v,w)local x=nil;w=w or function(y,z)return y<z end;for A in v do if not x or w(A,x)then x=A end end;return x end;function MathUtil.max(v,w)local B=nil;w=w or function(y,z)return y<z end;for A in v do if not B or w(B,A)then B=A end end;return B end;function MathUtil.range(y,z,C)local D,E=y,z;local F;if not y then return end;if not z then D=0;E=y;F=D<E and 1 or-1 elseif C then F=C end;return function(G,H)local I=H+F;if I==E then return nil end;return I end,nil,D-F end;function MathUtil.shuffle(l)local J={}for p=1,#l do J[p]=l[p]end;for p=#l,2,-1 do local K=math.random(p)J[p],J[K]=J[K],J[p]end;return J end;function MathUtil.combine(y,z,L)if#y==#z then local M={}for N,O in pairs(y)do M[N]=L(N,O,z[N])end;return M end end;function MathUtil.distribution()return{n=0}end;function MathUtil.updateDistribution(P,Q)P.n=P.n+1;if P.n==1 then P.mean=Q;P.covariance={}local h=#Q;for p=1,h do local R={}for K=1,h do R[K]=0 end;P.covariance[p]=R end else P.mean=P.mean+1/(P.n+1)*Q end end;function MathUtil.mean(P)return P.mean end;function MathUtil.covariance(P)return P.cov end;function MathUtil.normal()local S,T=MathUtil.boxMuller()return S end;function MathUtil.normalPDF(S)return math.exp(-0.5*S*S)/math.sqrt(2*math.pi)end;function MathUtil.normalCDF(S)local U=0.2316419;local V=0.319381530;local W=-0.356563782;local X=1.781477937;local Y=-1.821255978;local Z=1.330274429;local g=1/(1+U*S)return 1-MathUtil.normalPDF(S)*(V*g+W*g^2+X*g^3+Y*g^4+Z*g^5)end;function MathUtil.inverseNorm(_)local a0=_>=0.5 and _ or-_;local S=5.55556*(1-((1-a0)/a0)^0.1186)if _<0.5 then S=-S end;return S end;function MathUtil.boxMuller()local a1=math.random()local a2=math.random()a2=math.random()a2=math.random()local a3=math.sqrt(-2*math.log(a1))local a4=2*math.pi*a2;return a3*math.cos(a4),a3*math.sin(a4)end;function MathUtil.angleSSS(y,z,C)if y+z<C or y+C<z or z+C<y then return nil end;local a5=math.deg(math.acos((z*z+C*C-y*y)/(2*z*C)))local a6,a7=MathUtil.angleSAS(z,a5,C)return a5,a6,a7 end;function MathUtil.sideSAS(y,a7,z)local a8=y*y+z*z-2*y*z*math.cos(math.rad(a7))return math.sqrt(a8)end;function MathUtil.angleSAS(y,a7,z)local C=MathUtil.sideSAS(y,a7,z)if MathUtil.isZero(C)then return nil end;local a5,a6;if y<z then a5=MathUtil.angleLoSin(C,y,a7)a6=180-a5-a7 else a6=MathUtil.angleLoSin(C,z,a7)a5=180-a6-a7 end;return a5,a6 end;function MathUtil.sideSSA(y,z,a5)local a9=z*z-y*y;local aa=-2*z*math.cos(math.rad(a5))local ab,ac=MathUtil.solveQuadratic(1,aa,a9)if not ac then return ab,ac end;if ab<ac then return ab,ac end;return ac,ab end;function MathUtil.angleSSA(y,z,a5)local ab,ac=MathUtil.sideSSA(y,z,a5)if not ab then return nil end;local ad,ae=MathUtil.angleSAS(z,a5,ab)if not ac then return ad,ae end;local af,ag=MathUtil.angleSAS(z,a5,ac)return ad,ae,af,ag end;function MathUtil.sideAAS(a5,a6,y)local a7=180-a5-a6;local z=MathUtil.sideLoSin(a5,a6,y)local C=MathUtil.sideLoSin(a5,a7,y)return z,C end;function MathUtil.sideLoSin(y,a5,a6)return y*math.sin(math.rad(a6))/math.sin(math.rad(a5))end;function MathUtil.angleLoSin(y,z,a5)return math.deg(math.asin(z*math.sin(math.rad(a5))/y))end;function MathUtil.clampCone(ah,ai,aj)local ak=math.min(aj,Vector3.Angle(ah,ai))local al=Vector3.Cross(ah,ai)return Quaternion.AngleAxis(ak,al)*ah end;function MathUtil.fourier(am)return"Work in progress"end;function MathUtil.newton(an,ao,ap,aq,ar,as)aq=aq or 1e-5;as=as or 10*aq;ar=ar or 25;ao=ao or function(at)return(an(at+as)-an(at))/as end;ap=ap or 0;local au=aq+1;local av=0;while au>aq and av<ar do local aw=an(ap)local ax=ao(ap)if not aw or not ax then return nil end;au=-aw/ax;ap=ap+au;av=av+1 end;if av<ar then return ap,false end;return ap,true end;function MathUtil.ITP(an,y,z,aq,ar)aq=aq or 1e-5;local ay=math.ceil(math.log((z-y)/aq,2))ar=ar or 25;local az=0.2/(z-y)local aA=2;local aB=1;local K=0;while z-y>2*aq and K<ar do local aC=(y+z)/2;local aD=(z*an(y)-y*an(z))/(an(y)-an(z))local aE=aC-aD;local aF=az*math.abs(z-y)^aA;local aG=aE>0 and 1 or(aE==0 and 0 or-1)local aH=aF<=math.abs(aE)and aD+aG*aF or aC;local aI=aq*2^(ay+aB-K)-(z-y)/2;local aJ=aI<math.abs(aH-aC)and aC-aG*aI or aH;local aK=an(aJ)if aK>0 then z=aJ elseif aK<0 then y=aJ else y=aJ;z=aJ end;K=K+1 end;return(y+z)/2,K==ar end;function MathUtil.binomCoeffs(aL,aM)if aM then coeffs={}else coeffs={}coeffs[1]=1;for N=1,aL do coeffs[N+1]=coeffs[N]*(aL-N)/(N+1)end;return coeffs end end;function MathUtil.ruleOfSigns(coeffs,aN)local aO={}local aP=#coeffs;for p=1,aP do aO[p]=coeffs[aP-p+1]end;if aN~=0 then local aQ={}for p=1,aP do aQ[p]=(p-1)*coeffs[aP-p+1]end;local aR=1;for p=2,aP do local aS=aN^(p-1)for K=1,aP-p+1 do local aT=p+K-1;aO[K]=aO[K]+aR*aQ[aT]*aS;aQ[aT]=aQ[aT]*(K-1)end;aR=aR/p end end;local aU={}local aV=1;for p,aW in ipairs(aO)do if aW~=0 then aU[aV]=aW;aV=aV+1 end end;local aX=0;for p=1,#aU-1 do if aU[p]*aU[p+1]<0 then aX=aX+1 end end;return aX end;function MathUtil._factorial(aY,aV)local m=aY[aY.size]for p=aY.size+1,aV do m=m*p;aY[p]=m end;aY.size=aV;return m end;MathUtil._factorialCache={1,size=1}MathUtil._factorialMt=getmetatable(MathUtil._factorialCache)or{}MathUtil._factorialMt.__index=MathUtil._factorial;setmetatable(MathUtil._factorialCache,MathUtil._factorialMt)function MathUtil.factorial(aV)return Mathutil._factorialCache[aV]end;MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(aq)MathUtil.eps=aq end;function MathUtil.cuberoot(at)return at>0 and at^(1/3)or-math.abs(at)^(1/3)end;function MathUtil.solveQuadratic(aZ,ab,ac)local a_,b0;local _,b1,b2;_=ab/(2*aZ)b1=ac/aZ;b2=_*_-b1;if MathUtil.isZero(b2)then a_=-_;return a_ elseif b2<0 then return else local b3=math.sqrt(b2)a_=b3-_;b0=-b3-_;return a_,b0 end end;function MathUtil.solveCubic(aZ,ab,ac,b4)local a_,b0,b5;local b6,b7;local a5,a6,a7;local b8,_,b1;local b9,b2;a5=ab/aZ;a6=ac/aZ;a7=b4/aZ;b8=a5*a5;_=1/3*(-(1/3)*b8+a6)b1=0.5*(2/27*a5*b8-1/3*a5*a6+a7)b9=_*_*_;b2=b1*b1+b9;if MathUtil.isZero(b2)then if MathUtil.isZero(b1)then a_=0;b6=1 else local ba=MathUtil.cuberoot(-b1)a_=2*ba;b0=-ba;b6=2 end elseif b2<0 then local bb=1/3*math.acos(-b1/math.sqrt(-b9))local g=2*math.sqrt(-_)a_=g*math.cos(bb)b0=-g*math.cos(bb+math.pi/3)b5=-g*math.cos(bb-math.pi/3)b6=3 else local b3=math.sqrt(b2)local ba=MathUtil.cuberoot(b3-b1)local O=-MathUtil.cuberoot(b3+b1)a_=ba+O;b6=1 end;b7=1/3*a5;if b6>0 then a_=a_-b7 end;if b6>1 then b0=b0-b7 end;if b6>2 then b5=b5-b7 end;return a_,b0,b5 end;function MathUtil.solveQuartic(aZ,ab,ac,b4,bc)local a_,b0,b5,bd;local coeffs={}local S,ba,O,b7;local a5,a6,a7,b2;local b8,_,b1,a3;local b6=0;a5=ab/aZ;a6=ac/aZ;a7=b4/aZ;b2=bc/aZ;b8=a5*a5;_=-0.375*b8+a6;b1=0.125*b8*a5-0.5*a5*a6+a7;a3=-(3/256)*b8*b8+0.0625*b8*a6-0.25*a5*a7+b2;if MathUtil.isZero(a3)then coeffs[3]=b1;coeffs[2]=_;coeffs[1]=0;coeffs[0]=1;local be={MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])}b6=#be;a_,b0,b5=be[1],be[2],be[3]elseif MathUtil.isZero(b1)then local bf={MathUtil.solveQuadratic(1,_,a3)}if bf[1]>=0 then a_=-math.sqrt(bf[1])b0=math.sqrt(bf[1])b6=2 end;if bf[2]>=0 then if b6==0 then a_=-math.sqrt(bf[2])b0=math.sqrt(bf[2])b6=2 else b5=-math.sqrt(bf[2])bd=math.sqrt(bf[2])b6=4 end end else coeffs[3]=0.5*a3*_-0.125*b1*b1;coeffs[2]=-a3;coeffs[1]=-0.5*_;coeffs[0]=1;a_,b0,b5=MathUtil.solveCubic(coeffs[0],coeffs[1],coeffs[2],coeffs[3])S=a_;ba=S*S-a3;O=2*S-_;if MathUtil.isZero(ba)then ba=0 elseif ba>0 then ba=math.sqrt(ba)else return end;if MathUtil.isZero(O)then O=0 elseif O>0 then O=math.sqrt(O)else return end;coeffs[2]=S-ba;coeffs[1]=b1<0 and-O or O;coeffs[0]=1;do local be={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}b6=#be;a_,b0=be[1],be[2]end;coeffs[2]=S+ba;coeffs[1]=b1<0 and O or-O;coeffs[0]=1;if b6==0 then local be={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}b6=b6+#be;a_,b0=be[1],be[2]end;if b6==1 then local be={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}b6=b6+#be;b0,b5=be[1],be[2]end;if b6==2 then local be={MathUtil.solveQuadratic(coeffs[0],coeffs[1],coeffs[2])}b6=b6+#be;b5,bd=be[1],be[2]end end;b7=0.25*a5;if b6>0 then a_=a_-b7 end;if b6>1 then b0=b0-b7 end;if b6>2 then b5=b5-b7 end;if b6>3 then bd=bd-b7 end;return a_,b0,b5,bd end;function RingBuffer.RingBuffer(bg)local bh={}bh.buf={}bh.capacity=bg;bh.size=0;bh.head=1;local bi=getmetatable(bh)or{}bi.__index=RingBuffer.get;setmetatable(bh,bi)return bh end;function RingBuffer.isFull(bh)return bh.size>=bh.capacity end;function RingBuffer.setSize(bh,bj)bh.size=bj end;function RingBuffer.push(bh,d)bh.buf[(bh.head+bh.size-1)%bh.capacity+1]=d;if bh.size==bh.capacity then bh.head=bh.head%bh.capacity+1 else bh.size=bh.size+1 end end;function RingBuffer.pop(bh)if bh.size==0 then return nil end;local m=bh.buf[bh.head]bh.buf[bh.head]=nil;bh.head=bh.head%bh.capacity+1;bh.size=bh.size-1;return m end;function RingBuffer.get(bh,bk)if type(bk)~="number"or math.floor(bk)~=bk then return nil end;if bk<1 or bk>bh.size then return nil end;return bh.buf[(bh.head+bk-2)%bh.capacity+1]end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(y,z)local bl=type(y)=="number"local bm=type(z)=="number"if not bl and bm then return z+y end;if bl and not bm then return MathUtil.combine(y,z,function(N,at,bn)return y+bn end)else return MathUtil.combine(y,z,function(N,at,bn)return at+bn end)end end;VectorN.mt.__sub=function(y,z)return y+-z end;VectorN.mt.__mul=function(y,z)local bl=type(y)=="number"local bm=type(z)=="number"if not bl and bm then return z*y end;if bl and not bm then local M={}for N,O in pairs(z)do M[N]=y*O end;return M else return MathUtil.combine(y,z,function(N,at,bn)return at*bn end)end end;VectorN.mt.__div=function(y,z)local bl=type(y)=="number"local bm=type(z)=="number"if not bl and bm then return y*1/z end;if bl and not bm then local M={}for N,O in pairs(z)do M[N]=y/O end;return M else return MathUtil.combine(y,z,function(N,at,bn)return at/bn end)end end;VectorN.mt.__unm=function(y)local M={}for N,O in pairs(y)do M[N]=-O end;return M end;function VectorN.VectorN(l)local bo={}for N,O in pairs(l)do if type(O)=="table"then bo[N]=VectorN.VectorN(O)else bo[N]=O end end;setmetatable(bo,VectorN.mt)return bo end;function Control.PID(bp,bq,br,bs,bt,bu)local bv={}bv.kP=bp;bv.kI=bq;bv.kD=br;bv.Iacc=Accumulator.Accumulator(bs,bt)if bu and bu~=0 then bv.period=bu end;return bv end;function Control.processPID(bw,bx,e)bx=bw.period and(bx+bw.period/2)%bw.period-bw.period/2 or bx;local _=bw.kP*bx;local p,by=bw.kI*Accumulator.update(bw.Iacc,bx,e)p=p/by;local h=bw.kD*(bx-(bw.lastError or bx))/e;bw.lastError=bx;return _+p+h end;function Control.FF(coeffs,bu)local bz={}bz.coeffs=coeffs;bz.degree=#coeffs-1;if bu and bu~=0 then bz.period=bu end;bz.derivs={}return bz end;function Control.processFF(bw,bA,e)local bB=0*bA;local bC=bA;local bD=bA;for p=1,bw.degree+1 do bD=bw.derivs[p]bw.derivs[p]=bC;bB=bB+bw.coeffs[p]*bC;if bD then local aE=bC-bD;if p==1 and bw.period then aE=(aE+bw.period/2)%bw.period-bw.period/2 end;bC=aE/e else break end end;return bB end;function Nav.toLocal(bE,bF,bG)local bH=bE-bF;return Quaternion.Inverse(bG)*bH end;function Nav.toGlobal(bI,bF,bG)local bH=bG*bI;return bH+bF end;function Nav.cartToPol(bJ)local a3=bJ.magnitude;local a4=Vector3.SignedAngle(Vector3.forward,bJ,Vector3.up)local bb=90-Vector3.Angle(Vector3.up,bJ)return Vector3(a3,a4,bb)end;function Nav.cartToCyl(bJ)local bK=Vector3(bJ.x,0,bJ.z)local bL=bK.magnitude;local bb=Vector3.SignedAngle(Vector3.forward,bJ,Vector3.up)local S=bJ.y;return Vector3(bL,bb,S)end;function Nav.polToCart(bJ)local a3,a4,bb=bJ.x,bJ.y,bJ.z;local at=Mathf.Sin(a4)*Mathf.Cos(bb)local bn=Mathf.Sin(bb)local S=Mathf.Cos(a4)*Mathf.Cos(bb)return a3*Vector3(at,bn,S)end;function Nav.cylToCart(bJ)local bL,bb,bM=bJ.x,bJ.y,bJ.z;local at=bL*Mathf.Sin(bb)local bn=bM;local S=bL*Mathf.Cos(bb)return Vector3(at,bn,S)end;function Targeting.firstOrderTargeting(bN,bO,bP)local bQ=bN-Vector3.Project(bN,bO)local bR=Vector3.Dot(bO,bN-bQ)/bO.sqrMagnitude;local y,z=MathUtil.solveQuadratic(bR-bP*bP,2*bR,bQ.sqrMagnitude+bR*bR)local bS=nil;if y and y>=0 then bS=y end;if z and z>=0 and z<y then bS=z end;return bS and(bN+bS*bO).normalized or nil end;function Targeting.secondOrderTargeting(bN,bT,bU,bP,bV,bW)local y=0.25*bU.sqrMagnitude;local z=Vector3.Dot(bT,bU)local C=bT.sqrMagnitude-bP*bP+Vector3.Dot(bN,bU)local h=2*Vector3.Dot(bN,bT)local bx=bN.sqrMagnitude;local g;local bX=bU.magnitude;local bY=bT.magnitude;local bZ=bN.magnitude;local G,b_=MathUtil.solveQuadratic(0.5*bX,bY+bP,-bZ)local c0;local coeffs={0.5*bX,bY-bP,bZ}if MathUtil.ruleOfSigns(coeffs,0)==2 then G,c0=MathUtil.solveQuadratic(coeffs[1],coeffs[2],coeffs[3])end;if not c0 or c0<0 then local a_,G,b5=MathUtil.solveCubic(4*y,3*z,2*C,h)if a_>0 then c0=a_ elseif b5 and b5>0 then c0=b5 else return nil end end;local function c1(at)return bx+at*(h+at*(C+at*(z+at*y)))end;g=MathUtil.ITP(c1,b_,c0)local c2;if g and g>0 then c2=bN/g+bT+0.5*bU*g end;if c2.sqrMagnitude>bV*bV and c2.sqrMagnitude<bW*bW then return c2,g end end;function Targeting.AIPPN(c3,bN,c4,bO,c5)local bT=bO-c4;local c6=Vector3.Dot(-bT,bN.normalized)if c6<=0 then c6=10 end;local c7=bN.magnitude/c6;local c8=Vector3.Cross(bN,bT)/bN.sqrMagnitude;local c9=Vector3.Cross(bN,c5)/bN.sqrMagnitude*c7/2;local ca=c8+c9;local cb=Vector3.Cross(ca,bN.normalized)local cc=Vector3.ProjectOnPlane(cb,c4).normalized;local cd=c3*c4.magnitude*ca.magnitude;return cd*cc end;function Targeting.ATPN(c3,bN,c4,bO,c5)local bT=bO-c4;local c6=-Vector3.Dot(bT,bN.normalized)if c6<=0 then c6=10 end;local c8=Vector3.Cross(bN,bT)/bN.sqrMagnitude;local cb=Vector3.Cross(c8,bN.normalized)local ce=Vector3.ProjectOnPlane(c5,bN)return c3*c6*cb+0.5*c3*c5 end;function BlockUtil.getWeaponsByName(cf,cg,aX,ch)if DEBUG then cf:Log("searching for "..cg)end;local ci=cf:GetAllSubConstructs()local cj={}aX=aX or-1;local C=aX;if not ch or ch==0 or ch==2 then for p=0,cf:GetWeaponCount()-1 do if C==0 then break end;if cf:GetWeaponBlockInfo(p).CustomName==cg then table.insert(cj,{subIdx=nil,wpnIdx=p})if DEBUG then cf:Log("found weapon "..cg.." on hull, type "..cf:GetWeaponInfo(p).WeaponType)end;C=C-1 end end end;if not ch or ch==1 or ch==2 then for bk=1,#ci do local b7=ci[bk]for p=0,cf:GetWeaponCountOnSubConstruct(b7)-1 do if C==0 then break end;if cf:GetWeaponBlockInfoOnSubConstruct(b7,p).CustomName==cg then table.insert(cj,{subIdx=b7,wpnIdx=p})if DEBUG then cf:Log("found weapon "..cg.." on subobj "..b7 ..", type "..cf:GetWeaponInfo(p).WeaponType)end;C=C-1 end end end end;if DEBUG then cf:Log("weapon count: "..#cj)end;return cj end;function BlockUtil.getSubConstructsByName(cf,cg,aX)if DEBUG then cf:Log("searching for "..cg)end;local ci=cf:GetAllSubConstructs()local ck={}aX=aX or-1;local C=aX;for bk=1,#ci do local b7=ci[bk]if C==0 then break end;if cf:GetSubConstructInfo(b7).CustomName==cg then table.insert(ck,b7)if DEBUG then cf:Log("found subobj "..cg)end;C=C-1 end end;if DEBUG then cf:Log("subobj count: "..#ck)end;return ck end;function BlockUtil.getBlocksByName(cf,cg,type,aX)if DEBUG then cf:Log("searching for "..cg)end;local cl={}aX=aX or-1;local C=aX;for bk=0,cf:Component_GetCount(type)-1 do if C==0 then break end;if cf:Component_GetBlockInfo(type,bk).CustomName==cg then table.insert(cl,bk)if DEBUG then cf:Log("found component "..cg)end;C=C-1 end end;if DEBUG then cf:Log("component count: "..#cl)end;return cl end;function BlockUtil.getWeaponInfo(cf,cm)local cn;if cm.subIdx then cn=cf:GetWeaponInfoOnSubConstruct(cm.subIdx,cm.wpnIdx)else cn=cf:GetWeaponInfo(cm.wpnIdx)end;return cn end;function BlockUtil.aimWeapon(cf,cm,co,cp)if cm.subIdx then cf:AimWeaponInDirectionOnSubConstruct(cm.subIdx,cm.wpnIdx,co.x,co.y,co.z,cp)else cf:AimWeaponInDirection(cm.wpnIdx,co.x,co.y,co.z,cp)end end;function BlockUtil.fireWeapon(cf,cm,cp)if cm.subIdx then cf:FireWeaponOnSubConstruct(cm.subIdx,cm.wpnIdx,cp)else cf:FireWeapon(cm.wpnIdx,cp)end end;function Combat.pickTarget(cf,cq,cr)cr=cr or function(G,cs)return cs.Priority end;local bA,ct;for p in MathUtil.range(cf:GetNumberOfTargets(cq))do local cs=cf:GetTargetInfo(cq,p)local cu=cr(cf,cs)if not bA or cu>ct then bA=cs;ct=cu end end;return bA end;function Combat.CheckConstraints(cf,cv,cw,cx)local cy;if cx then cy=cf:GetWeaponConstraintsOnSubConstruct(cx,cw)else cy=cf:GetWeaponConstraints(cw)end;local cz=cf:GetConstructForwardVector()local cA=cf:GetConstructUpVector()local cB=Quaternion.LookRotation(cz,cA)cv=Quaternion.Inverse(cB)*cv;if cy.InParentConstructSpace and cx then local cC=cf:GetSubConstructInfo(cx).localRotation;cv=Quaternion.inverse(cC)*cv end;local cD=MathUtil.angleOnPlane(Vector3.forward,cv,Vector3.up)local cE=cv;cE.z=0;local A=Mathf.Atan2(cv.z,cE.magnitude)local cF=cD>cy.MinAzimuth and cD<cy.MaxAzimuth;local cG=A>cy.MinElevation and A<cy.MaxElevation;if cy.FlipAzimuth then cF=not cF end;if cF and cG then return true end;cD=cD+180;A=180-A;if A>180 then A=A-360 end;if A<-180 then A=A+360 end;cF=cD>cy.MinAzimuth and cD<cy.MaxAzimuth;cG=A>cy.MinElevation and A<cy.MaxElevation;if cy.FlipAzimuth then cF=not cF end;if cF and cG then return true end;return false end;function StringUtil.LogVector(cf,bo,cH)cf:Log(cH.."("..bo.x..", "..bo.y..", "..bo.z..")")end
