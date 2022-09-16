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
  -- enemy: aim at current target
  -- fire: aim and fire at current target
  -- none: return to idle position
  -- last: continue aiming at last absolute bearing
local idleAim = "enemy"
-- degrees of inaccuracy allowed when firing
-- weapon will start firing within this angle
-- but will always try to obtain perfect accuracy
local AIM_TOL = 0.1
-- physics ticks per second (Lua runs in sync with game physics)
local TICKS_PER_S = 40

local projectilePos
local times
local enemies
local currentLine
local lastFrameTime
local inited
local prevTime
local lastOrigin
local lastOriginSwitchTime
local lastAim
local turrets = {}
local velocities = {}

local BlockUtil = {}
local Combat = {}
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
  inited = true
end

function Update(I)
  if not inited then Init(I) end
  for tarIdx = 0, I:GetNumberOfTargets(0) do
    local target = I:GetTargetInfo(0, tarIdx)
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

  -- compare to past projectile locations
  -- to make sure we're tracking the same projectile

  if projectilePos.size == 1 then return end
  -- see if it matches current line
  -- todo: store multiple lines and find match

  -- 3/4ths the estimated drop in two frames due to gravity
  -- inconsistent with theoretical formula due to discrete integration
  -- powered missiles have no gravity so their expected error
  -- is the negative of the drop due to gravity
  local eps = 15 * frameTime * frameTime
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
  local target = I:GetTargetInfo(0, 0)
  if target and target.Valid then
    local enemy = enemies[target.Id]
    local fp = lastOrigin
    if not fp or not lastOriginSwitchTime or I:GetTimeSinceSpawn() - lastOriginSwitchTime > originSwitchTime then
      while enemy.origins.size > 0 and I:GetTimeSinceSpawn() - enemy.originTimes[1] > maxStaleness do
        RingBuffer.pop(enemy.origins)
        RingBuffer.pop(enemy.originTimes)
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
      lastOriginSwitchTime = I:GetTimeSinceSpawn()
      lastOrigin = fp
    end
    local aim
    for i, turret in ipairs(turrets) do
      for j, weapon in ipairs(turret) do
        local wInfo = BlockUtil.getWeaponInfo(I, weapon)
        if not fp and idleAim == "fire" then
          fp = target.AimPointPosition - target.Position
        end
        if fp then
          if velocities[i] == math.huge then
            local range = (fp + target.Position - I:GetConstructPosition()).magnitude
            if range > minRange and range < maxRange then
              aim = fp + target.Position - wInfo.GlobalFirePoint
            end
          else
            aim = Targeting.secondOrderTargeting(fp + target.Position - wInfo.GlobalFirePoint,
                        target.Velocity - I:GetVelocityVector(),
                        -I:GetGravityForAltitude(target.Position.y),
                        velocities[i], minRange, maxRange)
          end
        end
        if not aim then
          if idleAim == "enemy" or idleAim == "fire" then
            aim = target.Position - wInfo.GlobalFirePoint
          elseif idleAim == "last" then
            aim = lastAim
          end
        end
        if aim then
          lastAim = aim
          if Combat.CheckConstraints(I, aim, weapon.wpnIdx, weapon.subIdx) then
            BlockUtil.aimWeapon(I, weapon, aim, 0)
            if fp and Vector3.Angle(wInfo.CurrentDirection, aim) < AIM_TOL then
              BlockUtil.fireWeapon(I, weapon, 0)
            end
          end
        end
      end
    end
  end
end

function LogVector(I, vec, label)
  I:Log(label.."("..vec.x..", "..vec.y..", "..vec.z..")")
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
      local firstSqrDist = SqrDistance(firstRoot)
      local lastSqrDist = SqrDistance(lastRoot)
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
    local dt = times[tIdxClosest] - line.tStart
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
  local vi = line.ds / line.dt
  local halfG = 0.5 * I:GetGravityForAltitude(line.start.y)
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
function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function LinkedList.LinkedList()local k={}k.value=nil;k.next=k;k.prev=k;return k end;function LinkedList.pushFront(l,d)local k={}k.value=d;LinkedList.connect(k,l.next)LinkedList.connect(l,k)end;function LinkedList.pushBack(l,d)local k={}k.value=d;LinkedList.connect(l.prev,k)LinkedList.connect(k,l)end;function LinkedList.popFront(l)local m=l.next.value;LinkedList.connect(l,l.next.next)return m end;function LinkedList.popBack(l)local m=l.prev.value;LinkedList.connect(l.prev.prev,l)return m end;function LinkedList.peekFront(l)return l.next.val end;function LinkedList.peekBack(l)return l.prev.val end;function LinkedList.connect(n,o)n.next=o;o.prev=n end;function LinkedList.toArray(l)local p=1;local q={}local k=l.next;while k~=l do q[p]=k.value;k=k.next end;return q end;function MathUtil.angleOnPlane(r,s,t)local u=Vector3.ProjectOnPlane(r,t)local g=Vector3.ProjectOnPlane(s,t)return Vector3.SignedAngle(u,g,t)end;function MathUtil.min(v,w)local x=nil;w=w or function(y,z)return y<z end;for A in v do if not x or w(A,x)then x=A end end;return x end;function MathUtil.max(v,w)local B=nil;w=w or function(y,z)return y<z end;for A in v do if not B or w(B,A)then B=A end end;return B end;function MathUtil.range(y,z,C)local D,E=y,z;local F;if not y then return end;if not z then D=0;E=y;F=D<E and 1 or-1 elseif C then F=C end;return function(G,H)local I=H+F;if I==E then return nil end;return I end,nil,D-F end;function MathUtil.shuffle(l)local J={}for p=1,#l do J[p]=l[p]end;for p=#l,2,-1 do local K=math.random(p)J[p],J[K]=J[K],J[p]end;return J end;function MathUtil.combine(y,z,L)if#y==#z then local M={}for N,O in pairs(y)do M[N]=L(N,O,z[N])end;return M end end;function MathUtil.distribution()return{n=0}end;function MathUtil.updateDistribution(P,Q)P.n=P.n+1;if P.n==1 then P.mean=Q;P.covariance={}local h=#Q;for p=1,h do local R={}for K=1,h do R[K]=0 end;P.covariance[p]=R end else P.mean=P.mean+1/(P.n+1)*Q end end;function MathUtil.mean(P)return P.mean end;function MathUtil.covariance(P)return P.cov end;function MathUtil.normal()local S,T=MathUtil.boxMuller()return S end;function MathUtil.normalPDF(S)return math.exp(-0.5*S*S)/math.sqrt(2*math.pi)end;function MathUtil.normalCDF(S)local U=0.2316419;local V=0.319381530;local W=-0.356563782;local X=1.781477937;local Y=-1.821255978;local Z=1.330274429;local g=1/(1+U*S)return 1-MathUtil.normalPDF(S)*(V*g+W*g^2+X*g^3+Y*g^4+Z*g^5)end;function MathUtil.inverseNorm(_)local a0=_>=0.5 and _ or-_;local S=5.55556*(1-((1-a0)/a0)^0.1186)if _<0.5 then S=-S end;return S end;function MathUtil.boxMuller()local a1=math.random()local a2=math.random()a2=math.random()a2=math.random()local a3=math.sqrt(-2*math.log(a1))local a4=2*math.pi*a2;return a3*math.cos(a4),a3*math.sin(a4)end;function MathUtil.angleSSS(y,z,C)if y+z<C or y+C<z or z+C<y then return nil end;local a5=math.deg(math.acos((z*z+C*C-y*y)/(2*z*C)))local a6,a7=MathUtil.angleSAS(z,a5,C)return a5,a6,a7 end;function MathUtil.sideSAS(y,a7,z)local a8=y*y+z*z-2*y*z*math.cos(math.rad(a7))return math.sqrt(a8)end;function MathUtil.angleSAS(y,a7,z)local C=MathUtil.sideSAS(y,a7,z)if MathUtil.isZero(C)then return nil end;local a5,a6;if y<z then a5=MathUtil.angleLoSin(C,y,a7)a6=180-a5-a7 else a6=MathUtil.angleLoSin(C,z,a7)a5=180-a6-a7 end;return a5,a6 end;function MathUtil.sideSSA(y,z,a5)local a9=z*z-y*y;local aa=-2*z*math.cos(math.rad(a5))local ab,ac=MathUtil.solveQuadratic(1,aa,a9)if not ac then return ab,ac end;if ab<ac then return ab,ac end;return ac,ab end;function MathUtil.angleSSA(y,z,a5)local ab,ac=MathUtil.sideSSA(y,z,a5)if not ab then return nil end;local ad,ae=MathUtil.angleSAS(z,a5,ab)if not ac then return ad,ae end;local af,ag=MathUtil.angleSAS(z,a5,ac)return ad,ae,af,ag end;function MathUtil.sideAAS(a5,a6,y)local a7=180-a5-a6;local z=MathUtil.sideLoSin(a5,a6,y)local C=MathUtil.sideLoSin(a5,a7,y)return z,C end;function MathUtil.sideLoSin(y,a5,a6)return y*math.sin(math.rad(a6))/math.sin(math.rad(a5))end;function MathUtil.angleLoSin(y,z,a5)return math.deg(math.asin(z*math.sin(math.rad(a5))/y))end;function MathUtil.clampCone(ah,ai,aj)local ak=math.min(aj,Vector3.Angle(ah,ai))local al=Vector3.Cross(ah,ai)return Quaternion.AngleAxis(ak,al)*ah end;local am=1e-9;function MathUtil.isZero(h)return h>-am and h<am end;function MathUtil.cuberoot(an)return an>0 and an^(1/3)or-math.abs(an)^(1/3)end;function MathUtil.solveQuadratic(ao,ab,ac)local ap,aq;local _,ar,as;_=ab/(2*ao)ar=ac/ao;as=_*_-ar;if MathUtil.isZero(as)then ap=-_;return ap elseif as<0 then return else local at=math.sqrt(as)ap=at-_;aq=-at-_;return ap,aq end end;function MathUtil.solveCubic(ao,ab,ac,au)local ap,aq,av;local aw,ax;local a5,a6,a7;local ay,_,ar;local az,as;a5=ab/ao;a6=ac/ao;a7=au/ao;ay=a5*a5;_=1/3*(-(1/3)*ay+a6)ar=0.5*(2/27*a5*ay-1/3*a5*a6+a7)az=_*_*_;as=ar*ar+az;if MathUtil.isZero(as)then if MathUtil.isZero(ar)then ap=0;aw=1 else local aA=MathUtil.cuberoot(-ar)ap=2*aA;aq=-aA;aw=2 end elseif as<0 then local aB=1/3*math.acos(-ar/math.sqrt(-az))local g=2*math.sqrt(-_)ap=g*math.cos(aB)aq=-g*math.cos(aB+math.pi/3)av=-g*math.cos(aB-math.pi/3)aw=3 else local at=math.sqrt(as)local aA=MathUtil.cuberoot(at-ar)local O=-MathUtil.cuberoot(at+ar)ap=aA+O;aw=1 end;ax=1/3*a5;if aw>0 then ap=ap-ax end;if aw>1 then aq=aq-ax end;if aw>2 then av=av-ax end;return ap,aq,av end;function MathUtil.solveQuartic(ao,ab,ac,au,aC)local ap,aq,av,aD;local aE={}local S,aA,O,ax;local a5,a6,a7,as;local ay,_,ar,a3;local aw;a5=ab/ao;a6=ac/ao;a7=au/ao;as=aC/ao;ay=a5*a5;_=-0.375*ay+a6;ar=0.125*ay*a5-0.5*a5*a6+a7;a3=-(3/256)*ay*ay+0.0625*ay*a6-0.25*a5*a7+as;if MathUtil.isZero(a3)then aE[3]=ar;aE[2]=_;aE[1]=0;aE[0]=1;local aF={MathUtil.solveCubic(aE[0],aE[1],aE[2],aE[3])}aw=#aF;ap,aq,av=aF[1],aF[2],aF[3]else aE[3]=0.5*a3*_-0.125*ar*ar;aE[2]=-a3;aE[1]=-0.5*_;aE[0]=1;ap,aq,av=MathUtil.solveCubic(aE[0],aE[1],aE[2],aE[3])S=ap;aA=S*S-a3;O=2*S-_;if MathUtil.isZero(aA)then aA=0 elseif aA>0 then aA=math.sqrt(aA)else return end;if MathUtil.isZero(O)then O=0 elseif O>0 then O=math.sqrt(O)else return end;aE[2]=S-aA;aE[1]=ar<0 and-O or O;aE[0]=1;do local aF={MathUtil.solveQuadratic(aE[0],aE[1],aE[2])}aw=#aF;ap,aq=aF[1],aF[2]end;aE[2]=S+aA;aE[1]=ar<0 and O or-O;aE[0]=1;if aw==0 then local aF={MathUtil.solveQuadratic(aE[0],aE[1],aE[2])}aw=aw+#aF;ap,aq=aF[1],aF[2]end;if aw==1 then local aF={MathUtil.solveQuadratic(aE[0],aE[1],aE[2])}aw=aw+#aF;aq,av=aF[1],aF[2]end;if aw==2 then local aF={MathUtil.solveQuadratic(aE[0],aE[1],aE[2])}aw=aw+#aF;av,aD=aF[1],aF[2]end end;ax=0.25*a5;if aw>0 then ap=ap-ax end;if aw>1 then aq=aq-ax end;if aw>2 then av=av-ax end;if aw>3 then aD=aD-ax end;return ap,aq,av,aD end;function RingBuffer.RingBuffer(aG)local aH={}aH.buf={}aH.capacity=aG;aH.size=0;aH.head=1;local aI=getmetatable(aH)or{}aI.__index=RingBuffer.get;setmetatable(aH,aI)return aH end;function RingBuffer.isFull(aH)return aH.size>=aH.capacity end;function RingBuffer.setSize(aH,aJ)aH.size=aJ end;function RingBuffer.push(aH,d)aH.buf[(aH.head+aH.size-1)%aH.capacity+1]=d;if aH.size==aH.capacity then aH.head=aH.head%aH.capacity+1 else aH.size=aH.size+1 end end;function RingBuffer.pop(aH)if aH.size==0 then return nil end;local m=aH.buf[aH.head]aH.buf[aH.head]=nil;aH.head=aH.head%aH.capacity+1;aH.size=aH.size-1;return m end;function RingBuffer.get(aH,aK)if type(aK)~="number"or math.floor(aK)~=aK then return nil end;if aK<1 or aK>aH.size then return nil end;return aH.buf[(aH.head+aK-2)%aH.capacity+1]end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(y,z)local aL=type(y)=="int"local aM=type(z)=="int"if not aL and aM then return z+y end;if aL and not aM then return MathUtil.combine(y,z,function(N,an,aN)return y+aN end)else return MathUtil.combine(y,z,function(N,an,aN)return an+aN end)end end;VectorN.mt.__sub=function(y,z)return y+-z end;VectorN.mt.__mul=function(y,z)local aL=type(y)=="int"local aM=type(z)=="int"if not aL and aM then return z*y end;if aL and not aM then local M={}for N,O in pairs(z)do M[N]=y*O end;return M else return MathUtil.combine(y,z,function(N,an,aN)return an*aN end)end end;VectorN.mt.__div=function(y,z)local aL=type(y)=="int"local aM=type(z)=="int"if not aL and aM then return y*1/z end;if aL and not aM then local M={}for N,O in pairs(z)do M[N]=y/O end;return M else return MathUtil.combine(y,z,function(N,an,aN)return an/aN end)end end;VectorN.mt.__unm=function(y)local M={}for N,O in pairs(y)do M[N]=-O end;return M end;function VectorN.VectorN(l)local aO={}for N,O in pairs(l)do if type(O)=="table"then aO[N]=VectorN.VectorN(O)else aO[N]=O end end;setmetatable(aO,VectorN.mt)return aO end;function Control.PID(aP,aQ,aR,aS,aT,aU)local aV={}aV.kP=aP;aV.kI=aQ;aV.kD=aR;aV.Iacc=Accumulator.Accumulator(aS,aT)if aU and aU~=0 then aV.period=aU end;return aV end;function Control.processPID(aW,aX,e)aX=aW.period and(aX+aW.period/2)%aW.period-aW.period/2 or aX;local _=aW.kP*aX;local p,aY=aW.kI*Accumulator.update(aW.Iacc,aX,e)p=p/aY;local h=aW.kD*(aX-(aW.lastError or aX))/e;aW.lastError=aX;return _+p+h end;function Control.FF(aE,aU)local aZ={}aZ.coeffs=aE;aZ.degree=#aE-1;if aU and aU~=0 then aZ.period=aU end;aZ.derivs={}return aZ end;function Control.processFF(aW,a_,e)local b0=0*a_;local b1=a_;local b2=a_;for p=1,aW.degree+1 do b2=aW.derivs[p]aW.derivs[p]=b1;b0=b0+aW.coeffs[p]*b1;if b2 then local b3=b1-b2;if p==1 and aW.period then b3=(b3+aW.period/2)%aW.period-aW.period/2 end;b1=b3/e else break end end;return b0 end;function Nav.toLocal(b4,b5,b6)local b7=b4-b5;return Quaternion.Inverse(b6)*b7 end;function Nav.toGlobal(b8,b5,b6)local b7=b6*b8;return b7+b5 end;function Nav.cartToPol(b9)local a3=b9.magnitude;local a4=Vector3.SignedAngle(Vector3.forward,b9,Vector3.up)local aB=90-Vector3.Angle(Vector3.up,b9)return Vector3(a3,a4,aB)end;function Nav.cartToCyl(b9)local ba=Vector3(b9.x,0,b9.z)local bb=ba.magnitude;local aB=Vector3.SignedAngle(Vector3.forward,b9,Vector3.up)local S=b9.y;return Vector3(bb,aB,S)end;function Nav.polToCart(b9)local a3,a4,aB=b9.x,b9.y,b9.z;local an=Mathf.Sin(a4)*Mathf.Cos(aB)local aN=Mathf.Sin(aB)local S=Mathf.Cos(a4)*Mathf.Cos(aB)return a3*Vector3(an,aN,S)end;function Nav.cylToCart(b9)local bb,aB,bc=b9.x,b9.y,b9.z;local an=bb*Mathf.Sin(aB)local aN=bc;local S=bb*Mathf.Cos(aB)return Vector3(an,aN,S)end;function Targeting.firstOrderTargeting(bd,be,bf)local bg=bd-Vector3.Project(bd,be)local bh=Vector3.Dot(be,bd-bg)/be.sqrMagnitude;local y,z=MathUtil.solveQuadratic(bh-bf*bf,2*bh,bg.sqrMagnitude+bh*bh)local bi=nil;if y and y>=0 then bi=y end;if z and z>=0 and z<y then bi=z end;return bi and(bd+bi*be).normalized or nil end;function Targeting.secondOrderTargeting(bd,bj,bk,bf,bl,bm)local g=Targeting.secondOrderTargetingTime(bd,bj,bk,bf,bl/bf,bm/bf)if g and g>0 then return(bd/g+bj+0.5*bk*g).normalized end;return nil end;function Targeting.secondOrderTargetingTime(bd,bj,bk,bf,bn,bo)local y=0.25*bk.sqrMagnitude;local z=Vector3.Dot(bj,bk)local C=bj.sqrMagnitude-bf*bf+Vector3.Dot(bd,bk)local h=2*Vector3.Dot(bd,bj)local aX=bd.sqrMagnitude;local bp={MathUtil.solveQuartic(y,z,C,h,aX)}local g=nil;for p=1,4 do if bp[p]and bp[p]>bn and bp[p]<bo then if not g or g and bp[p]<g then g=bp[p]end end end;return g end;function Targeting.AIPPN(bq,bd,br,be,bs)local bj=be-br;local bt=Vector3.Dot(-bj,bd.normalized)if bt<=0 then bt=10 end;local bu=bd.magnitude/bt;local bv=Vector3.Cross(bd,bj)/bd.sqrMagnitude;local bw=Vector3.Cross(bd,bs)/bd.sqrMagnitude*bu/2;local bx=bv+bw;local by=Vector3.Cross(bx,bd.normalized)local bz=Vector3.ProjectOnPlane(by,br).normalized;local bA=bq*br.magnitude*bx.magnitude;return bA*bz end;function Targeting.ATPN(bq,bd,br,be,bs)local bj=be-br;local bt=-Vector3.Dot(bj,bd.normalized)if bt<=0 then bt=10 end;local bv=Vector3.Cross(bd,bj)/bd.sqrMagnitude;local by=Vector3.Cross(bv,bd.normalized)local bB=Vector3.ProjectOnPlane(bs,bd)return bq*bt*by+0.5*bq*bs end;function BlockUtil.getWeaponsByName(bC,bD,bE,bF)if DEBUG then bC:Log("searching for "..bD)end;local bG=bC:GetAllSubConstructs()local bH={}bE=bE or-1;local C=bE;if not bF or bF==0 or bF==2 then for p=0,bC:GetWeaponCount()-1 do if C==0 then break end;if bC:GetWeaponBlockInfo(p).CustomName==bD then table.insert(bH,{subIdx=nil,wpnIdx=p})if DEBUG then bC:Log("found weapon "..bD.." on hull, type "..bC:GetWeaponInfo(p).WeaponType)end;C=C-1 end end end;if not bF or bF==1 or bF==2 then for aK=1,#bG do local ax=bG[aK]for p=0,bC:GetWeaponCountOnSubConstruct(ax)-1 do if C==0 then break end;if bC:GetWeaponBlockInfoOnSubConstruct(ax,p).CustomName==bD then table.insert(bH,{subIdx=ax,wpnIdx=p})if DEBUG then bC:Log("found weapon "..bD.." on subobj "..ax..", type "..bC:GetWeaponInfo(p).WeaponType)end;C=C-1 end end end end;if DEBUG then bC:Log("weapon count: "..#bH)end;return bH end;function BlockUtil.getSubConstructsByName(bC,bD,bE)if DEBUG then bC:Log("searching for "..bD)end;local bG=bC:GetAllSubConstructs()local bI={}bE=bE or-1;local C=bE;for aK=1,#bG do local ax=bG[aK]if C==0 then break end;if bC:GetSubConstructInfo(ax).CustomName==bD then table.insert(bI,ax)if DEBUG then bC:Log("found subobj "..bD)end;C=C-1 end end;if DEBUG then bC:Log("subobj count: "..#bI)end;return bI end;function BlockUtil.getBlocksByName(bC,bD,type,bE)if DEBUG then bC:Log("searching for "..bD)end;local bJ={}bE=bE or-1;local C=bE;for aK=0,bC:Component_GetCount(type)-1 do if C==0 then break end;if bC:Component_GetBlockInfo(type,aK).CustomName==bD then table.insert(bJ,aK)if DEBUG then bC:Log("found component "..bD)end;C=C-1 end end;if DEBUG then bC:Log("component count: "..#bJ)end;return bJ end;function BlockUtil.getWeaponInfo(bC,bK)local bL;if bK.subIdx then bL=bC:GetWeaponInfoOnSubConstruct(bK.subIdx,bK.wpnIdx)else bL=bC:GetWeaponInfo(bK.wpnIdx)end;return bL end;function BlockUtil.aimWeapon(bC,bK,bM,bN)if bK.subIdx then bC:AimWeaponInDirectionOnSubConstruct(bK.subIdx,bK.wpnIdx,bM.x,bM.y,bM.z,bN)else bC:AimWeaponInDirection(bK.wpnIdx,bM.x,bM.y,bM.z,bN)end end;function BlockUtil.fireWeapon(bC,bK,bN)if bK.subIdx then bC:FireWeaponOnSubConstruct(bK.subIdx,bK.wpnIdx,bN)else bC:FireWeapon(bK.wpnIdx,bN)end end;function Combat.pickTarget(bC,bO,bP)bP=bP or function(G,bQ)return bQ.Priority end;local a_,bR;for p in MathUtil.range(bC:GetNumberOfTargets(bO))do local bQ=bC:GetTargetInfo(bO,p)local bS=bP(bC,bQ)if not a_ or bS>bR then a_=bQ;bR=bS end end;return a_ end;function Combat.CheckConstraints(bC,bT,bU,bV)local bW;if bV then bW=bC:GetWeaponConstraintsOnSubConstruct(bV,bU)else bW=bC:GetWeaponConstraints(bU)end;local bX=bC:GetConstructForwardVector()local bY=bC:GetConstructUpVector()local bZ=Quaternion.LookRotation(bX,bY)bT=Quaternion.Inverse(bZ)*bT;if bW.InParentConstructSpace and bV then local b_=bC:GetSubConstructInfo(bV).localRotation;bT=Quaternion.inverse(b_)*bT end;local c0=MathUtil.angleOnPlane(Vector3.forward,bT,Vector3.up)local c1=bT;c1.z=0;local A=Mathf.Atan2(bT.z,c1.magnitude)local c2=c0>bW.MinAzimuth and c0<bW.MaxAzimuth;local c3=A>bW.MinElevation and A<bW.MaxElevation;if bW.FlipAzimuth then c2=not c2 end;if c2 and c3 then return true end;c0=c0+180;A=180-A;if A>180 then A=A-360 end;if A<-180 then A=A+360 end;c2=c0>bW.MinAzimuth and c0<bW.MaxAzimuth;c3=A>bW.MinElevation and A<bW.MaxElevation;if bW.FlipAzimuth then c2=not c2 end;if c2 and c3 then return true end;return false end
