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
-- parameters to find weapons (important: name firing pieces AND turret blocks)
local weaponDef = {
  { name = "laser", velocity = math.huge }
}
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
                             originTimes = RingBuffer.RingBuffer(numOrigins) }
      RingBuffer.setSize(enemies[target.Id].pos, times.size)
      RingBuffer.setSize(enemies[target.Id].vel, times.size)
    end
    RingBuffer.push(enemies[target.Id].pos, target.Position)
    RingBuffer.push(enemies[target.Id].vel, target.Velocity)
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
  if currentLine and CheckAndUpdateLine(I, currentLine, projectile, frameTime, eps) then
    local target = I:GetTargetInfo(0, 0)
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
        RingBuffer.push(enemy.origins, closest)
        RingBuffer.push(enemy.originTimes, I:GetTimeSinceSpawn())
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
    if not lastOriginSwitchTime or I:GetTimeSinceSpawn() - lastOriginSwitchTime > originSwitchTime then
      while enemy.origins.size > 0 and I:GetTimeSinceSpawn() - enemy.originTimes[1] > maxStaleness do
        RingBuffer.pop(enemy.origins)
        RingBuffer.pop(enemy.originTimes)
      end
      if enemy.origins.size == 0 then
        fp = nil
      end
      fp = enemy.origins[math.random(1, enemy.origins.size)]
      lastOriginSwitchTime = I:GetTimeSinceSpawn()
      lastOrigin = fp
    end
    if fp then
      local aim
      for i, turret in ipairs(turrets) do
        for j, weapon in ipairs(turret) do
          local wInfo = BlockUtil.getWeaponInfo(I, weapon)
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
          if aim then
            BlockUtil.aimWeapon(I, weapon, aim, 0)
            if Vector3.Angle(wInfo.CurrentDirection, aim) < AIM_TOL then
              I:FireWeapon(0, 0)
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
function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function LinkedList.LinkedList()local k={}k.value=nil;k.next=k;k.prev=k;return k end;function LinkedList.pushFront(l,d)local k={}k.value=d;LinkedList.connect(k,l.next)LinkedList.connect(l,k)end;function LinkedList.pushBack(l,d)local k={}k.value=d;LinkedList.connect(l.prev,k)LinkedList.connect(k,l)end;function LinkedList.popFront(l)local m=l.next.value;LinkedList.connect(l,l.next.next)return m end;function LinkedList.popBack(l)local m=l.prev.value;LinkedList.connect(l.prev.prev,l)return m end;function LinkedList.peekFront(l)return l.next.val end;function LinkedList.peekBack(l)return l.prev.val end;function LinkedList.connect(n,o)n.next=o;o.prev=n end;function LinkedList.toArray(l)local p=1;local q={}local k=l.next;while k~=l do q[p]=k.value;k=k.next end;return q end;function MathUtil.angleOnPlane(r,s,t)local u=Vector3.ProjectOnPlane(r,t)local g=Vector3.ProjectOnPlane(s,t)return Vector3.SignedAngle(u,g,t)end;function MathUtil.min(v,w)local x=nil;w=w or function(y,z)return y<z end;for ele in v do if not x or w(ele,x)then x=ele end end;return x end;function MathUtil.max(v,w)local A=nil;w=w or function(y,z)return y<z end;for ele in v do if not A or w(A,ele)then A=ele end end;return A end;function MathUtil.range(y,z,B)local C,D=y,z;local E;if not y then return end;if not z then C=0;D=y;E=C<D and 1 or-1 elseif B then E=B end;return function(F,G)local H=G+E;if H==D then return nil end;return H end,nil,C-E end;function MathUtil.shuffle(l)local I={}for p=1,#l do I[p]=l[p]end;for p=#l,2,-1 do local J=math.random(p)I[p],I[J]=I[J],I[p]end;return I end;function MathUtil.combine(y,z,K)if#y==#z then local L={}for M,N in pairs(y)do L[M]=K(M,N,z[M])end;return L end end;function MathUtil.distribution()return{n=0}end;function MathUtil.updateDistribution(O,P)O.n=O.n+1;if O.n==1 then O.mean=P;O.covariance={}local h=#P;for p=1,h do local Q={}for J=1,h do Q[J]=0 end;O.covariance[p]=Q end else O.mean=O.mean+1/(O.n+1)*P end end;function MathUtil.mean(O)return O.mean end;function MathUtil.covariance(O)return O.cov end;function MathUtil.normal()local R,S=MathUtil.boxMuller()return R end;function MathUtil.normalPDF(R)return math.exp(-0.5*R*R)/math.sqrt(2*math.pi)end;function MathUtil.normalCDF(R)local T=0.2316419;local U=0.319381530;local V=-0.356563782;local W=1.781477937;local X=-1.821255978;local Y=1.330274429;local g=1/(1+T*R)return 1-MathUtil.normalPDF(R)*(U*g+V*g^2+W*g^3+X*g^4+Y*g^5)end;function MathUtil.inverseNorm(Z)local _=Z>=0.5 and Z or-Z;local R=5.55556*(1-((1-_)/_)^0.1186)if Z<0.5 then R=-R end;return R end;function MathUtil.boxMuller()local a0=math.random()local a1=math.random()a1=math.random()a1=math.random()local a2=math.sqrt(-2*math.log(a0))local a3=2*math.pi*a1;return a2*math.cos(a3),a2*math.sin(a3)end;function MathUtil.angleSSS(y,z,B)if y+z<B or y+B<z or z+B<y then return nil end;local a4=math.deg(math.acos((z*z+B*B-y*y)/(2*z*B)))local a5,a6=MathUtil.angleSAS(z,a4,B)return a4,a5,a6 end;function MathUtil.sideSAS(y,a6,z)local a7=y*y+z*z-2*y*z*math.cos(math.rad(a6))return math.sqrt(a7)end;function MathUtil.angleSAS(y,a6,z)local B=MathUtil.sideSAS(y,a6,z)if MathUtil.isZero(B)then return nil end;local a4,a5;if y<z then a4=MathUtil.angleLoSin(B,y,a6)a5=180-a4-a6 else a5=MathUtil.angleLoSin(B,z,a6)a4=180-a5-a6 end;return a4,a5 end;function MathUtil.sideSSA(y,z,a4)local a8=z*z-y*y;local a9=-2*z*math.cos(math.rad(a4))local aa,ab=MathUtil.solveQuadratic(1,a9,a8)if not ab then return aa,ab end;if aa<ab then return aa,ab end;return ab,aa end;function MathUtil.angleSSA(y,z,a4)local aa,ab=MathUtil.sideSSA(y,z,a4)if not aa then return nil end;local ac,ad=MathUtil.angleSAS(z,a4,aa)if not ab then return ac,ad end;local ae,af=MathUtil.angleSAS(z,a4,ab)return ac,ad,ae,af end;function MathUtil.sideAAS(a4,a5,y)local a6=180-a4-a5;local z=MathUtil.sideLoSin(a4,a5,y)local B=MathUtil.sideLoSin(a4,a6,y)return z,B end;function MathUtil.sideLoSin(y,a4,a5)return y*math.sin(math.rad(a5))/math.sin(math.rad(a4))end;function MathUtil.angleLoSin(y,z,a4)return math.deg(math.asin(z*math.sin(math.rad(a4))/y))end;function MathUtil.clampCone(ag,ah,ai)local aj=math.min(ai,Vector3.Angle(ag,ah))local ak=Vector3.Cross(ag,ah)return Quaternion.AngleAxis(aj,ak)*ag end;local al=1e-9;function MathUtil.isZero(h)return h>-al and h<al end;function MathUtil.cuberoot(am)return am>0 and am^(1/3)or-math.abs(am)^(1/3)end;function MathUtil.solveQuadratic(an,aa,ab)local ao,ap;local Z,aq,ar;Z=aa/(2*an)aq=ab/an;ar=Z*Z-aq;if MathUtil.isZero(ar)then ao=-Z;return ao elseif ar<0 then return else local as=math.sqrt(ar)ao=as-Z;ap=-as-Z;return ao,ap end end;function MathUtil.solveCubic(an,aa,ab,at)local ao,ap,au;local av,aw;local a4,a5,a6;local ax,Z,aq;local ay,ar;a4=aa/an;a5=ab/an;a6=at/an;ax=a4*a4;Z=1/3*(-(1/3)*ax+a5)aq=0.5*(2/27*a4*ax-1/3*a4*a5+a6)ay=Z*Z*Z;ar=aq*aq+ay;if MathUtil.isZero(ar)then if MathUtil.isZero(aq)then ao=0;av=1 else local az=MathUtil.cuberoot(-aq)ao=2*az;ap=-az;av=2 end elseif ar<0 then local aA=1/3*math.acos(-aq/math.sqrt(-ay))local g=2*math.sqrt(-Z)ao=g*math.cos(aA)ap=-g*math.cos(aA+math.pi/3)au=-g*math.cos(aA-math.pi/3)av=3 else local as=math.sqrt(ar)local az=MathUtil.cuberoot(as-aq)local N=-MathUtil.cuberoot(as+aq)ao=az+N;av=1 end;aw=1/3*a4;if av>0 then ao=ao-aw end;if av>1 then ap=ap-aw end;if av>2 then au=au-aw end;return ao,ap,au end;function MathUtil.solveQuartic(an,aa,ab,at,aB)local ao,ap,au,aC;local aD={}local R,az,N,aw;local a4,a5,a6,ar;local ax,Z,aq,a2;local av;a4=aa/an;a5=ab/an;a6=at/an;ar=aB/an;ax=a4*a4;Z=-0.375*ax+a5;aq=0.125*ax*a4-0.5*a4*a5+a6;a2=-(3/256)*ax*ax+0.0625*ax*a5-0.25*a4*a6+ar;if MathUtil.isZero(a2)then aD[3]=aq;aD[2]=Z;aD[1]=0;aD[0]=1;local aE={MathUtil.solveCubic(aD[0],aD[1],aD[2],aD[3])}av=#aE;ao,ap,au=aE[1],aE[2],aE[3]else aD[3]=0.5*a2*Z-0.125*aq*aq;aD[2]=-a2;aD[1]=-0.5*Z;aD[0]=1;ao,ap,au=MathUtil.solveCubic(aD[0],aD[1],aD[2],aD[3])R=ao;az=R*R-a2;N=2*R-Z;if MathUtil.isZero(az)then az=0 elseif az>0 then az=math.sqrt(az)else return end;if MathUtil.isZero(N)then N=0 elseif N>0 then N=math.sqrt(N)else return end;aD[2]=R-az;aD[1]=aq<0 and-N or N;aD[0]=1;do local aE={MathUtil.solveQuadratic(aD[0],aD[1],aD[2])}av=#aE;ao,ap=aE[1],aE[2]end;aD[2]=R+az;aD[1]=aq<0 and N or-N;aD[0]=1;if av==0 then local aE={MathUtil.solveQuadratic(aD[0],aD[1],aD[2])}av=av+#aE;ao,ap=aE[1],aE[2]end;if av==1 then local aE={MathUtil.solveQuadratic(aD[0],aD[1],aD[2])}av=av+#aE;ap,au=aE[1],aE[2]end;if av==2 then local aE={MathUtil.solveQuadratic(aD[0],aD[1],aD[2])}av=av+#aE;au,aC=aE[1],aE[2]end end;aw=0.25*a4;if av>0 then ao=ao-aw end;if av>1 then ap=ap-aw end;if av>2 then au=au-aw end;if av>3 then aC=aC-aw end;return ao,ap,au,aC end;function RingBuffer.RingBuffer(aF)local aG={}aG.buf={}aG.capacity=aF;aG.size=0;aG.head=1;local aH=getmetatable(aG)or{}aH.__index=RingBuffer.get;setmetatable(aG,aH)return aG end;function RingBuffer.isFull(aG)return aG.size>=aG.capacity end;function RingBuffer.setSize(aG,aI)aG.size=aI end;function RingBuffer.push(aG,d)aG.buf[(aG.head+aG.size-1)%aG.capacity+1]=d;if aG.size==aG.capacity then aG.head=aG.head%aG.capacity+1 else aG.size=aG.size+1 end end;function RingBuffer.pop(aG)if aG.size==0 then return nil end;local m=aG.buf[aG.head]aG.buf[aG.head]=nil;aG.head=aG.head%aG.capacity+1;aG.size=aG.size-1;return m end;function RingBuffer.get(aG,aJ)if type(aJ)~="number"or math.floor(aJ)~=aJ then return nil end;if aJ<1 or aJ>aG.size then return nil end;return aG.buf[(aG.head+aJ-2)%aG.capacity+1]end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(y,z)local aK=type(y)=="int"local aL=type(z)=="int"if not aK and aL then return z+y end;if aK and not aL then return MathUtil.combine(y,z,function(M,am,aM)return y+aM end)else return MathUtil.combine(y,z,function(M,am,aM)return am+aM end)end end;VectorN.mt.__sub=function(y,z)return y+-z end;VectorN.mt.__mul=function(y,z)local aK=type(y)=="int"local aL=type(z)=="int"if not aK and aL then return z*y end;if aK and not aL then local L={}for M,N in pairs(z)do L[M]=y*N end;return L else return MathUtil.combine(y,z,function(M,am,aM)return am*aM end)end end;VectorN.mt.__div=function(y,z)local aK=type(y)=="int"local aL=type(z)=="int"if not aK and aL then return y*1/z end;if aK and not aL then local L={}for M,N in pairs(z)do L[M]=y/N end;return L else return MathUtil.combine(y,z,function(M,am,aM)return am/aM end)end end;VectorN.mt.__unm=function(y)local L={}for M,N in pairs(y)do L[M]=-N end;return L end;function VectorN.VectorN(l)local aN={}for M,N in pairs(l)do if type(N)=="table"then aN[M]=VectorN.VectorN(N)else aN[M]=N end end;setmetatable(aN,VectorN.mt)return aN end;function Control.PID(aO,aP,aQ,aR,aS,aT)local aU={}aU.kP=aO;aU.kI=aP;aU.kD=aQ;aU.Iacc=Accumulator.Accumulator(aR,aS)if aT and aT~=0 then aU.period=aT end;return aU end;function Control.processPID(aV,aW,e)aW=aV.period and(aW+aV.period/2)%aV.period-aV.period/2 or aW;local Z=aV.kP*aW;local p,aX=aV.kI*Accumulator.update(aV.Iacc,aW,e)p=p/aX;local h=aV.kD*(aW-(aV.lastError or aW))/e;aV.lastError=aW;return Z+p+h end;function Control.FF(aD,aT)local aY={}aY.coeffs=aD;aY.degree=#aD-1;if aT and aT~=0 then aY.period=aT end;aY.derivs={}return aY end;function Control.processFF(aV,aZ,e)local a_=0*aZ;local b0=aZ;local b1=aZ;for p=1,aV.degree+1 do b1=aV.derivs[p]aV.derivs[p]=b0;a_=a_+aV.coeffs[p]*b0;if b1 then local b2=b0-b1;if p==1 and aV.period then b2=(b2+aV.period/2)%aV.period-aV.period/2 end;b0=b2/e else break end end;return a_ end;function Nav.toLocal(b3,b4,b5)local b6=b3-b4;return Quaternion.Inverse(b5)*b6 end;function Nav.toGlobal(b7,b4,b5)local b6=b5*b7;return b6+b4 end;function Nav.cartToPol(b8)local a2=b8.magnitude;local a3=Vector3.SignedAngle(Vector3.forward,b8,Vector3.up)local aA=90-Vector3.Angle(Vector3.up,b8)return Vector3(a2,a3,aA)end;function Nav.cartToCyl(b8)local b9=Vector3(b8.x,0,b8.z)local ba=b9.magnitude;local aA=Vector3.SignedAngle(Vector3.forward,b8,Vector3.up)local R=b8.y;return Vector3(ba,aA,R)end;function Nav.polToCart(b8)local a2,a3,aA=b8.x,b8.y,b8.z;local am=Mathf.Sin(a3)*Mathf.Cos(aA)local aM=Mathf.Sin(aA)local R=Mathf.Cos(a3)*Mathf.Cos(aA)return a2*Vector3(am,aM,R)end;function Nav.cylToCart(b8)local ba,aA,bb=b8.x,b8.y,b8.z;local am=ba*Mathf.Sin(aA)local aM=bb;local R=ba*Mathf.Cos(aA)return Vector3(am,aM,R)end;function Targeting.firstOrderTargeting(bc,bd,be)local bf=bc-Vector3.Project(bc,bd)local bg=Vector3.Dot(bd,bc-bf)/bd.sqrMagnitude;local y,z=MathUtil.solveQuadratic(bg-be*be,2*bg,bf.sqrMagnitude+bg*bg)local bh=nil;if y and y>=0 then bh=y end;if z and z>=0 and z<y then bh=z end;return bh and(bc+bh*bd).normalized or nil end;function Targeting.secondOrderTargeting(bc,bi,bj,be,bk,bl)local g=Targeting.secondOrderTargetingTime(bc,bi,bj,be,bk/be,bl/be)if g and g>0 then return(bc/g+bi+0.5*bj*g).normalized end;return nil end;function Targeting.secondOrderTargetingTime(bc,bi,bj,be,bm,bn)local y=0.25*bj.sqrMagnitude;local z=Vector3.Dot(bi,bj)local B=bi.sqrMagnitude-be*be+Vector3.Dot(bc,bj)local h=2*Vector3.Dot(bc,bi)local aW=bc.sqrMagnitude;local bo={MathUtil.solveQuartic(y,z,B,h,aW)}local g=nil;for p=1,4 do if bo[p]and bo[p]>bm and bo[p]<bn then if not g or g and bo[p]<g then g=bo[p]end end end;return g end;function Targeting.AIPPN(bp,bc,bq,bd,br)local bi=bd-bq;local bs=Vector3.Dot(-bi,bc.normalized)if bs<=0 then bs=10 end;local bt=bc.magnitude/bs;local bu=Vector3.Cross(bc,bi)/bc.sqrMagnitude;local bv=Vector3.Cross(bc,br)/bc.sqrMagnitude*bt/2;local bw=bu+bv;local bx=Vector3.Cross(bw,bc.normalized)local by=Vector3.ProjectOnPlane(bx,bq).normalized;local bz=bp*bq.magnitude*bw.magnitude;return bz*by end;function Targeting.ATPN(bp,bc,bq,bd,br)local bi=bd-bq;local bs=-Vector3.Dot(bi,bc.normalized)if bs<=0 then bs=10 end;local bu=Vector3.Cross(bc,bi)/bc.sqrMagnitude;local bx=Vector3.Cross(bu,bc.normalized)local bA=Vector3.ProjectOnPlane(br,bc)return bp*bs*bx+0.5*bp*br end;function BlockUtil.getWeaponsByName(bB,bC,bD,bE)if DEBUG then bB:Log("searching for "..bC)end;local bF=bB:GetAllSubConstructs()local bG={}bD=bD or-1;local B=bD;if not bE or bE==0 or bE==2 then for p=0,bB:GetWeaponCount()-1 do if B==0 then break end;if bB:GetWeaponBlockInfo(p).CustomName==bC then table.insert(bG,{subIdx=nil,wpnIdx=p})if DEBUG then bB:Log("found weapon "..bC.." on hull, type "..bB:GetWeaponInfo(p).WeaponType)end;B=B-1 end end end;if not bE or bE==1 or bE==2 then for aJ=1,#bF do local aw=bF[aJ]for p=0,bB:GetWeaponCountOnSubConstruct(aw)-1 do if B==0 then break end;if bB:GetWeaponBlockInfoOnSubConstruct(aw,p).CustomName==bC then table.insert(bG,{subIdx=aw,wpnIdx=p})if DEBUG then bB:Log("found weapon "..bC.." on subobj "..aw..", type "..bB:GetWeaponInfo(p).WeaponType)end;B=B-1 end end end end;if DEBUG then bB:Log("weapon count: "..#bG)end;return bG end;function BlockUtil.getSubConstructsByName(bB,bC,bD)if DEBUG then bB:Log("searching for "..bC)end;local bF=bB:GetAllSubConstructs()local bH={}bD=bD or-1;local B=bD;for aJ=1,#bF do local aw=bF[aJ]if B==0 then break end;if bB:GetSubConstructInfo(aw).CustomName==bC then table.insert(bH,aw)if DEBUG then bB:Log("found subobj "..bC)end;B=B-1 end end;if DEBUG then bB:Log("subobj count: "..#bH)end;return bH end;function BlockUtil.getBlocksByName(bB,bC,type,bD)if DEBUG then bB:Log("searching for "..bC)end;local bI={}bD=bD or-1;local B=bD;for aJ=0,bB:Component_GetCount(type)-1 do if B==0 then break end;if bB:Component_GetBlockInfo(type,aJ).CustomName==bC then table.insert(bI,aJ)if DEBUG then bB:Log("found component "..bC)end;B=B-1 end end;if DEBUG then bB:Log("component count: "..#bI)end;return bI end;function BlockUtil.getWeaponInfo(bB,bJ)local bK;if bJ.subIdx then bK=bB:GetWeaponInfoOnSubConstruct(bJ.subIdx,bJ.wpnIdx)else bK=bB:GetWeaponInfo(bJ.wpnIdx)end;return bK end;function BlockUtil.aimWeapon(bB,bJ,bL,bM)if bJ.subIdx then bB:AimWeaponInDirectionOnSubConstruct(bJ.subIdx,bJ.wpnIdx,bL.x,bL.y,bL.z,bM)else bB:AimWeaponInDirection(bJ.wpnIdx,bL.x,bL.y,bL.z,bM)end end;function BlockUtil.fireWeapon(bB,bJ,bM)if bJ.subIdx then bB:FireWeaponOnSubConstruct(bJ.subIdx,bJ.wpnIdx,bM)else bB:FireWeapon(bJ.wpnIdx,bM)end end;function Combat.pickTarget(bB,bN,bO)bO=bO or function(F,bP)return bP.Priority end;local aZ,bQ;for p in MathUtil.range(bB:GetNumberOfTargets(bN))do local bP=bB:GetTargetInfo(bN,p)local bR=bO(bB,bP)if not aZ or bR>bQ then aZ=bP;bQ=bR end end;return aZ end;function Combat.CheckConstraints(bB,bS,bT,bU)local bV;if bU then bV=bB:GetWeaponConstraintsOnSubConstruct(bU,bT)else bV=bB:GetWeaponConstraints(bT)end;local bW=bB:GetConstructForwardVector()local bX=bB:GetConstructUpVector()local bY=Quaternion.LookRotation(bW,bX)bS=Quaternion.Inverse(bY)*bS;if bV.InParentConstructSpace and bU then local bZ=bB:GetSubConstructInfo(bU).localRotation;bS=Quaternion.inverse(bZ)*bS end;local b_=MathUtil.angleOnPlane(Vector3.forward,bS,Vector3.up)local c0=bS;c0.z=0;local c1=Mathf.Atan2(bS.z,c0.magnitude)local c2=b_>bV.MinAzimuth and b_<bV.MaxAzimuth;local c3=c1>bV.MinElevation and c1<bV.MaxElevation;if bV.FlipAzimuth then c2=not c2 end;if c2 and c3 then return true end;b_=b_+180;ele=180-ele;if ele>180 then ele=ele-360 end;if ele<-180 then ele=ele+360 end;c2=b_>bV.MinAzimuth and b_<bV.MaxAzimuth;c3=c1>bV.MinElevation and c1<bV.MaxElevation;if bV.FlipAzimuth then c2=not c2 end;if c2 and c3 then return true end;return false end
