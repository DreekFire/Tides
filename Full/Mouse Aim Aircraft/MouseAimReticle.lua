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

-- Settings
-- Flight Settings
-- full roll will point top of aircraft towards desired direction
  -- actual roll will be error angle / fullRollAngle
  -- times the full roll, capped at 1
local fullRollRate = 40
local fullRollAngle = 40

local autoAimAngle = 5.5
local autoFireRange = 2000
local lockRange = 3000
local maxFireAngle = 5
local mainWeaponName = "cannons"

-- trim in case of unbalanced aircraft (yaw/pitch/roll)
  -- in breadboards and lua, positive pitch is pitch down
local trim = {yaw = 0, pitch = 0, roll = 0}

-- PIDFF constants
--[[
  Tides uses the parallel form of PID, i.e. K_p * e(t) + K_i * integral of e(t) + K_d * d/dt e(t),
  whereas stock PIDs use the standard form, i.e. K_p * [e(t) + 1/T_i * integral of e(t) + T_d * d/dt e(t)].
  To convert from standard form coefficients to parallel form coefficients,
  K_p remains the same,
  K_d = T_d * K_p,
  K_i = K_p / T_i

  to deal with global/local coordinate
  issues, the feedforward controllers
  will receive the desired angular
  velocity instead of the direction
--]]
local yawS   = {  P = 0.025,
                  I = 0.05,
                  D = 0.02,
                  IDecay = 0.125, -- set IDecay to 1 to match stock PID behavior
                  V = 0.001, -- advanced: feedforward constants. set to 0 for stock PID behavior
                  A = 0,
                  FFwindow = 0.2,
                  FFdecay = 0.2  }
local pitchS = {  P = 0.03,
                  I = 0.1,
                  D = 0.025,
                  IDecay = 0.125,
                  V = 0.004,
                  A = 0,
                  FFwindow = 0.2,
                  FFdecay = 0.2  }
local rollS  = {  P = 0.03,
                  I = 0,
                  D = 0.01,
                  IDecay = 0.125,
                  V = 0,
                  A = 0,
                  FFwindow = 0.2,
                  FFdecay = 0.2  }

-- Cockpit Settings
--[[
  the amount by which the chair shifts rambot's head relative to the center
  of the cockpit.
  the camera position in the chair is not centered. I suspect it was placed
  by hand. These values are simply my best guess.
 
  Normally (-0.01, 1, 0.09) from the chair center, (-0.01, 1.68, 0.09) for elevated
]]--
local chairOffset = Vector3(-0.009, 0, 0)

-- shifts crosshairs to compensate for non-centered images
local hologramOffset = Vector3(0, 0, 0)

-- distance from camera to HUD
local hudDistance = 0.65

-- the name of the spinblocks that stabilize the chair
  -- set to nil if using articulated chair
local chairSpinblockNames = {  roll = "rollSeat",
                               pitch = "pitchSeat",
                               yaw = "yawSeat"  }

-- the position of the chair relative to vehicle origin
  -- only needed if using articulated chair
local manualHeadPos = Vector3(0, 1, 19)

-- the name of the weapon used to steer the craft
local steeringWepName = "joystick"

-- "leading" mode will tell you where to shoot
  -- "trailing" mode will tell you where you'll hit if you shoot now
local sightMode = "trailing"

local sightProjNames = {  aim = "aimProjector",
                          lead = "leadProjector",
                          lock = "lockProjector"  }

-- shell velocity of your cannon
local muzzleVel = 1129

-- averaging time and decay rate for acceleration tracking
  -- set accelAvgTime to 0 to ignore acceleration
local accelAvgTime = 0.2
local accelDecay = 0.1

local steerAvgTime = 0.125
local steerDecay = 0.0625

-- misc variables
local timeDiff
local velDiff
local pointDiff, pointAcc
local rollDiff
local rollAcc, pitchAcc, yawAcc
local accelAcc
local pitchPID, pitchFF
local yawPID, yawFF
local rollPID, rollFF
local proj = {}
local lastDir
local gunOffset
local headPos
local prevTarget

local steeringWep
local mainWeps
local chairSpinblocks = {}
local inited = false
local DEBUG = true

function Init(I)
  if chairSpinblockNames then
    for block, name in pairs(chairSpinblockNames) do
      chairSpinblocks[block] = BlockUtil.getSubConstructsByName(I, name, 1)[1]
    end
  end
  SetHeadPos(I)
  steeringWep = BlockUtil.getWeaponsByName(I, steeringWepName, 1)[1]
  mainWeps = BlockUtil.getWeaponsByName(I, mainWeaponName, -1)
  gunOffset = Vector3.zero
  for i, wep in ipairs(mainWeps) do
    local info = BlockUtil.getWeaponInfo(I, wep)
    gunOffset = gunOffset + info.LocalFirePoint
  end
  gunOffset = gunOffset / #mainWeps - headPos

  for p, name in pairs(sightProjNames) do
    proj[p] = BlockUtil.getBlocksByName(I, sightProjNames[p], 33, 1)[1]
  end

  timeDiff = Differ.Differ(I:GetGameTime())
  velDiff = Differ.Differ(Vector3.zero)
  if accelAvgTime > 0 then
    accelAcc = Accumulator.Accumulator(accelAvgTime, accelDecay)
  end

  pointDiff = Differ.Differ(I:GetConstructForwardVector())
  if steerAvgTime > 0 then
    pointAcc = Accumulator.Accumulator(steerAvgTime, steerDecay)
  end
  rollDiff = Differ.Differ(0)
  yawAcc = Accumulator.Accumulator(yawS.FFwindow, yawS.FFdecay)
  pitchAcc = Accumulator.Accumulator(pitchS.FFwindow, pitchS.FFdecay)
  rollAcc = Accumulator.Accumulator(rollS.FFwindow, rollS.FFdecay)

  pitchPID = Control.PID(pitchS.P, pitchS.I, pitchS.D, pitchS.IDecay, 0, 360)
  yawPID = Control.PID(yawS.P, yawS.I, yawS.D, yawS.IDecay, 0, 360)
  rollPID = Control.PID(rollS.P, rollS.I, rollS.D, rollS.IDecay, 0, 360)
  pitchFF = Control.FF({pitchS.V, pitchS.A})
  yawFF = Control.FF({yawS.V, yawS.A})
  rollFF = Control.FF({rollS.V, rollS.A})
end

function Update(I)
  if not inited then
    Init(I)
    inited = true
    return
  end

  local elapsedTime = Differ.update(timeDiff, I:GetGameTime())

  SetHeadPos(I)
  RotateSeat(I)

  local target = Combat.pickTarget(I, 0, Priority)
  if not target or (prevTarget and prevTarget.Id ~= target.Id) then
    SwapTarget(target)
  end
  prevTarget = target

  if target and accelAvgTime > 0 then
    local dv = Differ.update(velDiff, target.Velocity) / elapsedTime
    Accumulator.update(accelAcc, dv, elapsedTime)
  end
  Sight(I, target)
  Steer(I)
end

function SetHeadPos(I)
  if manualHeadPos then
    headPos = manualHeadPos
  elseif chairSpinblockNames then
    local rotation = Quaternion.LookRotation(I:GetConstructForwardVector(), I:GetConstructUpVector())
    local invRot = Quaternion.Inverse(rotation)
    headPos = invRot * (I:GetSubConstructInfo(chairSpinblocks.roll).Position - I:GetConstructPosition())
  else
    headPos = Vector3.Zero
  end
end

function Priority(I, t)
  local relPos = t.Position - I:GetConstructPosition()
  local rangePenalty = 0
  if relPos.magnitude > lockRange then rangePenalty = 1000000 end
  return -Vector3.Angle(relPos, BlockUtil.getWeaponInfo(I, steeringWep).CurrentDirection) - rangePenalty
end

function RotateSeat(I)
  if chairSpinblockNames then
    local av = I:GetLocalAngularVelocity() * Differ.get(timeDiff)
    I:SetSpinBlockRotationAngle(chairSpinblocks.yaw, -I:GetConstructYaw() - Mathf.Rad2Deg * av.y)
    I:SetSpinBlockRotationAngle(chairSpinblocks.pitch, I:GetConstructPitch() + Mathf.Rad2Deg * av.x)
    I:SetSpinBlockRotationAngle(chairSpinblocks.roll, -I:GetConstructRoll() - Mathf.Rad2Deg * av.z)
  end
end

function PositionReticle(I, aim, lead, lock)
  local seatPosLoc = headPos + 0.5 * Vector3.forward
  local vecs = {aim = aim, lead = lead, lock = lock}
  local r = (Mathf.Repeat(I:GetConstructRoll() + 90, 180) - 90)
  for p, id in pairs(proj) do
    vecs[p] = vecs[p] + Quaternion.AngleAxis(-r, Vector3.forward) * hologramOffset
    if vecs[p].z >= 0 then
      vecs[p].z = 0
      local info = I:Component_GetBlockInfo(33, id)
      local dest = (vecs[p] + hudDistance * Vector3.forward).normalized * hudDistance + seatPosLoc
      local diff = dest - info.LocalPosition
      I:Component_SetBoolLogic(33, id, true)
      I:Component_SetFloatLogic_1(33, id, 2, diff.z)
      I:Component_SetFloatLogic_1(33, id, 3, diff.x)
      I:Component_SetFloatLogic_1(33, id, 4, diff.y)
      I:Component_SetFloatLogic_1(33, id, 5, Mathf.Atan2(vecs[p].x, hudDistance) * Mathf.Rad2Deg)
      I:Component_SetFloatLogic_1(33, id, 6, Mathf.Atan2(-vecs[p].y, hudDistance) * Mathf.Rad2Deg)
      I:Component_SetFloatLogic_1(33, id, 7, -r)
    else
      I:Component_SetBoolLogic(33, id, false)
    end
  end
end

function Sight(I, target)
  local fw = I:GetConstructForwardVector()
  local rotation = Quaternion.LookRotation(fw, I:GetConstructUpVector())
  local invRot = Quaternion.Inverse(rotation)
  local offset = invRot * chairOffset
  local sw = BlockUtil.getWeaponInfo(I, steeringWep)
  local pointing = sw.CurrentDirection
  local pLoc = invRot * pointing
  local leadOffset = Vector3(0, 0, -1)
  local lockOffset = Vector3(0, 0, -1)
  local aimDir = MathUtil.clampCone(Vector3.forward, pLoc, maxFireAngle)
  local aimOffset = offset + hudDistance * aimDir / aimDir.z
  if target then
    local relPos = target.Position - (rotation * (headPos + gunOffset) + I:GetConstructPosition())
    local relVel = target.Velocity - I:GetVelocityVector()
    local grav = I:GetGravityForAltitude(I:GetConstructPosition().y)
    local ac, w
    if accelAvgTime > 0 then ac, w = Accumulator.get(accelAcc) else ac, w = Vector3.zero, 1 end
    local accel = -grav --ac / w - grav
    local intercept, t = Targeting.secondOrderTargeting(relPos, relVel, accel, muzzleVel, -math.huge, math.huge)
    lockOffset = (invRot * relPos) / Mathf.Abs(Vector3.Dot(relPos, fw)) * (hudDistance - offset.z)
    if t then
      local globalIntercept = intercept + (rotation * (headPos + gunOffset) + I:GetConstructPosition())
      if Vector3.Angle(fw, intercept) < autoAimAngle
       and intercept.magnitude < autoFireRange then
        local optPointing = (intercept - rotation * gunOffset).normalized
        for i, wep in ipairs(mainWeps) do
          local info = BlockUtil.getWeaponInfo(I, wep)
          if not info.PlayerCurrentlyControllingIt then
            local diff = globalIntercept - info.GlobalFirePoint
            diff = MathUtil.clampCone(I:GetConstructForwardVector(), diff, maxFireAngle)
            pointing = optPointing
            BlockUtil.aimWeapon(I, wep, diff, 0)
            BlockUtil.fireWeapon(I, wep, 0)
          end
        end
      end
      if sightMode == "trailing" then
        pointing = MathUtil.clampCone(I:GetConstructForwardVector(), pointing, maxFireAngle)
        local shellPosition = t * (muzzleVel * pointing - relVel) + 0.5 * t * t * (-accel)
        local shellPosLoc = invRot * shellPosition + gunOffset

        leadOffset = offset + (hudDistance - offset.z) / Mathf.Abs(shellPosLoc.z) * shellPosLoc
      elseif sightMode == "leading" then
        local enemyPosLoc = invRot * intercept

        leadOffset = offset + (hudDistance - offset.z) / Mathf.Abs(enemyPosLoc.z) * enemyPosLoc
      end
      aimOffset = offset + (hudDistance - offset.z) / relPos.magnitude * gunOffset + hudDistance * aimDir / aimDir.z
    end
  end
  aimOffset.z = 0
  PositionReticle(I, aimOffset, leadOffset, lockOffset)
end

function Steer(I)
  I:TellAiThatWeAreTakingControl()

  local pointing
  local cWep = BlockUtil.getWeaponInfo(I, steeringWep)
  local elapsedTime = Differ.get(timeDiff)
  if cWep.PlayerCurrentlyControllingIt then
    lastDir = nil
    if steerAvgTime > 0 then
      local wt
      pointing, wt = Accumulator.update(pointAcc, cWep.CurrentDirection, elapsedTime)
      pointing = pointing / wt
    else
      pointing = cWep.CurrentDirection
    end
  else
    if not lastDir then
      lastDir = I:GetConstructForwardVector()
      lastDir.y = 0
      lastDir = lastDir.normalized
    end
    pointing = lastDir
  end


  local fw = I:GetConstructForwardVector()
  local yawE = MathUtil.angleOnPlane(fw, pointing, I:GetConstructUpVector())
  local pitchE = MathUtil.angleOnPlane(fw, pointing, I:GetConstructRightVector())

  local pointDel = Differ.update(pointDiff, pointing) / elapsedTime
  local pointVel = Vector3.Cross(pointing, pointDel) * Mathf.Rad2Deg

  local r, wt
  r, wt = Accumulator.update(yawAcc,
    Vector3.Dot(pointVel, I:GetConstructUpVector()),
    elapsedTime)
  local yawRate = r / wt
  r, wt = Accumulator.update(pitchAcc,
    Vector3.Dot(pointVel, I:GetConstructRightVector()),
    elapsedTime)
  local pitchRate = r / wt

  local rollScale = Mathf.Min(1, Vector3.Angle(pointing, fw) / fullRollAngle)
  local trs = rollScale * rollScale
  local pra = rollScale * rollScale * MathUtil.angleOnPlane(Vector3.up, pointing, I:GetConstructForwardVector())
  rollScale = Mathf.Min(1, pointVel.magnitude / fullRollRate)
  local rra = rollScale * rollScale * MathUtil.angleOnPlane(Vector3.up, pointDel, I:GetConstructForwardVector())
  trs = trs + rollScale * rollScale
  local desiredRoll = (pra + rra) / Mathf.Max(1, trs) 
  if desiredRoll ~= desiredRoll then desiredRoll = 0 end
  local rollE = desiredRoll - I:GetConstructRoll()
  if rollE < -180 then
    rollE = rollE + 360
  elseif rollE > 180 then
    rollE = rollE - 360
  end

  r, wt = Accumulator.update(rollAcc,
    Differ.update(rollDiff, desiredRoll) / elapsedTime,
    elapsedTime)
  local rollRate = r / wt

  --steering
  local yaw = Control.processPID(yawPID, yawE, elapsedTime)
              + Control.processFF(yawFF, yawRate, elapsedTime)
  local pitch = Control.processPID(pitchPID, pitchE, elapsedTime)
              + Control.processFF(pitchFF, pitchRate, elapsedTime)
  local roll = Control.processPID(rollPID, rollE, elapsedTime)
              + Control.processFF(rollFF, rollRate, elapsedTime)

  --positive: yaw right, pitch down, roll left
  I:RequestControl(2, 2, roll + trim.roll)
  I:RequestControl(2, 5, pitch + trim.pitch)
  I:RequestControl(2, 1, yaw + trim.yaw)
end

function SwapTarget(newTarget)
  velDiff = Differ.Differ(newTarget and newTarget.Velocity or Vector3.zero)
  accelAcc = Accumulator.Accumulator(accelAvgTime, accelDecay)
end

function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function Heapq.Heapq(i,k)local l={}l.data=i;l.comp=k or function(m,n)return m<n end;local o=#l.data;l.size=o;for p=math.floor(o/2),1,-1 do Heapq.siftDown(l,p)end;return l end;function Heapq.siftDown(l,q)local r=false;local s=q;local o=#l.data;while not r do r=true;local t=2*s;local u=2*s+1;local v=s;if t<=o and l.comp(l.data[t],l.data[v])then v=t;r=false end;if u<=o and l.comp(l.data[u],l.data[v])then v=u;r=false end;if not r then local w=l.data[v]l.data[v]=l.data[s]l.data[s]=w;s=v end end end;function Heapq.siftUp(l,q)local r=false;local s=q;while not r do r=true;local x=math.floor(s/2)if l.comp(l.data[s],l.data[x])then local w=l.data[x]l.data[x]=l.data[s]l.data[s]=w;s=x;r=false end end end;function Heapq.insert(l,y)l.data[l.size+1]=y;l.size=l.size+1;Heapq.siftUp(l,l.size)end;function Heapq.pop(l)local z=l.data[1]l.data[1]=l.data[l.size]l.data[l.size]=nil;l.size=l.size-1;Heapq.siftDown(l,1)return z end;function Heapq.peek(l)return l.data[1]end;function Heapq.size(l)return l.size end;function LinkedList.LinkedList()local A={}A.value=nil;A.next=A;A.prev=A;return A end;function LinkedList.pushFront(B,d)local A={}A.value=d;LinkedList.connect(A,B.next)LinkedList.connect(B,A)end;function LinkedList.pushBack(B,d)local A={}A.value=d;LinkedList.connect(B.prev,A)LinkedList.connect(A,B)end;function LinkedList.popFront(B)local C=B.next.value;LinkedList.connect(B,B.next.next)return C end;function LinkedList.popBack(B)local C=B.prev.value;LinkedList.connect(B.prev.prev,B)return C end;function LinkedList.peekFront(B)return B.next.val end;function LinkedList.peekBack(B)return B.prev.val end;function LinkedList.connect(D,E)D.next=E;E.prev=D end;function LinkedList.toArray(B)local F=1;local G={}local A=B.next;while A~=B do G[F]=A.value;A=A.next end;return G end;function MathUtil.angleOnPlane(H,I,J)local K=Vector3.ProjectOnPlane(H,J)local g=Vector3.ProjectOnPlane(I,J)return Vector3.SignedAngle(K,g,J)end;function MathUtil.min(L,M)local N=nil;M=M or function(m,n)return m<n end;for O in L do if not N or M(O,N)then N=O end end;return N end;function MathUtil.max(L,M)local P=nil;M=M or function(m,n)return m<n end;for O in L do if not P or M(P,O)then P=O end end;return P end;function MathUtil.range(m,n,Q)local R,S=m,n;local T;if not m then return end;if not n then R=0;S=m;T=R<S and 1 or-1 elseif Q then T=Q end;return function(U,V)local W=V+T;if W==S then return nil end;return W end,nil,R-T end;function MathUtil.shuffle(B,X)local s=X and B or{}if not X then for F=1,#B do s[F]=B[F]end end;for F=#B,2,-1 do local Y=math.random(F)s[F],s[Y]=s[Y],s[F]end;return s end;function MathUtil.combine(m,n,Z)if#m==#n then local z={}for _,a0 in pairs(m)do z[_]=Z(_,a0,n[_])end;return z end end;function MathUtil.angleSSS(m,n,Q)if m+n<Q or m+Q<n or n+Q<m then return nil end;local a1=math.deg(math.acos((n*n+Q*Q-m*m)/(2*n*Q)))local a2,a3=MathUtil.angleSAS(n,a1,Q)return a1,a2,a3 end;function MathUtil.sideSAS(m,a3,n)local a4=m*m+n*n-2*m*n*math.cos(math.rad(a3))return math.sqrt(a4)end;function MathUtil.angleSAS(m,a3,n)local Q=MathUtil.sideSAS(m,a3,n)if MathUtil.isZero(Q)then return nil end;local a1,a2;if m<n then a1=MathUtil.angleLoSin(Q,m,a3)a2=180-a1-a3 else a2=MathUtil.angleLoSin(Q,n,a3)a1=180-a2-a3 end;return a1,a2 end;function MathUtil.sideSSA(m,n,a1)local a5=n*n-m*m;local a6=-2*n*math.cos(math.rad(a1))local a7,a8=MathUtil.solveQuadratic(1,a6,a5)if not a8 then return a7,a8 end;if a7<a8 then return a7,a8 end;return a8,a7 end;function MathUtil.angleSSA(m,n,a1)local a7,a8=MathUtil.sideSSA(m,n,a1)if not a7 then return nil end;local a9,aa=MathUtil.angleSAS(n,a1,a7)if not a8 then return a9,aa end;local ab,ac=MathUtil.angleSAS(n,a1,a8)return a9,aa,ab,ac end;function MathUtil.sideAAS(a1,a2,m)local a3=180-a1-a2;local n=MathUtil.sideLoSin(a1,a2,m)local Q=MathUtil.sideLoSin(a1,a3,m)return n,Q end;function MathUtil.sideLoSin(m,a1,a2)return m*math.sin(math.rad(a2))/math.sin(math.rad(a1))end;function MathUtil.angleLoSin(m,n,a1)return math.deg(math.asin(n*math.sin(math.rad(a1))/m))end;function MathUtil.clampCone(ad,ae,af)local ag=math.min(af,Vector3.Angle(ad,ae))local ah=Vector3.Cross(ad,ae)return Quaternion.AngleAxis(ag,ah)*ad end;function MathUtil.newton(ai,aj,ak,al,am,an)al=al or 1e-5;an=an or 10*al;am=am or 25;aj=aj or function(ao)return(ai(ao+an)-ai(ao))/an end;ak=ak or 0;local ap=al+1;local aq=0;while ap>al and aq<am do local ar=ai(ak)local as=aj(ak)if not ar or not as then return nil end;ap=-ar/as;ak=ak+ap;aq=aq+1 end;if aq<am then return ak,false end;return ak,true end;function MathUtil.ITP(ai,m,n,al,am)if ai(m)*ai(n)>0 then return nil end;if ai(m)>ai(n)then ai=function(ao)return-ai(ao)end end;al=al or 1e-5;am=am or 25;local at=0.2/(n-m)local au=2;local av=1;local aw=math.ceil(math.log((n-m)/(2*al),2))local ax=aw+av;local aq=am;for Y=1,am do local ay=ai(m)local az=ai(n)local aA=ay-az;if aA==0 then return m end;local aB=0.5*(m+n)local aC=(n*ay+m*az)/aA;if aB<m or aB>n then aB=0.5*(m+n)end;local aD=aB-aC;local aE=at*math.abs(n-m)^au;local aF=aD>0 and 1 or(aD==0 and 0 or-1)local aG=aE<=math.abs(aD)and aC+aF*aE or aB;local aH=al*2^(ax-Y)-0.5*(n-m)local aI=math.abs(aG-aB)<=aH and aG or aB-aF*aH;local aJ=ai(aI)if aJ>0 then n=aI elseif aJ<0 then m=aI else return aI,Y==am end;if n-m<2*al then aq=Y;break end end;local ay=ai(m)local az=ai(n)local aA=az-ay;if aA~=0 then return(m*az-n*ay)/aA,aq==am end;return m,aq==am end;function MathUtil.binomCoeffs(aK,aL)if aL then local aM={}else local aM={}aM[1]=1;for _=1,aK do aM[_+1]=aM[_]*(aK-_)/(_+1)end;return aM end end;function MathUtil.ruleOfSigns(aM,aN)local aO={}local aP=#aM;for F=1,aP do aO[F]=aM[aP-F+1]end;if aN~=0 then local aQ={}for F=1,aP do aQ[F]=(F-1)*aM[aP-F+1]end;local aR=1;for F=2,aP do local aS=aN^(F-1)for Y=1,aP-F+1 do local aT=F+Y-1;aO[Y]=aO[Y]+aR*aQ[aT]*aS;aQ[aT]=aQ[aT]*(Y-1)end;aR=aR/F end end;local aU={}local o=1;for F,aV in ipairs(aO)do if aV~=0 then aU[o]=aV;o=o+1 end end;local aW=0;for F=1,#aU-1 do if aU[F]*aU[F+1]<0 then aW=aW+1 end end;return aW end;function MathUtil.cache(ai)local Q={}local aX=getmetatable(Q)or{}function aX.__index(aY,ao)local C=ai(ao)aY[ao]=C;return C end;setmetatable(Q,aX)return function(m)return Q[m]end end;function MathUtil.lerp(ai,R,S,T,aZ)local a_={}for F=1,math.floor((S-R)/T)+1 do a_[F]=ai(R+F*T)end;a_.start=R;a_.stop=S;a_.step=T;a_.lval=aZ and a_[1]or nil;a_.rval=aZ and a_[#a_]or nil;return function(ao)if ao>=a_.stop then return a_.rval end;if ao<=a_.start then return a_.lval end;local F=(ao-a_.start)/a_.step;local b0=F%1;F=math.floor(F)return(1-b0)*a_[F]+b0*a_[F+1]end end;function MathUtil._factorial(o)if o<2 then return 1 end;return MathUtil._factorial(o-1)end;MathUtil.factorial=MathUtil.cache(MathUtil._factorial)MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(al)MathUtil.eps=al end;function MathUtil.cuberoot(ao)return ao>0 and ao^(1/3)or-math.abs(ao)^(1/3)end;function MathUtil.solveQuadratic(b1,a7,a8)local b2,b3;local b4,b5,b6;b4=a7/(2*b1)b5=a8/b1;b6=b4*b4-b5;if MathUtil.isZero(b6)then b2=-b4;return b2 elseif b6<0 then return else local b7=math.sqrt(b6)b2=b7-b4;b3=-b7-b4;return b2,b3 end end;function MathUtil.solveCubic(b1,a7,a8,b8)local b2,b3,b9;local ba,bb;local a1,a2,a3;local bc,b4,b5;local bd,b6;a1=a7/b1;a2=a8/b1;a3=b8/b1;bc=a1*a1;b4=1/3*(-(1/3)*bc+a2)b5=0.5*(2/27*a1*bc-1/3*a1*a2+a3)bd=b4*b4*b4;b6=b5*b5+bd;if MathUtil.isZero(b6)then if MathUtil.isZero(b5)then b2=0;ba=1 else local be=MathUtil.cuberoot(-b5)b2=2*be;b3=-be;ba=2 end elseif b6<0 then local bf=1/3*math.acos(-b5/math.sqrt(-bd))local g=2*math.sqrt(-b4)b2=g*math.cos(bf)b3=-g*math.cos(bf+math.pi/3)b9=-g*math.cos(bf-math.pi/3)ba=3 else local b7=math.sqrt(b6)local be=MathUtil.cuberoot(b7-b5)local a0=-MathUtil.cuberoot(b7+b5)b2=be+a0;ba=1 end;bb=1/3*a1;if ba>0 then b2=b2-bb end;if ba>1 then b3=b3-bb end;if ba>2 then b9=b9-bb end;return b2,b3,b9 end;function MathUtil.solveQuartic(b1,a7,a8,b8,bg)local b2,b3,b9,bh;local aM={}local bi,be,a0,bb;local a1,a2,a3,b6;local bc,b4,b5,bj;local ba=0;a1=a7/b1;a2=a8/b1;a3=b8/b1;b6=bg/b1;bc=a1*a1;b4=-0.375*bc+a2;b5=0.125*bc*a1-0.5*a1*a2+a3;bj=-(3/256)*bc*bc+0.0625*bc*a2-0.25*a1*a3+b6;if MathUtil.isZero(bj)then aM[3]=b5;aM[2]=b4;aM[1]=0;aM[0]=1;local bk={MathUtil.solveCubic(aM[0],aM[1],aM[2],aM[3])}ba=#bk;b2,b3,b9=bk[1],bk[2],bk[3]elseif MathUtil.isZero(b5)then local bl={MathUtil.solveQuadratic(1,b4,bj)}if bl[1]>=0 then b2=-math.sqrt(bl[1])b3=math.sqrt(bl[1])ba=2 end;if bl[2]>=0 then if ba==0 then b2=-math.sqrt(bl[2])b3=math.sqrt(bl[2])ba=2 else b9=-math.sqrt(bl[2])bh=math.sqrt(bl[2])ba=4 end end else aM[3]=0.5*bj*b4-0.125*b5*b5;aM[2]=-bj;aM[1]=-0.5*b4;aM[0]=1;b2,b3,b9=MathUtil.solveCubic(aM[0],aM[1],aM[2],aM[3])bi=b2;be=bi*bi-bj;a0=2*bi-b4;if MathUtil.isZero(be)then be=0 elseif be>0 then be=math.sqrt(be)else return end;if MathUtil.isZero(a0)then a0=0 elseif a0>0 then a0=math.sqrt(a0)else return end;aM[2]=bi-be;aM[1]=b5<0 and-a0 or a0;aM[0]=1;do local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=#bk;b2,b3=bk[1],bk[2]end;aM[2]=bi+be;aM[1]=b5<0 and a0 or-a0;aM[0]=1;if ba==0 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b2,b3=bk[1],bk[2]end;if ba==1 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b3,b9=bk[1],bk[2]end;if ba==2 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b9,bh=bk[1],bk[2]end end;bb=0.25*a1;if ba>0 then b2=b2-bb end;if ba>1 then b3=b3-bb end;if ba>2 then b9=b9-bb end;if ba>3 then bh=bh-bb end;return b2,b3,b9,bh end;function Matrix3.Matrix3(a_)local bm={}for F=1,9 do bm[F]=a_[F]end;setmetatable(bm,Matrix3.meta)return bm end;function Matrix3.get(bm,bn,bo)return bm[(bn-1)*3+bo]end;function Matrix3.set(bm,bn,bo,C)bm[(bn-1)*3+bo]=C end;function Matrix3.scalarmul(bm,s)local bp={}for F=1,9 do bp[F]=s*bm[F]end;return bp end;function Matrix3.vecmul(bm,bq)local bp=Vector3.zero;for bn=0,2 do local C=0;for bo=1,3 do C=C+bq[Matrix3.vecIdx[bo]]*bm[bn*3+bo]end;bp[Matrix3.vecIdx[bn+1]]=C end end;function Matrix3.matmul(br,bs)local bp={}for F=0,2 do for Y=0,2 do local C=0;for _=0,2 do C=C+br[F*3+_+1]*bs[_*3+Y+1]end;bp[F*3+Y+1]=C end end end;function Matrix3.mul(m,n)if getmetatable(m)~=Matrix3.meta then if m.x then return Matrix3.vecmul(Matrix3.transpose(n),m)end;return Matrix3.mul(n,m)end;if getmetatable(n)==Matrix3.meta then return Matrix3.matmul(m,n)end;if n.x then return Matrix3.vecmul(m,n)end;return Matrix3.scalarmul(m,n)end;function Matrix3.Identity()return{1,0,0,0,1,0,0,0,1}end;function Matrix3.Zero()return{0,0,0,0,0,0,0,0,0}end;function Matrix3.pow(bm,bt)local bu=bm;local bv=bm;while true do bt=math.floor(bt/2)if bt%2==1 then bu=Matrix3.matmul(bm,bv)end;if bt>=2 then bv=Matrix3.matmul(bv,bv)else break end end;return bu end;function Matrix3.add(br,bs)local bp={}for F=1,9 do bp[F]=br[F]+bs[F]end;return bp end;function Matrix3.hadamard(br,bs)local bp={}for F=1,9 do bp[F]=br[F]*bs[F]end;return bp end;function Matrix3.transpose(bm)local bw={}for bn=0,2 do for bo=0,2 do bw[bo*3+bn+1]=bm[bn*3+bo+1]end end;return bw end;function Matrix3.determinant(bm)local bx=0;local by=0;for h=0,2 do for s=0,2 do bx=bx+bm[s*3+(s+h)%3+1]by=by+bm[s*3+(-s+h)%3+1]end end;return bx-by end;function Matrix3.adjugate(bm)local bz={}for bn=0,2 do for bo=0,2 do local bA=0;for F=0,1 do local bB=1;for Y=1,2 do bB=bB*bm[(bn+Y)%3*3+(bo+F+Y)%3+1]end;bA=bA+bB end;bz[bo*3+bn+1]=bA end end;return bz end;function Matrix3.inverse(bm)local bC=Matrix3.determinant(bm)if Stats.isZero(bC)then return end;local bz=Matrix3.cofactors(bm)return bz/bC end;Matrix3.vecIdx={'x','y','z'}Matrix3.meta={__add=Matrix3.add,__mul=Matrix3.mul,__unm=function(bD)return Matrix3.scalarmul(bD,-1)end,__pow=Matrix3.pow}function RingBuffer.RingBuffer(bE)local bF={}bF.buf={}bF.capacity=bE;bF.size=0;bF.head=1;local aX=getmetatable(bF)or{}aX.__index=RingBuffer.get;setmetatable(bF,aX)return bF end;function RingBuffer.isFull(bF)return bF.size>=bF.capacity end;function RingBuffer.push(bF,d)bF.buf[(bF.head+bF.size-1)%bF.capacity+1]=d;if bF.size==bF.capacity then bF.head=bF.head%bF.capacity+1 else bF.size=bF.size+1 end end;function RingBuffer.pop(bF)if bF.size==0 then return nil end;local C=bF.buf[bF.head]bF.buf[bF.head]=nil;bF.head=bF.head%bF.capacity+1;bF.size=bF.size-1;return C end;function RingBuffer.get(bF,p)if type(p)~="number"or math.floor(p)~=p then return nil end;if p<1 or p>bF.size then return nil end;return bF.buf[(bF.head+p-2)%bF.capacity+1]end;function Search.interpolatedSearch(bG,t,u,bH,bI,bJ)bJ=bJ or 50;local m,n,bK;local bL=0;while u>t do m=bG[t]if m==bH then return t end;if m>bH then return bI and t or nil end;n=bG[u]if n==bH then return u end;if n<bH then return bI and u or nil end;bK=math.floor((bH-m)/(n-m)*(u-t)+t)bK=math.min(math.max(bK,t+1),u-1)if bG[bK]==bH then return bK end;if bH<bG[bK]then if bI and math.abs(bG[bK-1]-bH)>math.abs(bG[bK]-bH)then return bK end;u=bK-1 else if bI and math.abs(bG[bK+1]-bH)>math.abs(bG[bK]-bH)then return bK end;t=bK+1 end;bL=bL+1;if bL>bJ then break end end;return bI and t or nil end;function Stats.Distribution(bM)local bN={n=0,vars=bM}if bM then local bO={}local bP={}local bQ=#bM;for F,a0 in ipairs(bM)do bO[a0]=0;for Y=1,bQ do bP[(F-1)*bQ+Y]=0 end end;bN.mean=bO;bN.cov=bP else bN.mean=0;bN.cov=0 end;return bN end;function Stats.updateDistribution(bN,bR,bS)local bT=bN.n;bS=bS or 1;bN.n=bN.n+bS;if bN.vars then local bU={}local bQ=bN.vars and#bN.vars or 1;for F,a0 in ipairs(bN.vars)do bU[F]=bN.mean[a0]local bV=bU[F]+bR[a0]*bS/bN.n;for Y=F,bQ do local ae=bN.vars[Y]local bW=bN.mean[ae]local bX=(bS or 1)*(bR[a0]-bV)*(bR[ae]-bW)bN.cov[(F-1)*bQ+Y]=(bN.cov[(F-1)*bQ+Y]*bT+bX)/bN.n;bN.cov[(Y-1)*bQ+F]=bN.cov[(F-1)*bQ+Y]end;bN.mean[a0]=bV end else local bV=bN.mean+bR*bS/bN.n;bN.cov=(bN.cov*bT+bS*(bR-bV)*(bR-bN.mean))/bN.n;bN.mean=bV end;return bN end;function Stats.updateDistributionBatched(bN,bY,bZ)if#bY==0 then return end;local b_;local bQ=bN.vars and#bN.vars or 1;local c0=0;for Y=1,#bY do c0=c0+(bZ and bZ[Y]or 1)end;bN.n=bN.n+c0;local bT=bN.n;if bN.vars then for F,c1 in ipairs(bN.vars)do local bo={}for Y=1,#bY do bo[Y]=bY[Y][c1]end;b_[F]=bo end;for F,a0 in ipairs(bN.vars)do local c2=0;for Y,s in ipairs(b_[F])do c2=c2+s*(bZ and bZ[Y]or 1)end;local bV=bN.mean[a0]+c2/bN.n;for Y=F,bQ do local bW=bN.mean[bN.vars[Y]]c2=0;for s=1,#bY do c2=c2+(bZ and bZ[s]or 1)*(b_[F][s]-bV)*(b_[Y][s]-bW)end;bN.cov[(F-1)*bQ+Y]=(bN.cov[(F-1)*bQ+Y]*bT+c2)/bN.n;bN.cov[(Y-1)*bQ+F]=bN.cov[(F-1)*bQ+Y]end;bN.mean[a0]=bV end else local c2=0;for F,s in ipairs(bY)do c2=c2+s*(bZ and bZ[F]or 1)end;local bV=bN.mean+c2/bN.n;c2=0;for F,s in ipairs(bY)do c2=c2+(bZ and bZ[F]or 1)*(s-bV)*(s-bN.mean)end;bN.cov=(bN.cov*bT+c2)/bN.n end;return bN end;function Stats.mean(bN)return bN.mean end;function Stats.covariance(bN)return bN.cov end;function Stats.normal()local bi,c3=Stats.boxMuller()return bi end;function Stats.normalPDF(bi)return math.exp(-0.5*bi*bi)/math.sqrt(2*math.pi)end;function Stats.normalCDF(bi)local c4=0.2316419;local c5=0.319381530;local c6=-0.356563782;local c7=1.781477937;local c8=-1.821255978;local c9=1.330274429;local g=1/(1+c4*bi)return 1-Stats.normalPDF(bi)*(c5*g+c6*g^2+c7*g^3+c8*g^4+c9*g^5)end;function Stats.inverseNorm(b4)local ca=b4>=0.5 and b4 or-b4;local bi=5.55556*(1-((1-ca)/ca)^0.1186)if b4<0.5 then bi=-bi end;return bi end;function Stats.boxMuller()local cb=math.random()local cc=math.random()cc=math.random()cc=math.random()local bj=math.sqrt(-2*math.log(cb))local cd=2*math.pi*cc;return bj*math.cos(cd),bj*math.sin(cd)end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(m,n)local ce=type(m)=="number"local cf=type(n)=="number"if not ce and cf then return n+m end;if ce and not cf then return Stats.combine(m,n,function(_,ao,cg)return m+cg end)else return Stats.combine(m,n,function(_,ao,cg)return ao+cg end)end end;VectorN.mt.__sub=function(m,n)return m+-n end;VectorN.mt.__mul=function(m,n)local ce=type(m)=="number"local cf=type(n)=="number"if not ce and cf then return n*m end;if ce and not cf then local z={}for _,a0 in pairs(n)do z[_]=m*a0 end;return z else return Stats.combine(m,n,function(_,ao,cg)return ao*cg end)end end;VectorN.mt.__div=function(m,n)local ce=type(m)=="number"local cf=type(n)=="number"if not ce and cf then return m*1/n end;if ce and not cf then local z={}for _,a0 in pairs(n)do z[_]=m/a0 end;return z else return Stats.combine(m,n,function(_,ao,cg)return ao/cg end)end end;VectorN.mt.__unm=function(m)local z={}for _,a0 in pairs(m)do z[_]=-a0 end;return z end;function VectorN.VectorN(B)local bq={}for _,a0 in pairs(B)do if type(a0)=="table"then bq[_]=VectorN.VectorN(a0)else bq[_]=a0 end end;setmetatable(bq,VectorN.mt)return bq end;function Control.PID(ch,ci,cj,ck,cl,cm)local cn={}cn.kP=ch;cn.kI=ci;cn.kD=cj;cn.Iacc=Accumulator.Accumulator(ck,cl)if cm and cm~=0 then cn.period=cm end;return cn end;function Control.processPID(co,cp,e)cp=co.period and(cp+co.period/2)%co.period-co.period/2 or cp;local b4=co.kP*cp;local F,cq=Accumulator.update(co.Iacc,cp,e)F=co.kI*F/cq;local h=co.kD*(cp-(co.lastError or cp))/e;co.lastError=cp;return b4+F+h end;function Control.FF(aM,cm)local cr={}cr.coeffs=aM;cr.degree=#aM-1;if cm and cm~=0 then cr.period=cm end;cr.derivs={}return cr end;function Control.processFF(co,bH,e)local cs=0*bH;local ct=bH;local cu=bH;for F=1,co.degree+1 do cu=co.derivs[F]co.derivs[F]=ct;cs=cs+co.coeffs[F]*ct;if cu then local aD=ct-cu;if F==1 and co.period then aD=(aD+co.period/2)%co.period-co.period/2 end;ct=aD/e else break end end;return cs end;function Nav.toLocal(cv,cw,cx)local cy=cv-cw;return Quaternion.Inverse(cx)*cy end;function Nav.toGlobal(cz,cw,cx)local cy=cx*cz;return cy+cw end;function Nav.cartToPol(cA)local bj=cA.magnitude;local cd=Vector3.SignedAngle(Vector3.forward,cA,Vector3.up)local bf=90-Vector3.Angle(Vector3.up,cA)return Vector3(bj,cd,bf)end;function Nav.cartToCyl(cA)local cB=Vector3(cA.x,0,cA.z)local cC=cB.magnitude;local bf=Vector3.SignedAngle(Vector3.forward,cA,Vector3.up)local bi=cA.y;return Vector3(cC,bf,bi)end;function Nav.polToCart(cA)local bj,cd,bf=cA.x,cA.y,cA.z;local ao=Mathf.Sin(cd)*Mathf.Cos(bf)local cg=Mathf.Sin(bf)local bi=Mathf.Cos(cd)*Mathf.Cos(bf)return bj*Vector3(ao,cg,bi)end;function Nav.cylToCart(cA)local cC,bf,cD=cA.x,cA.y,cA.z;local ao=cC*Mathf.Sin(bf)local cg=cD;local bi=cC*Mathf.Cos(bf)return Vector3(ao,cg,bi)end;function Targeting.firstOrderTargeting(cE,cF,cG)local cH=cE-Vector3.Project(cE,cF)local cI=Vector3.Dot(cF,cE-cH)/cF.sqrMagnitude;local m,n=MathUtil.solveQuadratic(cI-cG*cG,2*cI,cH.sqrMagnitude+cI*cI)local cJ=nil;if m and m>=0 then cJ=m end;if n and n>=0 and n<m then cJ=n end;return cJ and(cE+cJ*cF).normalized or nil end;function Targeting.secondOrderTargeting(cE,cK,cL,cG,cM,cN)local m=-0.25*cL.sqrMagnitude;local n=-Vector3.Dot(cK,cL)local Q=-(cK.sqrMagnitude-cG*cG+Vector3.Dot(cE,cL))local h=-2*Vector3.Dot(cE,cK)local cp=-cE.sqrMagnitude;local g;local cO=cL.magnitude;local cP=cK.magnitude;local cQ=cE.magnitude;local cR,cS=MathUtil.solveQuadratic(0.5*cO,cP+cG,-cQ)local cT=math.max(cR,cS)local cU;local aM={0.5*cO,cP-cG,cQ}if MathUtil.ruleOfSigns(aM,0)==2 then local cV,cW=MathUtil.solveQuadratic(aM[1],aM[2],aM[3])if cV then cU=math.min(cV,cW)end end;if not cU or cU<cT then local b2,b3,b9=MathUtil.solveCubic(4*m,3*n,2*Q,h)if not b9 then if b2>cT then cU=b2 end else local cV=math.min(b2,b9)local cW=math.max(b2,b9)if cV>cT then cU=cV elseif cW>cT then cU=cW end end;if not cU then return end end;local function cX(ao)return cp+ao*(h+ao*(Q+ao*(n+ao*m)))end;g=MathUtil.ITP(cX,cT,cU,1e-4,25)if not g then return end;if g>=cT and g<=cU then local cY=cE+cK*g+0.5*cL*g*g;if cY.magnitude>=cM and cY.magnitude<=cN then return cY,g end end end;function Targeting.AIPPN(cZ,cE,c_,cF,d0)local cK=cF-c_;local d1=Vector3.Dot(-cK,cE.normalized)if d1<=0 then d1=10 end;local d2=cE.magnitude/d1;local d3=Vector3.Cross(cE,cK)/cE.sqrMagnitude;local d4=Vector3.Cross(cE,d0)/cE.sqrMagnitude*d2/2;local d5=d3+d4;local d6=Vector3.Cross(d5,cE.normalized)local d7=Vector3.ProjectOnPlane(d6,c_).normalized;local d8=cZ*c_.magnitude*d5.magnitude;return d8*d7 end;function Targeting.ATPN(cZ,cE,c_,cF,d0)local cK=cF-c_;local d1=-Vector3.Dot(cK,cE.normalized)if d1<=0 then d1=10 end;local d3=Vector3.Cross(cE,cK)/cE.sqrMagnitude;local d6=Vector3.Cross(d3,cE.normalized)local d9=Vector3.ProjectOnPlane(d0,cE)return cZ*d1*d6+0.5*cZ*d0 end;function BlockUtil.getWeaponsByName(da,db,aW,dc)if DEBUG then da:Log("searching for "..db)end;local dd={}aW=aW or-1;local Q=aW;if not dc or dc==0 or dc==2 then for F=0,da:GetWeaponCount()-1 do if Q==0 then break end;if da:GetWeaponBlockInfo(F).CustomName==db then table.insert(dd,{subIdx=nil,wpnIdx=F})if DEBUG then da:Log("found weapon "..db.." on hull, type "..da:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end;if not dc or dc==1 or dc==2 then local de=da:GetAllSubConstructs()for p=1,#de do local bb=de[p]for F=0,da:GetWeaponCountOnSubConstruct(bb)-1 do if Q==0 then break end;if da:GetWeaponBlockInfoOnSubConstruct(bb,F).CustomName==db then table.insert(dd,{subIdx=bb,wpnIdx=F})if DEBUG then da:Log("found weapon "..db.." on subobj "..bb..", type "..da:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end;Q=Q-1 end end end end;if DEBUG then da:Log("weapon count: "..#dd)end;return dd end;function BlockUtil.getSubConstructsByName(da,db,aW)if DEBUG then da:Log("searching for "..db)end;local de=da:GetAllSubConstructs()local df={}aW=aW or-1;local Q=aW;for p=1,#de do local bb=de[p]if Q==0 then break end;if da:GetSubConstructInfo(bb).CustomName==db then table.insert(df,bb)if DEBUG then da:Log("found subobj "..db)end;Q=Q-1 end end;if DEBUG then da:Log("subobj count: "..#df)end;return df end;function BlockUtil.getBlocksByName(da,db,dg,aW)if DEBUG then da:Log("searching for "..db)end;local dh={}aW=aW or-1;local Q=aW;for p=0,da:Component_GetCount(dg)-1 do if Q==0 then break end;if da:Component_GetBlockInfo(dg,p).CustomName==db then table.insert(dh,p)if DEBUG then da:Log("found component "..db)end;Q=Q-1 end end;if DEBUG then da:Log("component count: "..#dh)end;return dh end;function BlockUtil.populateWeaponsByName(da,dc)if DEBUG then da:Log("populating all weapons, mode "..dc)end;local dd={}for p=0,da:GetWeaponCount()-1 do local db=da:Component_GetBlockInfo(type,p).CustomName;if db and db~=''then dd[db]=dd[db]or{}table.insert(dd[db],{subIdx=nil,wpnIdx=p})if DEBUG then da:Log("found weapon "..db.." on hull, type "..da:GetWeaponInfo(p).WeaponType)end else table.insert(dd,{subIdx=nil,wpnIdx=p})if DEBUG then da:Log("found unnamed weapon on hull, type "..da:GetWeaponInfo(p).WeaponType)end end end;if not dc or dc==1 or dc==2 then local de=da:GetAllSubConstructs()for p=1,#de do local bb=de[p]for F=0,da:GetWeaponCountOnSubConstruct(bb)-1 do local db=da:Component_GetBlockInfo(type,F).CustomName;if db and db~=''then dd[db]=dd[db]or{}table.insert(dd[db],{subIdx=bb,wpnIdx=F})if DEBUG then da:Log("found weapon "..db.." on subobj "..bb..", type "..da:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end else table.insert(dd,{subIdx=bb,wpnIdx=F})if DEBUG then da:Log("found unnamed weapon on subobj "..bb..", type "..da:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end end end end end;if DEBUG then local aW=0;for _,a0 in pairs(dd)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;da:Log("weapon count: "..aW)end;return dd end;function BlockUtil.populateSubConstructsByName(da)if DEBUG then da:Log("populating all subconstructs")end;local de=da:GetAllSubConstructs()local df={}for p=1,#de do local bb=de[p]local db=da:GetSubConstructInfo(bb).CustomName;if db and db~=''then df[db]=df[db]or{}table.insert(df[db],bb)if DEBUG then da:Log("found subobj "..db)end else table.insert(df,bb)if DEBUG then da:Log("found unnamed subobj")end end end;if DEBUG then local aW=0;for _,a0 in pairs(df)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;da:Log("subobject count: "..aW)end;return df end;function BlockUtil.populateBlocksByName(da,dg)if DEBUG then da:Log("populating all blocks of type "..dg)end;local dh={}for p=0,da:Component_GetCount(dg)-1 do local db=da:Component_GetBlockInfo(dg,p).CustomName;if db and db~=''then dh[db]=dh[db]or{}table.insert(dh[db],p)if DEBUG then da:Log("found component "..db)end else table.insert(dh,p)if DEBUG then da:Log("found unnamed component of type "..dg)end end end;if DEBUG then local aW=0;for _,a0 in pairs(dh)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;da:Log("component count: "..aW)end;return dh end;function BlockUtil.getWeaponInfo(da,di)if di.subIdx then return da:GetWeaponInfoOnSubConstruct(di.subIdx,di.wpnIdx)end;return da:GetWeaponInfo(di.wpnIdx)end;function BlockUtil.getWeaponBlockInfo(da,di)if di.subIdx then return da:GetWeaponBlockInfoOnSubConstruct(di.subIdx,di.wpnIdx)end;return da:GetWeaponBlockInfo(di.wpnIdx)end;function BlockUtil.aimWeapon(da,di,dj,dk)if di.subIdx then da:AimWeaponInDirectionOnSubConstruct(di.subIdx,di.wpnIdx,dj.x,dj.y,dj.z,dk)else da:AimWeaponInDirection(di.wpnIdx,dj.x,dj.y,dj.z,dk)end end;function BlockUtil.fireWeapon(da,di,dk)if di.subIdx then return da:FireWeaponOnSubConstruct(di.subIdx,di.wpnIdx,dk)end;return da:FireWeapon(di.wpnIdx,dk)end;function Combat.pickTarget(da,dl,dm)dm=dm or function(U,dn)return dn.Priority end;local bH,dp;for F in MathUtil.range(da:GetNumberOfTargets(dl))do local dn=da:GetTargetInfo(dl,F)local dq=dm(da,dn)if not bH or dq>dp then bH=dn;dp=dq end end;return bH end;function Combat.CheckConstraints(da,dr,ds,dt)local du;if dt then du=da:GetWeaponConstraintsOnSubConstruct(dt,ds)else du=da:GetWeaponConstraints(ds)end;local dv=da:GetConstructForwardVector()local dw=da:GetConstructUpVector()local dx=Quaternion.LookRotation(dv,dw)dr=Quaternion.Inverse(dx)*dr;if du.InParentConstructSpace and dt then local dy=da:GetSubConstructInfo(dt).localRotation;dr=Quaternion.inverse(dy)*dr end;local dz=MathUtil.angleOnPlane(Vector3.forward,dr,Vector3.up)local dA=dr;dA.z=0;local O=Mathf.Atan2(dr.z,dA.magnitude)local dB=dz>du.MinAzimuth and dz<du.MaxAzimuth;local dC=O>du.MinElevation and O<du.MaxElevation;if du.FlipAzimuth then dB=not dB end;if dB and dC then return true end;dz=dz+180;O=180-O;if O>180 then O=O-360 end;if O<-180 then O=O+360 end;dB=dz>du.MinAzimuth and dz<du.MaxAzimuth;dC=O>du.MinElevation and O<du.MaxElevation;if du.FlipAzimuth then dB=not dB end;if dB and dC then return true end;return false end;function StringUtil.LogVector(da,bq,dD)da:Log(dD.."("..bq.x..", "..bq.y..", "..bq.z..")")end
