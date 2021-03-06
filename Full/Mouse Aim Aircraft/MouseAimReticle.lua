local Accumulator = {}
local Differ = {}
local LinkedList = {}
local MathUtil = {}
local RingBuffer = {}
local Control = {}
local Nav = {}
local Targeting = {}
local BlockUtil = {}
local Combat = {}

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
local pitchS = {  P = 0.04,
                  I = 0.1,
                  D = 0.015,
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
local hudDistance = 0.4

-- the name of the spinblocks that stabilize the chair
local chairSpinblockNames = { yaw = "yawSeat",
                              pitch = "pitchSeat",
                              roll = "rollSeat" }

-- the name of the weapon used to steer the craft
local steeringWepName = "joystick"

-- "leading" mode will tell you where to shoot
  -- "trailing" mode will tell you where you'll hit if you shoot now
local sightMode = "trailing"

local sightProjNames = {  aim = "aimProjector",
                          lead = "leadProjector",
                          lock = "lockProjector"  }

-- shell velocity of your cannon
local muzzle = 1129

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
local target
local accelAcc
local pitchPID, pitchFF
local yawPID, yawFF
local rollPID, rollFF
local proj = {}
local lastDir
local gunOffset

local steeringWep
local mainWeps
local chairSpinblocks = {}
local inited = false
local DEBUG = true

function Init(I)
  for block, name in pairs(chairSpinblockNames) do
    chairSpinblocks[block] = BlockUtil.getSubConstructsByName(I, name, 1)[1]
  end
  steeringWep = BlockUtil.getWeaponsByName(I, steeringWepName, 1)[1]
  mainWeps = BlockUtil.getWeaponsByName(I, mainWeaponName, -1)
  gunOffset = Vector3.zero
  for i, wep in ipairs(mainWeps) do
    local info = BlockUtil.getWeaponInfo(I, wep)
    gunOffset = gunOffset + info.LocalFirePoint
  end
  gunOffset = gunOffset / #mainWeps - I:GetSubConstructInfo(chairSpinblocks.roll).LocalPosition - Vector3.forward

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

  RotateSeat(I)

  local oldTarget = target
  target = Combat.pickTarget(I, 0, Priority)
  if not target or (oldTarget and oldTarget.Id ~= target.Id) then
    SwapTarget(target)
  end

  if target and accelAvgTime > 0 then
    local dv = Differ.update(velDiff, target.Velocity) / elapsedTime
    Accumulator.update(accelAcc, dv, elapsedTime)
  end
  Sight(I, target)
  Steer(I)
end

function Priority(I, t)
  local relPos = t.Position - I:GetConstructPosition()
  local rangePenalty = 0
  if relPos.magnitude > lockRange then rangePenalty = 1000000 end
  return -Vector3.Angle(relPos, BlockUtil.getWeaponInfo(I, steeringWep).CurrentDirection) - rangePenalty
end

function RotateSeat(I)
  local av = I:GetLocalAngularVelocity() * Differ.get(timeDiff)
  I:SetSpinBlockRotationAngle(chairSpinblocks.yaw, -I:GetConstructYaw() - Mathf.Rad2Deg * av.y)
  I:SetSpinBlockRotationAngle(chairSpinblocks.pitch, I:GetConstructPitch() + Mathf.Rad2Deg * av.x)
  I:SetSpinBlockRotationAngle(chairSpinblocks.roll, -I:GetConstructRoll() - Mathf.Rad2Deg * av.z)
end

function PositionReticle(I, aim, lead, lock)
  local rotation = Quaternion.LookRotation(I:GetConstructForwardVector(), I:GetConstructUpVector())
  local invRot = Quaternion.Inverse(rotation)
  local seatPosition = I:GetSubConstructInfo(chairSpinblocks.roll).Position
  local seatPosLoc = invRot * (seatPosition - I:GetConstructPosition())
  local fw = I:GetConstructForwardVector()
  seatPosLoc = seatPosLoc + 0.5 * Vector3.forward
  local vecs = {aim = aim, lead = lead, lock = lock}
  local r = (Mathf.Repeat(I:GetConstructRoll() + 90, 180) - 90)
  for p, id in pairs(proj) do
    vecs[p] = vecs[p] + Quaternion.AngleAxis(-r, Vector3.forward) * hologramOffset
    if vecs[p].z >= 0 then
      vecs[p].z = 0
      local info = I:Component_GetBlockInfo(33, id)
      local infoPosLoc = invRot * (info.Position - I:GetConstructPosition())
      local dest = (vecs[p] + hudDistance * Vector3.forward).normalized * hudDistance + seatPosLoc
      local diff = dest - infoPosLoc
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
    local headPos = I:GetSubConstructInfo(chairSpinblocks.roll).Position + fw
    local relPos = target.Position - headPos
    local relVel = target.Velocity - I:GetVelocityVector()
    local grav = I:GetGravityForAltitude(I:GetConstructPosition().y)
    local ac, w
    if accelAvgTime > 0 then ac, w = Accumulator.get(accelAcc) else ac, w = Vector3.zero, 1 end
    local accel = ac / w - grav
    local a, b = MathUtil.solveQuadratic(0.5 * accel.magnitude, muzzle + relVel.magnitude, -relPos.magnitude)
    local minTTT
    if not b then minTTT = a
    elseif a > 0 and (a < b or b < 0) then minTTT = a
    elseif b > 0 and (b < a or a < 0) then minTTT = b end
    minTTT = minTTT / 2 or 0
    a, b = MathUtil.solveQuadratic(-0.5 * accel.magnitude, muzzle - relVel.magnitude, -relPos.magnitude)
    local maxTTT
    if not b then maxTTT = a
    elseif a > 0 and (a < b or b < 0) then maxTTT = a
    elseif b > 0 and (b < a or a < 0) then maxTTT = b end
    maxTTT = maxTTT or (1/0) --infinity
    local t = Targeting.secondOrderTargetingTime(relPos + gunOffset, relVel, accel, muzzle, minTTT, maxTTT)
    lockOffset = invRot * relPos / Mathf.Abs(Vector3.Dot(relPos, fw)) * (hudDistance - offset.z)
    if t then
      local enemyPosition = relPos + t * relVel + 0.5 * t * t * accel
      if Vector3.Angle(fw, enemyPosition) < autoAimAngle
       and enemyPosition.magnitude < autoFireRange then
        local headLoc = invRot * (headPos - I:GetConstructPosition())
        local optPointing = (enemyPosition - rotation * gunOffset).normalized
        for i, wep in ipairs(mainWeps) do
          local info = BlockUtil.getWeaponInfo(I, wep)
          if not info.PlayerCurrentlyControllingIt then
            local diff = enemyPosition + rotation * (headLoc - info.LocalFirePoint)
            diff = MathUtil.clampCone(I:GetConstructForwardVector(), diff, maxFireAngle)
            aimDir = invRot * optPointing
            pointing = optPointing
            BlockUtil.aimWeapon(I, wep, diff, 0)
            BlockUtil.fireWeapon(I, wep, 0)
          end
        end
      end
      if sightMode == "trailing" then
        pointing = MathUtil.clampCone(I:GetConstructForwardVector(), pointing, maxFireAngle)
        local shellPosition = t * (muzzle * pointing - relVel) + 0.5 * t * t * (-accel)
        local shellPosLoc = invRot * shellPosition + gunOffset

        leadOffset = offset + (hudDistance - offset.z) / Mathf.Abs(shellPosLoc.z) * shellPosLoc
      elseif sightMode == "leading" then
        local enemyPosLoc = invRot * enemyPosition

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
  target = newTarget
  velDiff = Differ.Differ(newTarget and newTarget.Velocity or Vector3.zero)
  accelAcc = Accumulator.Accumulator(accelAvgTime, accelDecay)
end

function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function LinkedList.LinkedList()local k={}k.value=nil;k.next=k;k.prev=k;return k end;function LinkedList.pushFront(l,d)local k={}k.value=d;LinkedList.connect(k,l.next)LinkedList.connect(l,k)end;function LinkedList.pushBack(l,d)local k={}k.value=d;LinkedList.connect(l.prev,k)LinkedList.connect(k,l)end;function LinkedList.popFront(l)local m=l.next.value;LinkedList.connect(l,l.next.next)return m end;function LinkedList.popBack(l)local m=l.prev.value;LinkedList.connect(l.prev.prev,l)return m end;function LinkedList.peekFront(l)return l.next.val end;function LinkedList.peekBack(l)return l.prev.val end;function LinkedList.connect(n,o)n.next=o;o.prev=n end;function LinkedList.toArray(l)local p=1;local q={}local k=l.next;while k~=l do q[p]=k.value;k=k.next end;return q end;function MathUtil.angleOnPlane(r,s,t)local u=Vector3.ProjectOnPlane(r,t)local g=Vector3.ProjectOnPlane(s,t)return Vector3.SignedAngle(u,g,t)end;function MathUtil.min(v,w)local x=nil;w=w or function(y,z)return y<z end;for ele in v do if not x or w(ele,x)then x=ele end end;return x end;function MathUtil.max(v,w)local A=nil;w=w or function(y,z)return y<z end;for ele in v do if not A or w(A,ele)then A=ele end end;return A end;function MathUtil.range(y,z,B)local C,D=y,z;local E;if not y then return end;if not z then C=0;D=y;E=C<D and 1 or-1 elseif B then E=B end;return function(F,G)local H=G+E;if H==D then return nil end;return H end,nil,C-E end;function MathUtil.angleSSS(y,z,B)if y+z<B or y+B<z or z+B<y then return nil end;local I=Mathf.Acos((z*z+B*B-y*y)/(2*z*B))*Mathf.Rad2Deg;local J,K=MathUtil.angleSAS(z,I,B)return I,J,K end;function MathUtil.sideSAS(y,K,z)local L=y*y+z*z-2*y*z*Mathf.Cos(K*Mathf.Deg2Rad)return Mathf.Sqrt(L)end;function MathUtil.angleSAS(y,K,z)local B=MathUtil.sideSAS(y,K,z)if MathUtil.isZero(B)then return nil end;local I,J;if y<z then I=MathUtil.angleLoSin(B,y,K)J=180-I-K else J=MathUtil.angleLoSin(B,z,K)I=180-J-K end;return I,J end;function MathUtil.sideSSA(y,z,I)local M=z*z-y*y;local N=-2*z*math.cos(math.rad(I))local O,P=MathUtil.solveQuadratic(1,N,M)if not P then return O,P end;if O<P then return O,P end;return P,O end;function MathUtil.angleSSA(y,z,I)local O,P=MathUtil.sideSSA(y,z,I)if not O then return nil end;local Q,R=MathUtil.angleSAS(z,I,O)if not P then return Q,R end;local S,T=MathUtil.angleSAS(z,I,P)return Q,R,S,T end;function MathUtil.sideAAS(I,J,y)local K=180-I-J;local z=MathUtil.sideLoSin(I,J,y)local B=MathUtil.sideLoSin(I,K,y)return z,B end;function MathUtil.sideLoSin(y,I,J)return y*Mathf.Sin(J*Mathf.Deg2Rad)/Mathf.Sin(I*Mathf.Deg2Rad)end;function MathUtil.angleLoSin(y,z,I)return Mathf.Asin(z*Mathf.Sin(I*Mathf.Deg2Rad)/y)*Mathf.Rad2Deg end;function MathUtil.clampCone(U,V,W)local X=Mathf.Min(W,Vector3.Angle(U,V))local Y=Vector3.Cross(U,V)return Quaternion.AngleAxis(X,Y)*U end;local Z=1e-9;function MathUtil.isZero(h)return h>-Z and h<Z end;function MathUtil.cuberoot(_)return _>0 and _^(1/3)or-math.abs(_)^(1/3)end;function MathUtil.solveQuadratic(a0,O,P)local a1,a2;local a3,a4,a5;a3=O/(2*a0)a4=P/a0;a5=a3*a3-a4;if MathUtil.isZero(a5)then a1=-a3;return a1 elseif a5<0 then return else local a6=math.sqrt(a5)a1=a6-a3;a2=-a6-a3;return a1,a2 end end;function MathUtil.solveCubic(a0,O,P,a7)local a1,a2,a8;local a9,aa;local I,J,K;local ab,a3,a4;local ac,a5;I=O/a0;J=P/a0;K=a7/a0;ab=I*I;a3=1/3*(-(1/3)*ab+J)a4=0.5*(2/27*I*ab-1/3*I*J+K)ac=a3*a3*a3;a5=a4*a4+ac;if MathUtil.isZero(a5)then if MathUtil.isZero(a4)then a1=0;a9=1 else local ad=MathUtil.cuberoot(-a4)a1=2*ad;a2=-ad;a9=2 end elseif a5<0 then local ae=1/3*math.acos(-a4/math.sqrt(-ac))local g=2*math.sqrt(-a3)a1=g*math.cos(ae)a2=-g*math.cos(ae+math.pi/3)a8=-g*math.cos(ae-math.pi/3)a9=3 else local a6=math.sqrt(a5)local ad=MathUtil.cuberoot(a6-a4)local af=-MathUtil.cuberoot(a6+a4)a1=ad+af;a9=1 end;aa=1/3*I;if a9>0 then a1=a1-aa end;if a9>1 then a2=a2-aa end;if a9>2 then a8=a8-aa end;return a1,a2,a8 end;function MathUtil.solveQuartic(a0,O,P,a7,ag)local a1,a2,a8,ah;local ai={}local aj,ad,af,aa;local I,J,K,a5;local ab,a3,a4,ak;local a9;I=O/a0;J=P/a0;K=a7/a0;a5=ag/a0;ab=I*I;a3=-0.375*ab+J;a4=0.125*ab*I-0.5*I*J+K;ak=-(3/256)*ab*ab+0.0625*ab*J-0.25*I*K+a5;if MathUtil.isZero(ak)then ai[3]=a4;ai[2]=a3;ai[1]=0;ai[0]=1;local al={MathUtil.solveCubic(ai[0],ai[1],ai[2],ai[3])}a9=#al;a1,a2,a8=al[1],al[2],al[3]else ai[3]=0.5*ak*a3-0.125*a4*a4;ai[2]=-ak;ai[1]=-0.5*a3;ai[0]=1;a1,a2,a8=MathUtil.solveCubic(ai[0],ai[1],ai[2],ai[3])aj=a1;ad=aj*aj-ak;af=2*aj-a3;if MathUtil.isZero(ad)then ad=0 elseif ad>0 then ad=math.sqrt(ad)else return end;if MathUtil.isZero(af)then af=0 elseif af>0 then af=math.sqrt(af)else return end;ai[2]=aj-ad;ai[1]=a4<0 and-af or af;ai[0]=1;do local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=#al;a1,a2=al[1],al[2]end;ai[2]=aj+ad;ai[1]=a4<0 and af or-af;ai[0]=1;if a9==0 then local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=a9+#al;a1,a2=al[1],al[2]end;if a9==1 then local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=a9+#al;a2,a8=al[1],al[2]end;if a9==2 then local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=a9+#al;a8,ah=al[1],al[2]end end;aa=0.25*I;if a9>0 then a1=a1-aa end;if a9>1 then a2=a2-aa end;if a9>2 then a8=a8-aa end;if a9>3 then ah=ah-aa end;return a1,a2,a8,ah end;function RingBuffer.RingBuffer(am)local an={}an.buf={}an.capacity=am;an.tail=1;an.head=1;return an end;function RingBuffer.isFull(an)return(an.head-an.tail)%an.capacity==1 end;function RingBuffer.isEmpty(an)return an.head==an.tail end;function RingBuffer.push(an,d)an.buf[an.tail]=d;if RingBuffer.isFull(an)then an.head=an.head%an.capacity+1 end;an.tail=an.tail%an.capacity+1 end;function RingBuffer.pop(an)if RingBuffer.isEmpty(an)then return nil end;local m=an.buf[an.head]an.buf[an.head]=nil;an.head=an.head%an.capacity+1;return m end;function Control.PID(ao,ap,aq,ar,as,at)local au={}au.kP=ao;au.kI=ap;au.kD=aq;au.Iacc=Accumulator.Accumulator(ar,as)if at and at~=0 then au.period=at end;return au end;function Control.processPID(av,aw,e)aw=av.period and(aw+av.period/2)%av.period-av.period/2 or aw;local a3=av.kP*aw;local p=av.kI*Accumulator.update(av.Iacc,aw,e)local h=av.kD*(aw-(av.lastError or aw))/e;av.lastError=aw;return a3+p+h end;function Control.FF(ai,at)local ax={}ax.coeffs=ai;ax.degree=#ai-1;if at and at~=0 then ax.period=at end;ax.derivs={}return ax end;function Control.processFF(av,ay,e)local az=0*ay;local aA=ay;local aB=ay;for p=1,av.degree+1 do aB=av.derivs[p]av.derivs[p]=aA;az=az+av.coeffs[p]*aA;if aB then local aC=aA-aB;if p==1 and av.period then aC=(aC+av.period/2)%av.period-av.period/2 end;aA=aC/e else break end end;return az end;function Nav.toLocal(aD,aE,aF)local aG=aD-aE;return Quaternion.Inverse(aF)*aG end;function Nav.toGlobal(aH,aE,aF)local aG=aF*aH;return aG+aE end;function Nav.cartToPol(aI)local ak=aI.magnitude;local aJ=Vector3.SignedAngle(Vector3.forward,aI,Vector3.up)local ae=90-Vector3.Angle(Vector3.up,aI)return Vector3(ak,aJ,ae)end;function Nav.cartToCyl(aI)local aK=Vector3(aI.x,0,aI.z)local aL=aK.magnitude;local ae=Vector3.SignedAngle(Vector3.forward,aI,Vector3.up)local aj=aI.y;return Vector3(aL,ae,aj)end;function Nav.polToCart(aI)local ak,aJ,ae=aI.x,aI.y,aI.z;local _=Mathf.Sin(aJ)*Mathf.Cos(ae)local aM=Mathf.Sin(ae)local aj=Mathf.Cos(aJ)*Mathf.Cos(ae)return ak*Vector3(_,aM,aj)end;function Nav.cylToCart(aI)local aL,ae,aN=aI.x,aI.y,aI.z;local _=aL*Mathf.Sin(ae)local aM=aN;local aj=aL*Mathf.Cos(ae)return Vector3(_,aM,aj)end;function Targeting.firstOrderTargeting(aO,aP,aQ)local aR=Vector3.Angle(-aO,aP)local aS,O,aT,P=MathUtil.angleSSA(aQ,aP.magnitude,aR)if not aS then return nil end;local aU=aT or aS;if not aU then return nil end;return(Quaternion.AngleAxis(aU,Vector3.Cross(aO,aP))*aO).normalized end;function Targeting.secondOrderTargeting(aO,aV,aW,aQ,aX,aY)local g=Targeting.secondOrderTargetingTime(aO,aV,aW,aQ,aX/aQ,aY/aQ)if g and g>0 then return(aO/g+aV-0.5*aW*g).normalized end;return nil end;function Targeting.secondOrderTargetingTime(aO,aV,aW,aQ,aZ,a_)local y=0.25*aW.sqrMagnitude;local z=Vector3.Dot(aV,aW)local B=aV.sqrMagnitude-aQ*aQ+Vector3.Dot(aO,aW)local h=2*Vector3.Dot(aO,aV)local aw=aO.sqrMagnitude;local b0={MathUtil.solveQuartic(y,z,B,h,aw)}local g=nil;for p=1,4 do if b0[p]and b0[p]>aZ and b0[p]<a_ then if not g or g and b0[p]<g then g=b0[p]end end end;return g end;function Targeting.AIPPN(b1,aO,b2,aP,b3)local aV=aP-b2;local b4=Vector3.Dot(-aV,aO.normalized)if b4<=0 then b4=10 end;local b5=aO.magnitude/b4;local b6=Vector3.Cross(aO,aV)/aO.sqrMagnitude;local b7=Vector3.Cross(aO,b3)/aO.sqrMagnitude*b5/2;local b8=b6+b7;local b9=Vector3.Cross(b8,aO.normalized)local ba=Vector3.ProjectOnPlane(b9,b2).normalized;local bb=b1*b2.magnitude*b8.magnitude;return bb*ba end;function Targeting.ATPN(b1,aO,b2,aP,b3)local aV=aP-b2;local b4=-Vector3.Dot(aV,aO.normalized)if b4<=0 then b4=10 end;local b6=Vector3.Cross(aO,aV)/aO.sqrMagnitude;local b9=Vector3.Cross(b6,aO.normalized)local bc=Vector3.ProjectOnPlane(b3,aO)return b1*b4*b9+0.5*b1*b3 end;function Targeting.accelToDirection(bd,be,e)local bf=Vector3.Cross(bd,be)/bd.sqrMagnitude*e*Mathf.Rad2Deg;return Quaternion.AngleAxis(bf.magnitude,bf)*bd end;function BlockUtil.getWeaponsByName(bg,bh,bi,bj)if DEBUG then bg:Log("searching for "..bh)end;local bk=bg:GetAllSubConstructs()local bl={}bi=bi or-1;local B=bi;if not bj or bj==0 or bj==2 then for p=0,bg:GetWeaponCount()-1 do if B==0 then break end;if bg:GetWeaponBlockInfo(p).CustomName==bh then table.insert(bl,{subIdx=nil,wpnIdx=p})if DEBUG then bg:Log("found weapon "..bh.." on hull, type "..bg:GetWeaponInfo(p).WeaponType)end;B=B-1 end end end;if not bj or bj==1 or bj==2 then for bm=1,#bk do local aa=bk[bm]for p=0,bg:GetWeaponCountOnSubConstruct(aa)-1 do if B==0 then break end;if bg:GetWeaponBlockInfoOnSubConstruct(aa,p).CustomName==bh then table.insert(bl,{subIdx=aa,wpnIdx=p})if DEBUG then bg:Log("found weapon "..bh.." on subobj "..aa..", type "..bg:GetWeaponInfo(p).WeaponType)end;B=B-1 end end end end;if DEBUG then bg:Log("weapon count: "..#bl)end;return bl end;function BlockUtil.getSubConstructsByName(bg,bh,bi)if DEBUG then bg:Log("searching for "..bh)end;local bk=bg:GetAllSubConstructs()local bn={}bi=bi or-1;local B=bi;for bm=1,#bk do local aa=bk[bm]if B==0 then break end;if bg:GetSubConstructInfo(aa).CustomName==bh then table.insert(bn,aa)if DEBUG then bg:Log("found subobj "..bh)end;B=B-1 end end;if DEBUG then bg:Log("subobj count: "..#bn)end;return bn end;function BlockUtil.getBlocksByName(bg,bh,bo,bi)if DEBUG then bg:Log("searching for "..bh)end;local bp={}bi=bi or-1;local B=bi;for bm=0,bg:Component_GetCount(bo)-1 do if B==0 then break end;if bg:Component_GetBlockInfo(bo,bm).CustomName==bh then table.insert(bp,bm)if DEBUG then bg:Log("found component "..bh)end;B=B-1 end end;if DEBUG then bg:Log("component count: "..#bp)end;return bp end;function BlockUtil.getWeaponInfo(bg,bq)local br;if bq.subIdx then br=bg:GetWeaponInfoOnSubConstruct(bq.subIdx,bq.wpnIdx)else br=bg:GetWeaponInfo(bq.wpnIdx)end;return br end;function BlockUtil.aimWeapon(bg,bq,bs,bt)if bq.subIdx then bg:AimWeaponInDirectionOnSubConstruct(bq.subIdx,bq.wpnIdx,bs.x,bs.y,bs.z,bt)else bg:AimWeaponInDirection(bq.wpnIdx,bs.x,bs.y,bs.z,bt)end end;function BlockUtil.fireWeapon(bg,bq,bt)if bq.subIdx then bg:FireWeaponOnSubConstruct(bq.subIdx,bq.wpnIdx,bt)else bg:FireWeapon(bq.wpnIdx,bt)end end;function Combat.pickTarget(bg,bu,bv)bv=bv or function(F,bw)return bw.Priority end;local ay,bx;for p in MathUtil.range(bg:GetNumberOfTargets(bu))do local bw=bg:GetTargetInfo(bu,p)local by=bv(bg,bw)if not ay or by>bx then ay=bw;bx=by end end;return ay end;function CheckConstraints(bg,bz,bA,bB)local bC;if bB then bC=bg:GetWeaponConstraintsOnSubConstruct(bB,bA)else bC=bg:GetWeaponConstraints(bA)end;local bD=bg:GetConstructForwardVEctor()local bE=bg:GetConstructUpVector()local bF=Quaternion.LookRotation(bD,bE)bz=Quaternion.Inverse(bF)*bz;if bC.InParentConstructSpace and bB then local bG=bg:GetSubConstructInfo(bB).localRotation;bz=Quaternion.inverse(bG)*bz end;local bH=MathUtil.angleOnPlane(Vector3.forward,bz,Vector3.up)local bI=bz;bI.z=0;local bJ=Mathf.Atan2(bz.z,bI.magnitude)local bK=bH>bC.MinAzimuth and bH<bC.MaxAzimuth;local bL=bJ>bC.MinElevation and bJ<bC.MaxElevation;if bC.FlipAzimuth then bK=not bK end;if bK and bL then return true end;bH=bH+180;ele=180-ele;if ele>180 then ele=ele-360 end;if ele<-180 then ele=ele+360 end;bK=bH>bC.MinAzimuth and bH<bC.MaxAzimuth;bL=bJ>bC.MinElevation and bJ<bC.MaxElevation;if bC.FlipAzimuth then bK=not bK end;if bK and bL then return true end;return false end
