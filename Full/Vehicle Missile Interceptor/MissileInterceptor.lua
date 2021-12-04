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
-- how far ahead the missile tried to lead the target to beamride into it head on
local maxTargetLead = 0.6
-- takes weighted average of TPG and PP guidance
-- set to -1 to use variable weight that scales inversely from 0 to 1
local tpgWeight = 1
-- max range to pursue missiles in
local maxRange = 1200
-- Flight settings
-- throttle to use for idle flight
local idleThrottle = 0.06
-- whether to use heading or velocity to maneuver
local maneuverByVelocity = true

-- names of spinblocks that hold the arms
local spinBlockNames = {  top = "topSpin",
                          left = "leftSpin",
                          right = "rightSpin",
                          bottom = "bottomSpin" }

-- Error is represented by rotation vectors instead of degrees
--   so these should be about 90 times the constants for the built-in PID
-- PID settings
local PIDSet = {
  P = 1.25,
  I = 0.1,
  D = 0.3,
  IWindow = 0.2,
  IDecay = 0.1
}

local PIDSetIdle = {
  P = 1,
  I = 0,
  D = 0.6,
  IWindow = 0,
  IDecay = 0
}

-- lower and upper altitude limits
  -- note: terrain avoidance not implemented yet
local altFloorWater, altFloorLand, altCeil = 50, 80, 800
-- altitude to fly at when idling
local idleAlt = 100
-- radius to circle at when idling
local idleRadius = 150
-- adjuster is turned off within this distance of the target
-- to hit submarine, land, or space targets
local altViolationZone = 100
-- minimum altitude of missiles to try to intercept
local minHeight = 5

-- averaging time and decay rate for acceleration
local accelAvgTime = 0.5
local accelAvgDecay = 0.25

-- misc variables
local timeDiff
local accelAcc
local combatPid, idlePid
local pid
local currentTargetId
local inited = false
local circlePoint
local armBlocks = {}

function Init(I)
  timeDiff = Differ.Differ(I:GetGameTime())
  accelAcc = Accumulator.Accumulator(accelAvgTime, accelAvgDecay)
  for s, name in pairs(spinBlockNames) do
    armBlocks[s] = BlockUtil.getSubConstructsByName(I, name, 1)[1]
  end
  combatPid = Control.PID(PIDSet.P, PIDSet.I, PIDSet.D, PIDSet.IWindow, PIDSet.IDecay)
  idlePid = Control.PID(PIDSetIdle.P, PIDSetIdle.I, PIDSetIdle.D, PIDSetIdle.IWindow, PIDSetIdle.IDecay)
end

function Update(I)
  if not inited then
    Init(I)
    inited = true
    return
  end

  I:TellAiThatWeAreTakingControl()

  local elapsedTime = Differ.update(timeDiff, I:GetGameTime())

  local desired
  -- determine whether to be in combat or idle mode
  if I:GetNumberOfWarnings() == 0 then
    ResetTargetInfo()
    I:RequestControl(2, 8, idleThrottle)
    pid = idlePid
    desired = Adjust(I, Idle(I))
    for s, id in pairs(armBlocks) do I:SetSpinBlockRotationAngle(id, 0) end
  else
    local target = CheckTarget(I, currentTargetId)

    if not target or not target.Valid or (target.Position - I:GetConstructPosition()).magnitude > maxRange then
      ResetTargetInfo()
      I:RequestControl(2, 8, idleThrottle)
      pid = idlePid
      desired = Adjust(I, Idle(I))
      for s, id in pairs(armBlocks) do I:SetSpinBlockRotationAngle(id, 0) end
    else
      currentTargetId = target.Id

      I:RequestControl(2, 8, 1)
      pid = combatPid
      desired = FinalGuidance(I, target)
      local targetRoll = Vector3.SignedAngle(I:GetConstructUpVector(), target.Position - I:GetConstructPosition(), I:GetConstructForwardVector())
      local angles = {top = 0, right = 90, bottom = 180, left = 270}
      local bestAngle = math.huge
      for s, id in pairs(armBlocks) do
        local diff = Mathf.Repeat(targetRoll - angles[s] + 180, 360) - 180
        if I:IsAlive(id) and diff < bestAngle then
          bestAngle = diff
        end
      end
      targetRoll = bestAngle
      if (target.Position - I:GetConstructPosition()).magnitude < 100 then
        for s, id in pairs(armBlocks) do I:SetSpinBlockRotationAngle(id, -90) end
      else
        for s, id in pairs(armBlocks) do I:SetSpinBlockRotationAngle(id, 0) end
      end
    end
  end

  --PID
  local e = Vector3.Cross(desired, maneuverByVelocity and I:GetVelocityVector().normalized or I:GetConstructForwardVector())
  local correction = Control.processPID(pid, e, elapsedTime) + pid.kD * I:GetAngularVelocity()

  --steering
  local yawResponse = Vector3.Dot(correction, -I:GetConstructUpVector())
  local pitchResponse = Vector3.Dot(correction, I:GetConstructRightVector())
  local rollResponse = ((targetRoll and 0.05 * targetRoll) or 0) - 0.1 * I:GetLocalAngularVelocity().z

  I:RequestControl(2, 1, yawResponse)
  I:RequestControl(2, 4, pitchResponse)
  I:RequestControl(2, 2, rollResponse)
end

function CheckTarget(I, currentTargetId)
  if Vector3.Angle(I:GetTargetInfo(0, 0).Position - I:GetConstructPosition(), I:GetWeaponInfo(0).CurrentDirection) < 0.01 then
    return nil
  end
  local candidate = nil
  for warning=0, I:GetNumberOfWarnings() do
    local missile = I:GetMissileWarning(warning)
    if missile.Valid and currentTargetId and missile.Id == currentTargetId and missile.Position.y > minHeight and missile.Velocity.magnitude > 50 then
      return missile
    end
    if missile.Valid and Vector3.Angle(missile.Position - I:GetConstructPosition(), I:GetWeaponInfo(0).CurrentDirection) < 0.1 then
      candidate = missile
    end
  end
  return candidate
end

function FinalGuidance(I, target)
  local t = Differ.get(timeDiff)
  local a, wt = Accumulator.update(accelAcc, target.Velocity, t)
  local accel = a / wt
  local targetLead = maxTargetLead * Mathf.Min((target.Position - I:GetConstructPosition()).magnitude / 300, 1)
  if targetLead < 0.1 then targetLead = 0 end
  local adjustedPos = target.Position + target.Velocity * targetLead
  local adjustedVel = target.Velocity + accel * targetLead
  local relPos = adjustedPos - I:GetConstructPosition()
  local r = relPos.magnitude
  local v = I:GetVelocityVector()
  if r < altViolationZone then
    return Targeting.firstOrderTargeting(relPos, adjustedVel, v.magnitude) or PP(I, target)
  elseif r < maxRange then
    local w = tpgWeight
    if tpgWeight == -1 then
      w = altViolationZone / relPos.magnitude
    end
    local tpgDes = Targeting.firstOrderTargeting(relPos, adjustedVel, v.magnitude)
    if not tpgDes then
      tpgDes = PP(I, target)
    end
    return Adjust(I, tpgDes * w + (1 - w) * PP(I, target))
  else
    I:Log("No targets in range")
    return Adjust(I, Idle(I))
  end
end

function Adjust(I, desired)
  -- Adjust steering to avoid water, space, and land
  if I:GetConstructPosition().y + 100 * desired.y < altFloorWater then
    desired.y = Mathf.Max(desired.y, (-I:GetConstructPosition().y  + altFloorWater) / 40)
  end
  if I:GetConstructPosition().y + 100 * desired.y > altCeil then
    desired.y = Mathf.Min(desired.y, (-I:GetConstructPosition().y  + altCeil) / 40)
  end

  return desired.normalized
end

function Beamride(I, target)
  local relVel = target.Velocity - I:GetVelocityVector()
  local relPos = target.Position - I:GetConstructPosition()
  local closingVec = -Vector3.Project(relVel, relPos) --negative because it's the missile's velocity
  local closingRate = Vector3.Dot(closingVec, relPos.normalized)
  local timeUntilHit = relPos.magnitude / closingRate
  if timeUntilHit < 0 then return PP(I, target) end
  local projection = Vector3.Project(relPos, target.Velocity)
  local correction = (relPos - projection) / timeUntilHit
  return projection.normalized + correction.normalized * (0.005 * correction.magnitude - 0.03 * Vector3.Dot(I:GetVelocityVector(), correction.normalized))
end

function PP(I, target)
  -- point straight at the target
  local relPos = target.Position - I:GetConstructPosition()

  return relPos.normalized
end

function Idle(I)
  -- flies in a circle at idleAlt when no target spotted
  local fwv = I:GetForwardsVelocityMagnitude()
  local thrust = 0.6 * I:GetDrive(0) * 3264 / Mathf.Pow(fwv, fwv*fwv/350/350)

  circlePoint = I.Fleet.Flagship.CenterOfMass

  local radius = I:GetConstructPosition() - circlePoint
  local centripetal = Mathf.Asin(Mathf.Clamp(3.9 * fwv * fwv / idleRadius / thrust, -1, 1))
  radius.y = 0
  local correction = 1 * Mathf.Max(radius.magnitude - idleRadius, 0) + 2.5 * Vector3.Dot(I:GetVelocityVector(), radius.normalized)
  local prediction = fwv * Differ.get(timeDiff) / idleRadius * 180 / Mathf.PI
  local desired = Quaternion.AngleAxis(Mathf.Min(correction + prediction + centripetal, 90), Vector3(0, 1, 0)) * Vector3.Cross(Vector3(0, 1, 0), radius.normalized)
  desired.y = 0.02 * (idleAlt - I:GetConstructPosition().y) - 0.05 * (I:GetVelocityVector().y)
  desired = desired.normalized

  -- missile has no lifting wings, so pitch up to account for gravity
  local pitchRad = I:GetConstructPitch() * Mathf.PI/180
  local gravForce = 3.9 * -I:GetGravityForAltitude(I:GetConstructPosition().y).y * Mathf.Cos(pitchRad)
  local pitchGravCorrection = 180/Mathf.PI * Mathf.Asin(Mathf.Clamp(gravForce / thrust, -1, 1))
  desired = Quaternion.AngleAxis(pitchGravCorrection, -Vector3.Cross(Vector3(0, 1, 0), I:GetConstructForwardVector())) * desired

  return desired.normalized
end

function ResetTargetInfo()
  accelAcc = Accumulator.Accumulator(accelAvgTime, accelAvgDecay)
  currentTargetId = nil
end

function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function LinkedList.LinkedList()local k={}k.value=nil;k.next=k;k.prev=k;return k end;function LinkedList.pushFront(l,d)local k={}k.value=d;LinkedList.connect(k,l.next)LinkedList.connect(l,k)end;function LinkedList.pushBack(l,d)local k={}k.value=d;LinkedList.connect(l.prev,k)LinkedList.connect(k,l)end;function LinkedList.popFront(l)local m=l.next.value;LinkedList.connect(l,l.next.next)return m end;function LinkedList.popBack(l)local m=l.prev.value;LinkedList.connect(l.prev.prev,l)return m end;function LinkedList.peekFront(l)return l.next.val end;function LinkedList.peekBack(l)return l.prev.val end;function LinkedList.connect(n,o)n.next=o;o.prev=n end;function LinkedList.toArray(l)local p=1;local q={}local k=l.next;while k~=l do q[p]=k.value;k=k.next end;return q end;function MathUtil.angleOnPlane(r,s,t)local u=Vector3.ProjectOnPlane(r,t)local g=Vector3.ProjectOnPlane(s,t)return Vector3.SignedAngle(u,g,t)end;function MathUtil.min(v,w)local x=nil;w=w or function(y,z)return y<z end;for ele in v do if not x or w(ele,x)then x=ele end end;return x end;function MathUtil.max(v,w)local A=nil;w=w or function(y,z)return y<z end;for ele in v do if not A or w(A,ele)then A=ele end end;return A end;function MathUtil.range(y,z,B)local C,D=y,z;local E;if not y then return end;if not z then C=0;D=y;E=C<D and 1 or-1 elseif B then E=B end;return function(F,G)local H=G+E;if H==D then return nil end;return H end,nil,C-E end;function MathUtil.angleSSS(y,z,B)if y+z<B or y+B<z or z+B<y then return nil end;local I=Mathf.Acos((z*z+B*B-y*y)/(2*z*B))*Mathf.Rad2Deg;local J,K=MathUtil.angleSAS(z,I,B)return I,J,K end;function MathUtil.sideSAS(y,K,z)local L=y*y+z*z-2*y*z*Mathf.Cos(K*Mathf.Deg2Rad)return Mathf.Sqrt(L)end;function MathUtil.angleSAS(y,K,z)local B=MathUtil.sideSAS(y,K,z)if MathUtil.isZero(B)then return nil end;local I,J;if y<z then I=MathUtil.angleLoSin(B,y,K)J=180-I-K else J=MathUtil.angleLoSin(B,z,K)I=180-J-K end;return I,J end;function MathUtil.sideSSA(y,z,I)local M=z*z-y*y;local N=-2*z*math.cos(math.rad(I))local O,P=MathUtil.solveQuadratic(1,N,M)if not P then return O,P end;if O<P then return O,P end;return P,O end;function MathUtil.angleSSA(y,z,I)local O,P=MathUtil.sideSSA(y,z,I)if not O then return nil end;local Q,R=MathUtil.angleSAS(z,I,O)if not P then return Q,R end;local S,T=MathUtil.angleSAS(z,I,P)return Q,R,S,T end;function MathUtil.sideAAS(I,J,y)local K=180-I-J;local z=MathUtil.sideLoSin(I,J,y)local B=MathUtil.sideLoSin(I,K,y)return z,B end;function MathUtil.sideLoSin(y,I,J)return y*Mathf.Sin(J*Mathf.Deg2Rad)/Mathf.Sin(I*Mathf.Deg2Rad)end;function MathUtil.angleLoSin(y,z,I)return Mathf.Asin(z*Mathf.Sin(I*Mathf.Deg2Rad)/y)*Mathf.Rad2Deg end;function MathUtil.clampCone(U,V,W)local X=Mathf.Min(W,Vector3.Angle(U,V))local Y=Vector3.Cross(U,V)return Quaternion.AngleAxis(X,Y)*U end;local Z=1e-9;function MathUtil.isZero(h)return h>-Z and h<Z end;function MathUtil.cuberoot(_)return _>0 and _^(1/3)or-math.abs(_)^(1/3)end;function MathUtil.solveQuadratic(a0,O,P)local a1,a2;local a3,a4,a5;a3=O/(2*a0)a4=P/a0;a5=a3*a3-a4;if MathUtil.isZero(a5)then a1=-a3;return a1 elseif a5<0 then return else local a6=math.sqrt(a5)a1=a6-a3;a2=-a6-a3;return a1,a2 end end;function MathUtil.solveCubic(a0,O,P,a7)local a1,a2,a8;local a9,aa;local I,J,K;local ab,a3,a4;local ac,a5;I=O/a0;J=P/a0;K=a7/a0;ab=I*I;a3=1/3*(-(1/3)*ab+J)a4=0.5*(2/27*I*ab-1/3*I*J+K)ac=a3*a3*a3;a5=a4*a4+ac;if MathUtil.isZero(a5)then if MathUtil.isZero(a4)then a1=0;a9=1 else local ad=MathUtil.cuberoot(-a4)a1=2*ad;a2=-ad;a9=2 end elseif a5<0 then local ae=1/3*math.acos(-a4/math.sqrt(-ac))local g=2*math.sqrt(-a3)a1=g*math.cos(ae)a2=-g*math.cos(ae+math.pi/3)a8=-g*math.cos(ae-math.pi/3)a9=3 else local a6=math.sqrt(a5)local ad=MathUtil.cuberoot(a6-a4)local af=-MathUtil.cuberoot(a6+a4)a1=ad+af;a9=1 end;aa=1/3*I;if a9>0 then a1=a1-aa end;if a9>1 then a2=a2-aa end;if a9>2 then a8=a8-aa end;return a1,a2,a8 end;function MathUtil.solveQuartic(a0,O,P,a7,ag)local a1,a2,a8,ah;local ai={}local aj,ad,af,aa;local I,J,K,a5;local ab,a3,a4,ak;local a9;I=O/a0;J=P/a0;K=a7/a0;a5=ag/a0;ab=I*I;a3=-0.375*ab+J;a4=0.125*ab*I-0.5*I*J+K;ak=-(3/256)*ab*ab+0.0625*ab*J-0.25*I*K+a5;if MathUtil.isZero(ak)then ai[3]=a4;ai[2]=a3;ai[1]=0;ai[0]=1;local al={MathUtil.solveCubic(ai[0],ai[1],ai[2],ai[3])}a9=#al;a1,a2,a8=al[1],al[2],al[3]else ai[3]=0.5*ak*a3-0.125*a4*a4;ai[2]=-ak;ai[1]=-0.5*a3;ai[0]=1;a1,a2,a8=MathUtil.solveCubic(ai[0],ai[1],ai[2],ai[3])aj=a1;ad=aj*aj-ak;af=2*aj-a3;if MathUtil.isZero(ad)then ad=0 elseif ad>0 then ad=math.sqrt(ad)else return end;if MathUtil.isZero(af)then af=0 elseif af>0 then af=math.sqrt(af)else return end;ai[2]=aj-ad;ai[1]=a4<0 and-af or af;ai[0]=1;do local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=#al;a1,a2=al[1],al[2]end;ai[2]=aj+ad;ai[1]=a4<0 and af or-af;ai[0]=1;if a9==0 then local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=a9+#al;a1,a2=al[1],al[2]end;if a9==1 then local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=a9+#al;a2,a8=al[1],al[2]end;if a9==2 then local al={MathUtil.solveQuadratic(ai[0],ai[1],ai[2])}a9=a9+#al;a8,ah=al[1],al[2]end end;aa=0.25*I;if a9>0 then a1=a1-aa end;if a9>1 then a2=a2-aa end;if a9>2 then a8=a8-aa end;if a9>3 then ah=ah-aa end;return a1,a2,a8,ah end;function RingBuffer.RingBuffer(am)local an={}an.buf={}an.capacity=am;an.tail=1;an.head=1;return an end;function RingBuffer.isFull(an)return(an.head-an.tail)%an.capacity==1 end;function RingBuffer.isEmpty(an)return an.head==an.tail end;function RingBuffer.push(an,d)an.buf[an.tail]=d;if RingBuffer.isFull(an)then an.head=an.head%an.capacity+1 end;an.tail=an.tail%an.capacity+1 end;function RingBuffer.pop(an)if RingBuffer.isEmpty(an)then return nil end;local m=an.buf[an.head]an.buf[an.head]=nil;an.head=an.head%an.capacity+1;return m end;function Control.PID(ao,ap,aq,ar,as,at)local au={}au.kP=ao;au.kI=ap;au.kD=aq;au.Iacc=Accumulator.Accumulator(ar,as)if at and at~=0 then au.period=at end;return au end;function Control.processPID(av,aw,e)aw=av.period and(aw+av.period/2)%av.period-av.period/2 or aw;local a3=av.kP*aw;local p=av.kI*Accumulator.update(av.Iacc,aw,e)local h=av.kD*(aw-(av.lastError or aw))/e;av.lastError=aw;return a3+p+h end;function Control.FF(ai,at)local ax={}ax.coeffs=ai;ax.degree=#ai-1;if at and at~=0 then ax.period=at end;ax.derivs={}return ax end;function Control.processFF(av,ay,e)local az=0*ay;local aA=ay;local aB=ay;for p=1,av.degree+1 do aB=av.derivs[p]av.derivs[p]=aA;az=az+av.coeffs[p]*aA;if aB then local aC=aA-aB;if p==1 and av.period then aC=(aC+av.period/2)%av.period-av.period/2 end;aA=aC/e else break end end;return az end;function Nav.toLocal(aD,aE,aF)local aG=aD-aE;return Quaternion.Inverse(aF)*aG end;function Nav.toGlobal(aH,aE,aF)local aG=aF*aH;return aG+aE end;function Nav.cartToPol(aI)local ak=aI.magnitude;local aJ=Vector3.SignedAngle(Vector3.forward,aI,Vector3.up)local ae=90-Vector3.Angle(Vector3.up,aI)return Vector3(ak,aJ,ae)end;function Nav.cartToCyl(aI)local aK=Vector3(aI.x,0,aI.z)local aL=aK.magnitude;local ae=Vector3.SignedAngle(Vector3.forward,aI,Vector3.up)local aj=aI.y;return Vector3(aL,ae,aj)end;function Nav.polToCart(aI)local ak,aJ,ae=aI.x,aI.y,aI.z;local _=Mathf.Sin(aJ)*Mathf.Cos(ae)local aM=Mathf.Sin(ae)local aj=Mathf.Cos(aJ)*Mathf.Cos(ae)return ak*Vector3(_,aM,aj)end;function Nav.cylToCart(aI)local aL,ae,aN=aI.x,aI.y,aI.z;local _=aL*Mathf.Sin(ae)local aM=aN;local aj=aL*Mathf.Cos(ae)return Vector3(_,aM,aj)end;function Targeting.firstOrderTargeting(aO,aP,aQ)local aR=Vector3.Angle(-aO,aP)local aS,O,aT,P=MathUtil.angleSSA(aQ,aP.magnitude,aR)if not aS then return nil end;local aU=aT or aS;if not aU then return nil end;return(Quaternion.AngleAxis(aU,Vector3.Cross(aO,aP))*aO).normalized end;function Targeting.secondOrderTargeting(aO,aV,aW,aQ,aX,aY)local g=Targeting.secondOrderTargetingTime(aO,aV,aW,aQ,aX/aQ,aY/aQ)if g and g>0 then return(aO/g+aV-0.5*aW*g).normalized end;return nil end;function Targeting.secondOrderTargetingTime(aO,aV,aW,aQ,aZ,a_)local y=0.25*aW.sqrMagnitude;local z=Vector3.Dot(aV,aW)local B=aV.sqrMagnitude-aQ*aQ+Vector3.Dot(aO,aW)local h=2*Vector3.Dot(aO,aV)local aw=aO.sqrMagnitude;local b0={MathUtil.solveQuartic(y,z,B,h,aw)}local g=nil;for p=1,4 do if b0[p]and b0[p]>aZ and b0[p]<a_ then if not g or g and b0[p]<g then g=b0[p]end end end;return g end;function Targeting.AIPPN(b1,aO,b2,aP,b3)local aV=aP-b2;local b4=Vector3.Dot(-aV,aO.normalized)if b4<=0 then b4=10 end;local b5=aO.magnitude/b4;local b6=Vector3.Cross(aO,aV)/aO.sqrMagnitude;local b7=Vector3.Cross(aO,b3)/aO.sqrMagnitude*b5/2;local b8=b6+b7;local b9=Vector3.Cross(b8,aO.normalized)local ba=Vector3.ProjectOnPlane(b9,b2).normalized;local bb=b1*b2.magnitude*b8.magnitude;return bb*ba end;function Targeting.ATPN(b1,aO,b2,aP,b3)local aV=aP-b2;local b4=-Vector3.Dot(aV,aO.normalized)if b4<=0 then b4=10 end;local b6=Vector3.Cross(aO,aV)/aO.sqrMagnitude;local b9=Vector3.Cross(b6,aO.normalized)local bc=Vector3.ProjectOnPlane(b3,aO)return b1*b4*b9+0.5*b1*b3 end;function Targeting.accelToDirection(bd,be,e)local bf=Vector3.Cross(bd,be)/bd.sqrMagnitude*e*Mathf.Rad2Deg;return Quaternion.AngleAxis(bf.magnitude,bf)*bd end;function BlockUtil.getWeaponsByName(bg,bh,bi,bj)if DEBUG then bg:Log("searching for "..bh)end;local bk=bg:GetAllSubConstructs()local bl={}bi=bi or-1;local B=bi;if not bj or bj==0 or bj==2 then for p=0,bg:GetWeaponCount()-1 do if B==0 then break end;if bg:GetWeaponBlockInfo(p).CustomName==bh then table.insert(bl,{subIdx=nil,wpnIdx=p})if DEBUG then bg:Log("found weapon "..bh.." on hull, type "..bg:GetWeaponInfo(p).WeaponType)end;B=B-1 end end end;if not bj or bj==1 or bj==2 then for bm=1,#bk do local aa=bk[bm]for p=0,bg:GetWeaponCountOnSubConstruct(aa)-1 do if B==0 then break end;if bg:GetWeaponBlockInfoOnSubConstruct(aa,p).CustomName==bh then table.insert(bl,{subIdx=aa,wpnIdx=p})if DEBUG then bg:Log("found weapon "..bh.." on subobj "..aa..", type "..bg:GetWeaponInfo(p).WeaponType)end;B=B-1 end end end end;if DEBUG then bg:Log("weapon count: "..#bl)end;return bl end;function BlockUtil.getSubConstructsByName(bg,bh,bi)if DEBUG then bg:Log("searching for "..bh)end;local bk=bg:GetAllSubConstructs()local bn={}bi=bi or-1;local B=bi;for bm=1,#bk do local aa=bk[bm]if B==0 then break end;if bg:GetSubConstructInfo(aa).CustomName==bh then table.insert(bn,aa)if DEBUG then bg:Log("found subobj "..bh)end;B=B-1 end end;if DEBUG then bg:Log("subobj count: "..#bn)end;return bn end;function BlockUtil.getBlocksByName(bg,bh,bo,bi)if DEBUG then bg:Log("searching for "..bh)end;local bp={}bi=bi or-1;local B=bi;for bm=0,bg:Component_GetCount(bo)-1 do if B==0 then break end;if bg:Component_GetBlockInfo(bo,bm).CustomName==bh then table.insert(bp,bm)if DEBUG then bg:Log("found component "..bh)end;B=B-1 end end;if DEBUG then bg:Log("component count: "..#bp)end;return bp end;function BlockUtil.getWeaponInfo(bg,bq)local br;if bq.subIdx then br=bg:GetWeaponInfoOnSubConstruct(bq.subIdx,bq.wpnIdx)else br=bg:GetWeaponInfo(bq.wpnIdx)end;return br end;function BlockUtil.aimWeapon(bg,bq,bs,bt)if bq.subIdx then bg:AimWeaponInDirectionOnSubConstruct(bq.subIdx,bq.wpnIdx,bs.x,bs.y,bs.z,bt)else bg:AimWeaponInDirection(bq.wpnIdx,bs.x,bs.y,bs.z,bt)end end;function BlockUtil.fireWeapon(bg,bq,bt)if bq.subIdx then bg:FireWeaponOnSubConstruct(bq.subIdx,bq.wpnIdx,bt)else bg:FireWeapon(bq.wpnIdx,bt)end end;function Combat.pickTarget(bg,bu,bv)bv=bv or function(F,bw)return bw.Priority end;local ay,bx;for p in MathUtil.range(bg:GetNumberOfTargets(bu))do local bw=bg:GetTargetInfo(bu,p)local by=bv(bg,bw)if not ay or by>bx then ay=bw;bx=by end end;return ay end;function CheckConstraints(bg,bz,bA,bB)local bC;if bB then bC=bg:GetWeaponConstraintsOnSubConstruct(bB,bA)else bC=bg:GetWeaponConstraints(bA)end;local bD=bg:GetConstructForwardVEctor()local bE=bg:GetConstructUpVector()local bF=Quaternion.LookRotation(bD,bE)bz=Quaternion.Inverse(bF)*bz;if bC.InParentConstructSpace and bB then local bG=bg:GetSubConstructInfo(bB).localRotation;bz=Quaternion.inverse(bG)*bz end;local bH=MathUtil.angleOnPlane(Vector3.forward,bz,Vector3.up)local bI=bz;bI.z=0;local bJ=Mathf.Atan2(bz.z,bI.magnitude)local bK=bH>bC.MinAzimuth and bH<bC.MaxAzimuth;local bL=bJ>bC.MinElevation and bJ<bC.MaxElevation;if bC.FlipAzimuth then bK=not bK end;if bK and bL then return true end;bH=bH+180;ele=180-ele;if ele>180 then ele=ele-360 end;if ele<-180 then ele=ele+360 end;bK=bH>bC.MinAzimuth and bH<bC.MaxAzimuth;bL=bJ>bC.MinElevation and bJ<bC.MaxElevation;if bC.FlipAzimuth then bK=not bK end;if bK and bL then return true end;return false end
