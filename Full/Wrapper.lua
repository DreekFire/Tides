-- Each redundant Lua box should have a unique, positive luaIdx (i.e. 1, 2, 3, ...). They will come online if all lower index Lua boxes are destroyed.
local luaIdx = 1

-- Tides header
local Accumulator = {}
local BlockUtil = {}
local Combat = {}
local Control = {}
local Differ = {}
local Graph = {}
local Heapq = {}
local LinkedList = {}
local MathUtil = {}
local Matrix3 = {}
local Nav = {}
local RingBuffer = {}
local Scheduling = {}
local Search = {}
local Stats = {}
local StringUtil = {}
local Targeting = {}
local VectorN = {}

local inited = false

function Init(I)
    -- Init code here
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
    -- Your code here

end

function Accumulator.Accumulator(a,b)local c={}c.decay=b;c.window=a;c.time=0;c.weight=0;if a>0 then c.vals=LinkedList.LinkedList()c.times=LinkedList.LinkedList()end;return c end;function Accumulator.update(c,d,e)local f=Mathf.Pow(c.decay,e)if not c.value then c.value=d*e else c.value=c.value*f;c.value=c.value+d*e end;c.time=c.time+e;c.weight=c.weight*f;c.weight=c.weight+e;if c.window>0 then LinkedList.pushFront(c.vals,d)LinkedList.pushFront(c.times,e)while c.time>c.window do local g=LinkedList.popBack(c.times)c.time=c.time-g;local h=Mathf.Pow(c.decay,c.time)c.weight=c.weight-g*h;c.value=c.value-LinkedList.popBack(c.vals)*g*h end end;return c.value,c.weight end;function Accumulator.get(c)return c.value,c.weight end;function Differ.Differ(i)local j={}j.lastVal=i;j.diff=nil;return j end;function Differ.update(j,d)if j.lastVal then j.diff=d-j.lastVal;j.lastVal=d end;j.lastVal=d;return j.diff end;function Differ.get(j)return j.diff end;function Heapq.Heapq(i,k)local l={}l.data=i;l.comp=k or function(m,n)return m<n end;local o=#l.data;l.size=o;for p=math.floor(o/2),1,-1 do Heapq.siftDown(l,p)end;return l end;function Heapq.siftDown(l,q)local r=false;local s=q;local o=#l.data;while not r do r=true;local t=2*s;local u=2*s+1;local v=s;if t<=o and l.comp(l.data[t],l.data[v])then v=t;r=false end;if u<=o and l.comp(l.data[u],l.data[v])then v=u;r=false end;if not r then local w=l.data[v]l.data[v]=l.data[s]l.data[s]=w;s=v end end end;function Heapq.siftUp(l,q)local r=false;local s=q;while not r do r=true;local x=math.floor(s/2)if l.comp(l.data[s],l.data[x])then local w=l.data[x]l.data[x]=l.data[s]l.data[s]=w;s=x;r=false end end end;function Heapq.insert(l,y)l.data[l.size+1]=y;l.size=l.size+1;Heapq.siftUp(l,l.size)end;function Heapq.pop(l)local z=l.data[1]l.data[1]=l.data[l.size]l.data[l.size]=nil;l.size=l.size-1;Heapq.siftDown(l,1)return z end;function Heapq.peek(l)return l.data[1]end;function Heapq.size(l)return l.size end;function LinkedList.LinkedList()local A={}A.value=nil;A.next=A;A.prev=A;return A end;function LinkedList.pushFront(B,d)local A={}A.value=d;LinkedList.connect(A,B.next)LinkedList.connect(B,A)end;function LinkedList.pushBack(B,d)local A={}A.value=d;LinkedList.connect(B.prev,A)LinkedList.connect(A,B)end;function LinkedList.popFront(B)local C=B.next.value;LinkedList.connect(B,B.next.next)return C end;function LinkedList.popBack(B)local C=B.prev.value;LinkedList.connect(B.prev.prev,B)return C end;function LinkedList.peekFront(B)return B.next.val end;function LinkedList.peekBack(B)return B.prev.val end;function LinkedList.connect(D,E)D.next=E;E.prev=D end;function LinkedList.toArray(B)local F=1;local G={}local A=B.next;while A~=B do G[F]=A.value;A=A.next end;return G end;function MathUtil.angleOnPlane(H,I,J)local K=Vector3.ProjectOnPlane(H,J)local g=Vector3.ProjectOnPlane(I,J)return Vector3.SignedAngle(K,g,J)end;function MathUtil.min(L,M)local N=nil;M=M or function(m,n)return m<n end;for O in L do if not N or M(O,N)then N=O end end;return N end;function MathUtil.max(L,M)local P=nil;M=M or function(m,n)return m<n end;for O in L do if not P or M(P,O)then P=O end end;return P end;function MathUtil.range(m,n,Q)local R,S=m,n;local T;if not m then return end;if not n then R=0;S=m;T=R<S and 1 or-1 elseif Q then T=Q end;return function(U,V)local W=V+T;if W==S then return nil end;return W end,nil,R-T end;function MathUtil.shuffle(B,X)local s=X and B or{}if not X then for F=1,#B do s[F]=B[F]end end;for F=#B,2,-1 do local Y=math.random(F)s[F],s[Y]=s[Y],s[F]end;return s end;function MathUtil.combine(m,n,Z)if#m==#n then local z={}for _,a0 in pairs(m)do z[_]=Z(_,a0,n[_])end;return z end end;function MathUtil.angleSSS(m,n,Q)if m+n<Q or m+Q<n or n+Q<m then return nil end;local a1=math.deg(math.acos((n*n+Q*Q-m*m)/(2*n*Q)))local a2,a3=MathUtil.angleSAS(n,a1,Q)return a1,a2,a3 end;function MathUtil.sideSAS(m,a3,n)local a4=m*m+n*n-2*m*n*math.cos(math.rad(a3))return math.sqrt(a4)end;function MathUtil.angleSAS(m,a3,n)local Q=MathUtil.sideSAS(m,a3,n)if MathUtil.isZero(Q)then return nil end;local a1,a2;if m<n then a1=MathUtil.angleLoSin(Q,m,a3)a2=180-a1-a3 else a2=MathUtil.angleLoSin(Q,n,a3)a1=180-a2-a3 end;return a1,a2 end;function MathUtil.sideSSA(m,n,a1)local a5=n*n-m*m;local a6=-2*n*math.cos(math.rad(a1))local a7,a8=MathUtil.solveQuadratic(1,a6,a5)if not a8 then return a7,a8 end;if a7<a8 then return a7,a8 end;return a8,a7 end;function MathUtil.angleSSA(m,n,a1)local a7,a8=MathUtil.sideSSA(m,n,a1)if not a7 then return nil end;local a9,aa=MathUtil.angleSAS(n,a1,a7)if not a8 then return a9,aa end;local ab,ac=MathUtil.angleSAS(n,a1,a8)return a9,aa,ab,ac end;function MathUtil.sideAAS(a1,a2,m)local a3=180-a1-a2;local n=MathUtil.sideLoSin(a1,a2,m)local Q=MathUtil.sideLoSin(a1,a3,m)return n,Q end;function MathUtil.sideLoSin(m,a1,a2)return m*math.sin(math.rad(a2))/math.sin(math.rad(a1))end;function MathUtil.angleLoSin(m,n,a1)return math.deg(math.asin(n*math.sin(math.rad(a1))/m))end;function MathUtil.clampCone(ad,ae,af)local ag=math.min(af,Vector3.Angle(ad,ae))local ah=Vector3.Cross(ad,ae)return Quaternion.AngleAxis(ag,ah)*ad end;function MathUtil.newton(ai,aj,ak,al,am,an)al=al or 1e-5;an=an or 10*al;am=am or 25;aj=aj or function(ao)return(ai(ao+an)-ai(ao))/an end;ak=ak or 0;local ap=al+1;local aq=0;while ap>al and aq<am do local ar=ai(ak)local as=aj(ak)if not ar or not as then return nil end;ap=-ar/as;ak=ak+ap;aq=aq+1 end;if aq<am then return ak,false end;return ak,true end;function MathUtil.ITP(ai,m,n,al,am)if ai(m)*ai(n)>0 then return nil end;if ai(m)>ai(n)then ai=function(ao)return-ai(ao)end end;al=al or 1e-5;am=am or 25;local at=0.2/(n-m)local au=2;local av=1;local aw=math.ceil(math.log((n-m)/(2*al),2))local ax=aw+av;local aq=am;for Y=1,am do local ay=ai(m)local az=ai(n)local aA=ay-az;if aA==0 then return m end;local aB=0.5*(m+n)local aC=(n*ay+m*az)/aA;if aB<m or aB>n then aB=0.5*(m+n)end;local aD=aB-aC;local aE=at*math.abs(n-m)^au;local aF=aD>0 and 1 or(aD==0 and 0 or-1)local aG=aE<=math.abs(aD)and aC+aF*aE or aB;local aH=al*2^(ax-Y)-0.5*(n-m)local aI=math.abs(aG-aB)<=aH and aG or aB-aF*aH;local aJ=ai(aI)if aJ>0 then n=aI elseif aJ<0 then m=aI else return aI,Y==am end;if n-m<2*al then aq=Y;break end end;local ay=ai(m)local az=ai(n)local aA=az-ay;if aA~=0 then return(m*az-n*ay)/aA,aq==am end;return m,aq==am end;function MathUtil.binomCoeffs(aK,aL)if aL then local aM={}else local aM={}aM[1]=1;for _=1,aK do aM[_+1]=aM[_]*(aK-_)/(_+1)end;return aM end end;function MathUtil.ruleOfSigns(aM,aN)local aO={}local aP=#aM;for F=1,aP do aO[F]=aM[aP-F+1]end;if aN~=0 then local aQ={}for F=1,aP do aQ[F]=(F-1)*aM[aP-F+1]end;local aR=1;for F=2,aP do local aS=aN^(F-1)for Y=1,aP-F+1 do local aT=F+Y-1;aO[Y]=aO[Y]+aR*aQ[aT]*aS;aQ[aT]=aQ[aT]*(Y-1)end;aR=aR/F end end;local aU={}local o=1;for F,aV in ipairs(aO)do if aV~=0 then aU[o]=aV;o=o+1 end end;local aW=0;for F=1,#aU-1 do if aU[F]*aU[F+1]<0 then aW=aW+1 end end;return aW end;function MathUtil.cache(ai)local Q={}local aX=getmetatable(Q)or{}function aX.__index(aY,ao)local C=ai(ao)aY[ao]=C;return C end;setmetatable(Q,aX)return function(m)return Q[m]end end;function MathUtil.lerp(ai,R,S,T,aZ)local a_={}for F=1,math.floor((S-R)/T)+1 do a_[F]=ai(R+F*T)end;a_.start=R;a_.stop=S;a_.step=T;a_.lval=aZ and a_[1]or nil;a_.rval=aZ and a_[#a_]or nil;return function(ao)if ao>=a_.stop then return a_.rval end;if ao<=a_.start then return a_.lval end;local F=(ao-a_.start)/a_.step;local b0=F%1;F=math.floor(F)return(1-b0)*a_[F]+b0*a_[F+1]end end;function MathUtil._factorial(o)if o<2 then return 1 end;return MathUtil._factorial(o-1)end;MathUtil.factorial=MathUtil.cache(MathUtil._factorial)MathUtil.eps=1e-9;function MathUtil.isZero(h)return h>-MathUtil.eps and h<MathUtil.eps end;function MathUtil.setTolerance(al)MathUtil.eps=al end;function MathUtil.cuberoot(ao)return ao>0 and ao^(1/3)or-math.abs(ao)^(1/3)end;function MathUtil.solveQuadratic(b1,a7,a8)local b2,b3;local b4,b5,b6;b4=a7/(2*b1)b5=a8/b1;b6=b4*b4-b5;if MathUtil.isZero(b6)then b2=-b4;return b2 elseif b6<0 then return else local b7=math.sqrt(b6)b2=b7-b4;b3=-b7-b4;return b2,b3 end end;function MathUtil.solveCubic(b1,a7,a8,b8)local b2,b3,b9;local ba,bb;local a1,a2,a3;local bc,b4,b5;local bd,b6;a1=a7/b1;a2=a8/b1;a3=b8/b1;bc=a1*a1;b4=1/3*(-(1/3)*bc+a2)b5=0.5*(2/27*a1*bc-1/3*a1*a2+a3)bd=b4*b4*b4;b6=b5*b5+bd;if MathUtil.isZero(b6)then if MathUtil.isZero(b5)then b2=0;ba=1 else local be=MathUtil.cuberoot(-b5)b2=2*be;b3=-be;ba=2 end elseif b6<0 then local bf=1/3*math.acos(-b5/math.sqrt(-bd))local g=2*math.sqrt(-b4)b2=g*math.cos(bf)b3=-g*math.cos(bf+math.pi/3)b9=-g*math.cos(bf-math.pi/3)ba=3 else local b7=math.sqrt(b6)local be=MathUtil.cuberoot(b7-b5)local a0=-MathUtil.cuberoot(b7+b5)b2=be+a0;ba=1 end;bb=1/3*a1;if ba>0 then b2=b2-bb end;if ba>1 then b3=b3-bb end;if ba>2 then b9=b9-bb end;return b2,b3,b9 end;function MathUtil.solveQuartic(b1,a7,a8,b8,bg)local b2,b3,b9,bh;local aM={}local bi,be,a0,bb;local a1,a2,a3,b6;local bc,b4,b5,bj;local ba=0;a1=a7/b1;a2=a8/b1;a3=b8/b1;b6=bg/b1;bc=a1*a1;b4=-0.375*bc+a2;b5=0.125*bc*a1-0.5*a1*a2+a3;bj=-(3/256)*bc*bc+0.0625*bc*a2-0.25*a1*a3+b6;if MathUtil.isZero(bj)then aM[3]=b5;aM[2]=b4;aM[1]=0;aM[0]=1;local bk={MathUtil.solveCubic(aM[0],aM[1],aM[2],aM[3])}ba=#bk;b2,b3,b9=bk[1],bk[2],bk[3]elseif MathUtil.isZero(b5)then local bl={MathUtil.solveQuadratic(1,b4,bj)}if bl[1]>=0 then b2=-math.sqrt(bl[1])b3=math.sqrt(bl[1])ba=2 end;if bl[2]>=0 then if ba==0 then b2=-math.sqrt(bl[2])b3=math.sqrt(bl[2])ba=2 else b9=-math.sqrt(bl[2])bh=math.sqrt(bl[2])ba=4 end end else aM[3]=0.5*bj*b4-0.125*b5*b5;aM[2]=-bj;aM[1]=-0.5*b4;aM[0]=1;b2,b3,b9=MathUtil.solveCubic(aM[0],aM[1],aM[2],aM[3])bi=b2;be=bi*bi-bj;a0=2*bi-b4;if MathUtil.isZero(be)then be=0 elseif be>0 then be=math.sqrt(be)else return end;if MathUtil.isZero(a0)then a0=0 elseif a0>0 then a0=math.sqrt(a0)else return end;aM[2]=bi-be;aM[1]=b5<0 and-a0 or a0;aM[0]=1;do local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=#bk;b2,b3=bk[1],bk[2]end;aM[2]=bi+be;aM[1]=b5<0 and a0 or-a0;aM[0]=1;if ba==0 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b2,b3=bk[1],bk[2]end;if ba==1 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b3,b9=bk[1],bk[2]end;if ba==2 then local bk={MathUtil.solveQuadratic(aM[0],aM[1],aM[2])}ba=ba+#bk;b9,bh=bk[1],bk[2]end end;bb=0.25*a1;if ba>0 then b2=b2-bb end;if ba>1 then b3=b3-bb end;if ba>2 then b9=b9-bb end;if ba>3 then bh=bh-bb end;return b2,b3,b9,bh end;function Matrix3.Matrix3(a_)local bm={}for F=1,9 do bm[F]=a_[F]end;setmetatable(bm,Matrix3.meta)return bm end;function Matrix3.get(bm,bn,bo)return bm[(bn-1)*3+bo]end;function Matrix3.set(bm,bn,bo,C)bm[(bn-1)*3+bo]=C end;function Matrix3.scalarmul(bm,s)local bp={}for F=1,9 do bp[F]=s*bm[F]end;return bp end;function Matrix3.vecmul(bm,bq)local bp=Vector3.zero;for bn=0,2 do local C=0;for bo=1,3 do C=C+bq[Matrix3.vecIdx[bo]]*bm[bn*3+bo]end;bp[Matrix3.vecIdx[bn+1]]=C end end;function Matrix3.matmul(br,bs)local bp={}for F=0,2 do for Y=0,2 do local C=0;for _=0,2 do C=C+br[F*3+_+1]*bs[_*3+Y+1]end;bp[F*3+Y+1]=C end end end;function Matrix3.mul(m,n)if getmetatable(m)~=Matrix3.meta then if m.x then return Matrix3.vecmul(Matrix3.transpose(n),m)end;return Matrix3.mul(n,m)end;if getmetatable(n)==Matrix3.meta then return Matrix3.matmul(m,n)end;if n.x then return Matrix3.vecmul(m,n)end;return Matrix3.scalarmul(m,n)end;function Matrix3.Identity()return{1,0,0,0,1,0,0,0,1}end;function Matrix3.Zero()return{0,0,0,0,0,0,0,0,0}end;function Matrix3.pow(bm,bt)local bu=bm;local bv=bm;while true do bt=math.floor(bt/2)if bt%2==1 then bu=Matrix3.matmul(bm,bv)end;if bt>=2 then bv=Matrix3.matmul(bv,bv)else break end end;return bu end;function Matrix3.add(br,bs)local bp={}for F=1,9 do bp[F]=br[F]+bs[F]end;return bp end;function Matrix3.hadamard(br,bs)local bp={}for F=1,9 do bp[F]=br[F]*bs[F]end;return bp end;function Matrix3.transpose(bm)local bw={}for bn=0,2 do for bo=0,2 do bw[bo*3+bn+1]=bm[bn*3+bo+1]end end;return bw end;function Matrix3.determinant(bm)local bx=0;local by=0;for h=0,2 do for s=0,2 do bx=bx+bm[s*3+(s+h)%3+1]by=by+bm[s*3+(-s+h)%3+1]end end;return bx-by end;function Matrix3.adjugate(bm)local bz={}for bn=0,2 do for bo=0,2 do local bA=0;for F=0,1 do local bB=1;for Y=1,2 do bB=bB*bm[(bn+Y)%3*3+(bo+F+Y)%3+1]end;bA=bA+bB end;bz[bo*3+bn+1]=bA end end;return bz end;function Matrix3.inverse(bm)local bC=Matrix3.determinant(bm)if Stats.isZero(bC)then return end;local bz=Matrix3.cofactors(bm)return bz/bC end;Matrix3.vecIdx={'x','y','z'}Matrix3.meta={__add=Matrix3.add,__mul=Matrix3.mul,__unm=function(bD)return Matrix3.scalarmul(bD,-1)end,__pow=Matrix3.pow}function RingBuffer.RingBuffer(bE)local bF={}bF.buf={}bF.capacity=bE;bF.size=0;bF.head=1;local aX=getmetatable(bF)or{}aX.__index=RingBuffer.get;setmetatable(bF,aX)return bF end;function RingBuffer.isFull(bF)return bF.size>=bF.capacity end;function RingBuffer.push(bF,d)bF.buf[(bF.head+bF.size-1)%bF.capacity+1]=d;if bF.size==bF.capacity then bF.head=bF.head%bF.capacity+1 else bF.size=bF.size+1 end end;function RingBuffer.pop(bF)if bF.size==0 then return nil end;local C=bF.buf[bF.head]bF.buf[bF.head]=nil;bF.head=bF.head%bF.capacity+1;bF.size=bF.size-1;return C end;function RingBuffer.get(bF,p)if type(p)~="number"or math.floor(p)~=p then return nil end;if p<1 or p>bF.size then return nil end;return bF.buf[(bF.head+p-2)%bF.capacity+1]end;function Search.interpolatedSearch(bG,t,u,bH,bI,bJ)bJ=bJ or 50;local m,n,bK;local bL=0;while u>t do m=bG[t]if m==bH then return t end;if m>bH then return bI and t or nil end;n=bG[u]if n==bH then return u end;if n<bH then return bI and u or nil end;bK=math.floor((bH-m)/(n-m)*(u-t)+t)bK=math.min(math.max(bK,t+1),u-1)if bG[bK]==bH then return bK end;if bH<bG[bK]then if bI and math.abs(bG[bK-1]-bH)>math.abs(bG[bK]-bH)then return bK end;u=bK-1 else if bI and math.abs(bG[bK+1]-bH)>math.abs(bG[bK]-bH)then return bK end;t=bK+1 end;bL=bL+1;if bL>bJ then break end end;return bI and t or nil end;function Stats.Distribution(bM)local bN={n=0,vars=bM}if bM then local bO={}local bP={}local bQ=#bM;for F,a0 in ipairs(bM)do bO[a0]=0;for Y=1,bQ do bP[(F-1)*bQ+Y]=0 end end;bN.mean=bO;bN.cov=bP else bN.mean=0;bN.cov=0 end;return bN end;function Stats.updateDistribution(bN,bR,bS)local bT=bN.n;bS=bS or 1;bN.n=bN.n+bS;if bN.vars then local bU={}local bQ=bN.vars and#bN.vars or 1;for F,a0 in ipairs(bN.vars)do bU[F]=bN.mean[a0]local bV=bU[F]+bR[a0]*bS/bN.n;for Y=F,bQ do local ae=bN.vars[Y]local bW=bN.mean[ae]local bX=(bS or 1)*(bR[a0]-bV)*(bR[ae]-bW)bN.cov[(F-1)*bQ+Y]=(bN.cov[(F-1)*bQ+Y]*bT+bX)/bN.n;bN.cov[(Y-1)*bQ+F]=bN.cov[(F-1)*bQ+Y]end;bN.mean[a0]=bV end else local bV=bN.mean+bR*bS/bN.n;bN.cov=(bN.cov*bT+bS*(bR-bV)*(bR-bN.mean))/bN.n;bN.mean=bV end;return bN end;function Stats.updateDistributionBatched(bN,bY,bZ)if#bY==0 then return end;local b_;local bQ=bN.vars and#bN.vars or 1;local c0=0;for Y=1,#bY do c0=c0+(bZ and bZ[Y]or 1)end;bN.n=bN.n+c0;local bT=bN.n;if bN.vars then for F,c1 in ipairs(bN.vars)do local bo={}for Y=1,#bY do bo[Y]=bY[Y][c1]end;b_[F]=bo end;for F,a0 in ipairs(bN.vars)do local c2=0;for Y,s in ipairs(b_[F])do c2=c2+s*(bZ and bZ[Y]or 1)end;local bV=bN.mean[a0]+c2/bN.n;for Y=F,bQ do local bW=bN.mean[bN.vars[Y]]c2=0;for s=1,#bY do c2=c2+(bZ and bZ[s]or 1)*(b_[F][s]-bV)*(b_[Y][s]-bW)end;bN.cov[(F-1)*bQ+Y]=(bN.cov[(F-1)*bQ+Y]*bT+c2)/bN.n;bN.cov[(Y-1)*bQ+F]=bN.cov[(F-1)*bQ+Y]end;bN.mean[a0]=bV end else local c2=0;for F,s in ipairs(bY)do c2=c2+s*(bZ and bZ[F]or 1)end;local bV=bN.mean+c2/bN.n;c2=0;for F,s in ipairs(bY)do c2=c2+(bZ and bZ[F]or 1)*(s-bV)*(s-bN.mean)end;bN.cov=(bN.cov*bT+c2)/bN.n end;return bN end;function Stats.mean(bN)return bN.mean end;function Stats.covariance(bN)return bN.cov end;function Stats.namedCovariance(bN,c3,c4)if not bN.vars then return bN.cov end;local bQ=bN.vars and#bN.vars or 1;for F=1,bQ do if bN.vars[F]==c3 then for Y=1,bQ do if bN.vars[Y]==c4 then return bN.cov[(F-1)*bQ+Y]end end end end end;function Stats.normal()local bi,c5=Stats.boxMuller()return bi end;function Stats.normalPDF(bi)return math.exp(-0.5*bi*bi)/math.sqrt(2*math.pi)end;function Stats.normalCDF(bi)local c6=0.2316419;local c7=0.319381530;local c8=-0.356563782;local c9=1.781477937;local ca=-1.821255978;local cb=1.330274429;local g=1/(1+c6*bi)return 1-Stats.normalPDF(bi)*(c7*g+c8*g^2+c9*g^3+ca*g^4+cb*g^5)end;function Stats.inverseNorm(b4)local cc=b4>=0.5 and b4 or-b4;local bi=5.55556*(1-((1-cc)/cc)^0.1186)if b4<0.5 then bi=-bi end;return bi end;function Stats.boxMuller()local cd=math.random()local ce=math.random()ce=math.random()ce=math.random()local bj=math.sqrt(-2*math.log(cd))local cf=2*math.pi*ce;return bj*math.cos(cf),bj*math.sin(cf)end;VectorN.mt=getmetatable({})or{}VectorN.mt.__add=function(m,n)local cg=type(m)=="number"local ch=type(n)=="number"if not cg and ch then return n+m end;if cg and not ch then return Stats.combine(m,n,function(_,ao,ci)return m+ci end)else return Stats.combine(m,n,function(_,ao,ci)return ao+ci end)end end;VectorN.mt.__sub=function(m,n)return m+-n end;VectorN.mt.__mul=function(m,n)local cg=type(m)=="number"local ch=type(n)=="number"if not cg and ch then return n*m end;if cg and not ch then local z={}for _,a0 in pairs(n)do z[_]=m*a0 end;return z else return Stats.combine(m,n,function(_,ao,ci)return ao*ci end)end end;VectorN.mt.__div=function(m,n)local cg=type(m)=="number"local ch=type(n)=="number"if not cg and ch then return m*1/n end;if cg and not ch then local z={}for _,a0 in pairs(n)do z[_]=m/a0 end;return z else return Stats.combine(m,n,function(_,ao,ci)return ao/ci end)end end;VectorN.mt.__unm=function(m)local z={}for _,a0 in pairs(m)do z[_]=-a0 end;return z end;function VectorN.VectorN(B)local bq={}for _,a0 in pairs(B)do if type(a0)=="table"then bq[_]=VectorN.VectorN(a0)else bq[_]=a0 end end;setmetatable(bq,VectorN.mt)return bq end;function Control.PID(cj,ck,cl,cm,cn,co)local cp={}cp.kP=cj;cp.kI=ck;cp.kD=cl;cp.Iacc=Accumulator.Accumulator(cm,cn)if co and co~=0 then cp.period=co end;return cp end;function Control.processPID(cq,cr,e)cr=cq.period and(cr+cq.period/2)%cq.period-cq.period/2 or cr;local b4=cq.kP*cr;local F,cs=Accumulator.update(cq.Iacc,cr,e)F=cq.kI*F/cs;local h=cq.kD*(cr-(cq.lastError or cr))/e;cq.lastError=cr;return b4+F+h end;function Control.FF(aM,co)local ct={}ct.coeffs=aM;ct.degree=#aM-1;if co and co~=0 then ct.period=co end;ct.derivs={}return ct end;function Control.processFF(cq,bH,e)local cu=0*bH;local cv=bH;local cw=bH;for F=1,cq.degree+1 do cw=cq.derivs[F]cq.derivs[F]=cv;cu=cu+cq.coeffs[F]*cv;if cw then local aD=cv-cw;if F==1 and cq.period then aD=(aD+cq.period/2)%cq.period-cq.period/2 end;cv=aD/e else break end end;return cu end;function Nav.toLocal(cx,cy,cz)local cA=cx-cy;return Quaternion.Inverse(cz)*cA end;function Nav.toGlobal(cB,cy,cz)local cA=cz*cB;return cA+cy end;function Nav.cartToPol(cC)local bj=cC.magnitude;local cf=Vector3.SignedAngle(Vector3.forward,cC,Vector3.up)local bf=90-Vector3.Angle(Vector3.up,cC)return Vector3(bj,cf,bf)end;function Nav.cartToCyl(cC)local cD=Vector3(cC.x,0,cC.z)local cE=cD.magnitude;local bf=Vector3.SignedAngle(Vector3.forward,cC,Vector3.up)local bi=cC.y;return Vector3(cE,bf,bi)end;function Nav.polToCart(cC)local bj,cf,bf=cC.x,cC.y,cC.z;local ao=Mathf.Sin(cf)*Mathf.Cos(bf)local ci=Mathf.Sin(bf)local bi=Mathf.Cos(cf)*Mathf.Cos(bf)return bj*Vector3(ao,ci,bi)end;function Nav.cylToCart(cC)local cE,bf,cF=cC.x,cC.y,cC.z;local ao=cE*Mathf.Sin(bf)local ci=cF;local bi=cE*Mathf.Cos(bf)return Vector3(ao,ci,bi)end;function Targeting.firstOrderTargeting(cG,cH,cI)if cH.sqrMagnitude==0 then return cG.normalized end;local cJ=cG-Vector3.Project(cG,cH)local cK=Vector3.Dot(cH,cG-cJ)/cH.sqrMagnitude;local m,n=MathUtil.solveQuadratic(cH.sqrMagnitude-cI*cI,2*cK*cH.sqrMagnitude,cJ.sqrMagnitude+cK*cK*cH.sqrMagnitude)local cL=nil;if m and m>=0 then cL=m end;if n and n>=0 and n<m then cL=n end;if cL then return cG+cL*cH,cL end end;function Targeting.secondOrderTargetingNewton(cG,cM,cN,cI,cO,cP,ak)local aD=10000;local cQ=0;local aq=0;if not ak then ak=0 end;local cR=cG+ak*cM+0.5*ak*ak*cN;while math.abs(aD)>0.001 and aq<10 do local g=cR.magnitude/cI;aD=g-cQ;cQ=g;aq=aq+1;cR=cG+g*cM+0.5*g*g*cN end;return cR,cQ,aq end;Targeting.secondOrderTargeting=Targeting.secondOrderTargetingNewton;function Targeting.secondOrderTargetingITP(cG,cM,cN,cI,cO,cP,ak)if not ak then ak=0 end;local m=-0.25*cN.sqrMagnitude;local n=-Vector3.Dot(cM,cN)local Q=-(cM.sqrMagnitude-cI*cI+Vector3.Dot(cG,cN))local h=-2*Vector3.Dot(cG,cM)local cr=-cG.sqrMagnitude;local g;local cS=cN.magnitude;local cT=cM.magnitude;local cU=cG.magnitude;local cV,cW=MathUtil.solveQuadratic(0.5*cS,cT+cI,-cU)local cX=math.max(cV,cW)local cY;local aM={0.5*cS,cT-cI,cU}if MathUtil.ruleOfSigns(aM,0)==2 then local cZ,c_=MathUtil.solveQuadratic(aM[1],aM[2],aM[3])if cZ then cY=math.min(cZ,c_)end end;if not cY or cY<cX then local b2,b3,b9=MathUtil.solveCubic(4*m,3*n,2*Q,h)if not b9 then if b2>cX then cY=b2 end else local cZ=math.min(b2,b9)local c_=math.max(b2,b9)if cZ>cX then cY=cZ elseif c_>cX then cY=c_ end end;if not cY then return end end;local function d0(ao)return cr+ao*(h+ao*(Q+ao*(n+ao*m)))end;if ak>cX and cY>ak then if d0(cX)*d0(g)<0 then cY=ak else cX=ak end end;g=MathUtil.ITP(d0,cX,cY,1e-4,25)if not g then return end;if g>=cX and g<=cY then local d1=cG+cM*g+0.5*cN*g*g;if d1.magnitude>=cO and d1.magnitude<=cP then return d1,g end end end;function Targeting.AIPPN(d2,cG,d3,cH,d4)local cM=cH-d3;local d5=Vector3.Dot(-cM,cG.normalized)if d5<=0 then d5=10 end;local d6=cG.magnitude/d5;local d7=Vector3.Cross(cG,cM)/cG.sqrMagnitude;local d8=Vector3.Cross(cG,d4)/cG.sqrMagnitude*d6/2;local d9=d7+d8;local da=Vector3.Cross(d9,cG.normalized)local db=Vector3.ProjectOnPlane(da,d3).normalized;local dc=d2*d3.magnitude*d9.magnitude;return dc*db end;function Targeting.ATPN(d2,cG,d3,cH,d4)local cM=cH-d3;local d5=-Vector3.Dot(cM,cG.normalized)if d5<=0 then d5=10 end;local d7=Vector3.Cross(cG,cM)/cG.sqrMagnitude;local da=Vector3.Cross(d7,cG.normalized)local dd=Vector3.ProjectOnPlane(d4,cG)return d2*d5*da+0.5*d2*d4 end;function BlockUtil.getWeaponsByName(de,df,aW,dg)if DEBUG then de:Log("searching for "..df)end;local dh={}aW=aW or-1;local Q=aW;if not dg or dg==0 or dg==2 then for F=0,de:GetWeaponCount()-1 do if Q==0 then break end;if de:GetWeaponBlockInfo(F).CustomName==df then table.insert(dh,{subIdx=nil,wpnIdx=F})if DEBUG then de:Log("found weapon "..df.." on hull, type "..de:GetWeaponInfo(F).WeaponType)end;Q=Q-1 end end end;if not dg or dg==1 or dg==2 then local di=de:GetAllSubConstructs()for p=1,#di do local bb=di[p]for F=0,de:GetWeaponCountOnSubConstruct(bb)-1 do if Q==0 then break end;if de:GetWeaponBlockInfoOnSubConstruct(bb,F).CustomName==df then table.insert(dh,{subIdx=bb,wpnIdx=F})if DEBUG then de:Log("found weapon "..df.." on subobj "..bb..", type "..de:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end;Q=Q-1 end end end end;if DEBUG then de:Log("weapon count: "..#dh)end;return dh end;function BlockUtil.getSubConstructsByName(de,df,aW)if DEBUG then de:Log("searching for "..df)end;local di=de:GetAllSubConstructs()local dj={}aW=aW or-1;local Q=aW;for p=1,#di do local bb=di[p]if Q==0 then break end;if de:GetSubConstructInfo(bb).CustomName==df then table.insert(dj,bb)if DEBUG then de:Log("found subobj "..df)end;Q=Q-1 end end;if DEBUG then de:Log("subobj count: "..#dj)end;return dj end;function BlockUtil.getBlocksByName(de,df,dk,aW)if DEBUG then de:Log("searching for "..df)end;local dl={}aW=aW or-1;local Q=aW;for p=0,de:Component_GetCount(dk)-1 do if Q==0 then break end;if de:Component_GetBlockInfo(dk,p).CustomName==df then table.insert(dl,p)if DEBUG then de:Log("found component "..df)end;Q=Q-1 end end;if DEBUG then de:Log("component count: "..#dl)end;return dl end;function BlockUtil.populateWeaponsByName(de,dg)if DEBUG then de:Log("populating all weapons, mode "..dg)end;local dh={}for p=0,de:GetWeaponCount()-1 do local df=de:Component_GetBlockInfo(type,p).CustomName;if df and df~=''then dh[df]=dh[df]or{}table.insert(dh[df],{subIdx=nil,wpnIdx=p})if DEBUG then de:Log("found weapon "..df.." on hull, type "..de:GetWeaponInfo(p).WeaponType)end else table.insert(dh,{subIdx=nil,wpnIdx=p})if DEBUG then de:Log("found unnamed weapon on hull, type "..de:GetWeaponInfo(p).WeaponType)end end end;if not dg or dg==1 or dg==2 then local di=de:GetAllSubConstructs()for p=1,#di do local bb=di[p]for F=0,de:GetWeaponCountOnSubConstruct(bb)-1 do local df=de:Component_GetBlockInfo(type,F).CustomName;if df and df~=''then dh[df]=dh[df]or{}table.insert(dh[df],{subIdx=bb,wpnIdx=F})if DEBUG then de:Log("found weapon "..df.." on subobj "..bb..", type "..de:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end else table.insert(dh,{subIdx=bb,wpnIdx=F})if DEBUG then de:Log("found unnamed weapon on subobj "..bb..", type "..de:GetWeaponInfoOnSubConstruct(bb,F).WeaponType)end end end end end;if DEBUG then local aW=0;for _,a0 in pairs(dh)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;de:Log("weapon count: "..aW)end;return dh end;function BlockUtil.populateSubConstructsByName(de)if DEBUG then de:Log("populating all subconstructs")end;local di=de:GetAllSubConstructs()local dj={}for p=1,#di do local bb=di[p]local df=de:GetSubConstructInfo(bb).CustomName;if df and df~=''then dj[df]=dj[df]or{}table.insert(dj[df],bb)if DEBUG then de:Log("found subobj "..df)end else table.insert(dj,bb)if DEBUG then de:Log("found unnamed subobj")end end end;if DEBUG then local aW=0;for _,a0 in pairs(dj)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;de:Log("subobject count: "..aW)end;return dj end;function BlockUtil.populateBlocksByName(de,dk)if DEBUG then de:Log("populating all blocks of type "..dk)end;local dl={}for p=0,de:Component_GetCount(dk)-1 do local df=de:Component_GetBlockInfo(dk,p).CustomName;if df and df~=''then dl[df]=dl[df]or{}table.insert(dl[df],p)if DEBUG then de:Log("found component "..df)end else table.insert(dl,p)if DEBUG then de:Log("found unnamed component of type "..dk)end end end;if DEBUG then local aW=0;for _,a0 in pairs(dl)do if type(a0)=="table"then aW=aW+#a0 else aW=aW+1 end end;de:Log("component count: "..aW)end;return dl end;function BlockUtil.getWeaponInfo(de,dm)if dm.subIdx then return de:GetWeaponInfoOnSubConstruct(dm.subIdx,dm.wpnIdx)end;return de:GetWeaponInfo(dm.wpnIdx)end;function BlockUtil.getWeaponBlockInfo(de,dm)if dm.subIdx then return de:GetWeaponBlockInfoOnSubConstruct(dm.subIdx,dm.wpnIdx)end;return de:GetWeaponBlockInfo(dm.wpnIdx)end;function BlockUtil.aimWeapon(de,dm,dn,dp)if dm.subIdx then de:AimWeaponInDirectionOnSubConstruct(dm.subIdx,dm.wpnIdx,dn.x,dn.y,dn.z,dp)else de:AimWeaponInDirection(dm.wpnIdx,dn.x,dn.y,dn.z,dp)end end;function BlockUtil.fireWeapon(de,dm,dp)if dm.subIdx then return de:FireWeaponOnSubConstruct(dm.subIdx,dm.wpnIdx,dp)end;return de:FireWeapon(dm.wpnIdx,dp)end;function Combat.pickTarget(de,dq,dr)dr=dr or function(U,ds)return ds.Priority end;local bH,dt;for F in MathUtil.range(de:GetNumberOfTargets(dq))do local ds=de:GetTargetInfo(dq,F)local du=dr(de,ds)if not bH or du>dt then bH=ds;dt=du end end;return bH end;function Combat.CheckConstraints(de,dv,dw,dx)local dy;if dx then dy=de:GetWeaponConstraintsOnSubConstruct(dx,dw)else dy=de:GetWeaponConstraints(dw)end;if not dy or not dy.Valid then return true end;local dz=de:GetConstructForwardVector()local dA=de:GetConstructUpVector()local dB=Quaternion.LookRotation(dz,dA)dv=Quaternion.Inverse(dB)*dv;if dy.InParentConstructSpace and dx then local dC=de:GetSubConstructInfo(dx).localRotation;dv=Quaternion.inverse(dC)*dv end;local dD=MathUtil.angleOnPlane(Vector3.forward,dv,Vector3.up)local dE=dv;dE.y=0;local O=Mathf.Atan2(dv.y,dE.magnitude)local dF=dD>dy.MinAzimuth and dD<dy.MaxAzimuth;local dG=O>dy.MinElevation and O<dy.MaxElevation;if dy.FlipAzimuth then dF=not dF end;if dF and dG then return true end;dD=dD+180;O=180-O;if O>180 then O=O-360 end;if O<-180 then O=O+360 end;dF=dD>dy.MinAzimuth and dD<dy.MaxAzimuth;dG=O>dy.MinElevation and O<dy.MaxElevation;if dy.FlipAzimuth then dF=not dF end;if dF and dG then return true end;return false end;function StringUtil.LogVector(de,bq,dH)de:Log(dH.."("..bq.x..", "..bq.y..", "..bq.z..")")end