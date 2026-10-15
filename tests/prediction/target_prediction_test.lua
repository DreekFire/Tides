local function near(a,b,tolerance)
  assert((a-b).magnitude < (tolerance or 1e-7), 'position error: '..(a-b).magnitude)
end
local options = {windowSize=32,sampleInterval=0.1,harmonics=4,maxGap=0.5}
local omega = 2*math.pi/3.2
local function motion(t)
  return Vector3(1000+7*t+20*math.cos(omega*t),
    -400+3*t+4*math.sin(3*omega*t), 100+20*math.sin(omega*t))
end
local state = TargetPrediction.new(options)
assert(not TargetPrediction.predict(state,'fourier',1).ready)
for i=0,96 do
  local stamp = i*0.1
  TargetPrediction.update(state,motion(stamp),stamp)
  if i < 32 then assert(not TargetPrediction.predict(state,'fourier',1).ready) end
  if i >= 32 then
    for _, horizon in ipairs({0,0.15,0.7,3.2}) do
      local forecast = TargetPrediction.predict(state,'fourier',horizon)
      assert(forecast.ready and forecast.sourceStamp == stamp)
      near(forecast.position,motion(stamp+horizon))
    end
  end
end
-- Forecast from raw observations between grid points, including odd windows.
for _,size in ipairs({15,16,31}) do
  local between=TargetPrediction.new({windowSize=size,sampleInterval=0.1,harmonics=2})
  local function orbit(t)
    local angle=2*math.pi*t/(size*0.1)
    return Vector3(4*t+10*math.cos(angle),2*t,10*math.sin(angle))
  end
  for i=0,4*size do
    local stamp=i*0.05
    TargetPrediction.update(between,orbit(stamp),stamp)
    if i>=2*size then
      near(TargetPrediction.predict(between,'fourier',0.37).position,orbit(stamp+0.37))
    end
  end
end
-- Snapshot output must not change on the next update.
local snapshot = TargetPrediction.predict(state,'fourier',0.7)
local saved = Vector3(snapshot.position.x,snapshot.position.y,snapshot.position.z)
TargetPrediction.update(state,motion(9.7),9.7)
near(snapshot.position,saved)
local count = state.spectra.x.count
local accepted, reason = TargetPrediction.update(state,Vector3(9999,0,0),9.7)
assert(not accepted and reason == 'duplicate-timestamp' and state.spectra.x.count == count)
near(state.position,motion(9.7))
local epoch = state.epoch
accepted, reason = TargetPrediction.update(state,motion(11),11)
assert(accepted and reason == 'observation-gap' and state.epoch == epoch+1)
assert(not TargetPrediction.predict(state,'fourier',1).ready)
accepted, reason = TargetPrediction.update(state,motion(1),1)
assert(accepted and reason == 'clock-reset' and state.epoch == epoch+2)
TargetPrediction.reset(state)
assert(state.stamp == nil and state.spectra.x.count == 0)

-- Actual nonuniform timestamps; quadratic derivatives are exact at the endpoint.
state = TargetPrediction.new(options)
local function parabola(t) return Vector3(4+2*t+3*t*t,7-t*t,3*t) end
for _,stamp in ipairs({0,0.08,0.21}) do TargetPrediction.update(state,parabola(stamp),stamp) end
near(TargetPrediction.predict(state,'quadratic',0.9).position,parabola(1.11))
assert(not TargetPrediction.predict(state,'fourier',0.9).ready)
-- Resampling is exact for steady drift even under jitter; Fourier DC must
-- continue translation instead of periodically wrapping world coordinates.
state = TargetPrediction.new(options)
local function line(t) return Vector3(5+3*t,-1+2*t,8-4*t) end
for i=0,180 do
  local stamp = i*0.06 + (i%2)*0.007
  TargetPrediction.update(state,line(stamp),stamp)
end
for _,model in ipairs({'linear','quadratic','circle','fourier'}) do
  near(TargetPrediction.predict(state,model,2.31).position,line(state.stamp+2.31))
end
-- Analytic osculating circle contract, including vertical/rotated motion.
local p,v,a = Vector3(0,2,0),Vector3(0,0,6),Vector3(0,-18,0)
near(TargetPrediction.kinematic(p,v,a,0.7,'circle'),Vector3(0,2*math.cos(2.1),2*math.sin(2.1)))
near(TargetPrediction.kinematic(p,v,Vector3.zero,0.7,'circle'),p+v*0.7)
near(TargetPrediction.kinematic(p,Vector3.zero,a,1,'circle'),p)
-- Constant signals and exact Nyquist bins remain finite.
for _, mode in ipairs({'constant','nyquist'}) do
  state = TargetPrediction.new(options)
  for i=0,96 do
    local point = mode == 'constant' and Vector3(7,8,9) or Vector3((-1)^i,0,0)
    TargetPrediction.update(state,point,i*0.1)
  end
  local forecast = TargetPrediction.predict(state,'fourier',0.1)
  near(forecast.position,mode == 'constant' and Vector3(7,8,9) or Vector3(-1,0,0))
end
for _,bad in ipairs({{windowSize=3},{sampleInterval=0},{maxGap=0.01},
  {harmonics=99,windowSize=8},{minRelativePower=2},{windowSize=math.huge}}) do
  assert(not pcall(TargetPrediction.new,bad))
end
assert(not pcall(TargetPrediction.update,state,Vector3(0/0,0,0),10))
assert(not pcall(TargetPrediction.predict,state,'fourier',-1))
assert(not pcall(TargetPrediction.predict,state,'legacy',1))
-- The canonical predictor must use the configured windows, not only expose metadata.
local fast=TargetPrediction.new({windowSize=64,sampleInterval=0.1,harmonics=2,minRelativePower=0})
local slow=TargetPrediction.new({windowSize=64,sampleInterval=0.1,harmonics=2,minRelativePower=0,
  windowMinHalfLife=6.4,windowMaxHalfLife=6.4})
local position=Vector3.zero
TargetPrediction.update(fast,position,0); TargetPrediction.update(slow,position,0)
for step=0,63 do
  local amplitude=step<48 and 1 or 0
  position=position+Vector3(amplitude*2*math.sin(math.pi/64)*math.cos(2*math.pi*step/64),
    0,amplitude*2*math.sin(8*math.pi/64)*math.cos(2*math.pi*8*step/64))
  TargetPrediction.update(fast,position,(step+1)*0.1)
  TargetPrediction.update(slow,position,(step+1)*0.1)
end
local function component(model,bin)
  for _,item in ipairs(model.components) do if item.bin==bin then return item end end
  error('missing test frequency '..bin)
end
local fastHigh,slowHigh=component(fast,8),component(slow,8)
local fastAmplitude=math.sqrt(fastHigh.real.z^2+fastHigh.imag.z^2)
local slowAmplitude=math.sqrt(slowHigh.real.z^2+slowHigh.imag.z^2)
assert(fastAmplitude<slowAmplitude*0.6)
near(component(fast,1).real,component(slow,1).real)
assert(TargetPrediction.predict(fast,'fourier',1).window.method=='frequency-exponential')
local metadata=TargetPrediction.predict(fast,'fourier',1).window
assert(metadata.minHalfLife==0.8 and metadata.maxHalfLife==6.4)
TargetPrediction.update(fast,Vector3.zero,9)
assert(not TargetPrediction.predict(fast,'fourier',1).ready)
print('Target prediction numerical contracts passed')
