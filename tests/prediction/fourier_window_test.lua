local function close(actual,expected,tolerance)
  assert(math.abs(actual-expected)<(tolerance or 1e-8),tostring(actual)..' ~= '..tostring(expected))
end
local window=FourierWindow.new(64,0.1)
close(FourierWindow.halfLife(window,0),6.4)
close(FourierWindow.halfLife(window,1),6.4)
close(FourierWindow.halfLife(window,2),3.2)
close(FourierWindow.halfLife(window,8),0.8)
close(FourierWindow.halfLife(window,32),0.8)
for bin=0,32 do
  local halfLife=FourierWindow.halfLife(window,bin)
  close(FourierWindow.weight(window,bin,0),1)
  close(FourierWindow.weight(window,bin,halfLife),0.5)
  close(FourierWindow.weight(window,bin,2*halfLife),0.25)
  if bin>0 then assert(halfLife<=FourierWindow.halfLife(window,bin-1)) end
end
assert(FourierWindow.weight(window,1,2)>FourierWindow.weight(window,8,2))
local signals={StreamingFourier.new(64),StreamingFourier.new(64)}
for step=0,63 do
  local low,high=2*math.pi*step/64,2*math.pi*8*step/64
  StreamingFourier.update(signals[1],3+2*math.cos(low)+4*math.sin(low)+5*math.cos(high))
  StreamingFourier.update(signals[2],-2+7*math.cos(low)-3*math.sin(high))
end
local fit=FourierWindow.fit(window,signals,{1,8})
close(fit.dc[1],3); close(fit.dc[2],-2)
local low,high=2*math.pi*63/64,2*math.pi*8*63/64
close(fit.components[1].cosine[1],2*math.cos(low)+4*math.sin(low))
close(fit.components[1].sine[1],-2*math.sin(low)+4*math.cos(low))
close(fit.components[2].cosine[1],5*math.cos(high))
close(fit.components[2].sine[1],-5*math.sin(high))
close(fit.components[1].cosine[2],7*math.cos(low))
close(fit.components[2].sine[2],-3*math.cos(high))
assert(fit.components[1].halfLife>fit.components[2].halfLife)
-- Same amplitude removal, same age: high-frequency memory must forget faster.
signals={StreamingFourier.new(64),StreamingFourier.new(64)}
for step=0,63 do
  local amplitude=step<48 and 1 or 0
  StreamingFourier.update(signals[1],amplitude*math.cos(2*math.pi*step/64))
  StreamingFourier.update(signals[2],amplitude*math.cos(2*math.pi*8*step/64))
end
fit=FourierWindow.fit(window,signals,{1,8})
local lowAmplitude=math.sqrt(fit.components[1].cosine[1]^2+fit.components[1].sine[1]^2)
local highAmplitude=math.sqrt(fit.components[2].cosine[2]^2+fit.components[2].sine[2]^2)
assert(highAmplitude<lowAmplitude*0.6,'high-frequency memory must decay faster')
-- Nyquist has only one column; the full basis must still be identifiable.
local edge=StreamingFourier.new(8)
for step=0,7 do StreamingFourier.update(edge,2+3*(-1)^step+math.sin(2*math.pi*step/8)) end
local edgeFit=FourierWindow.fit(FourierWindow.new(8,0.1),{edge},{1,2,3,4})
close(edgeFit.dc[1],2)
close(edgeFit.components[4].cosine[1],-3)
close(edgeFit.components[4].sine[1],0)
assert(not pcall(FourierWindow.fit,window,{StreamingFourier.new(64)},{1}))
assert(not pcall(FourierWindow.fit,window,signals,{1,1}))
assert(not pcall(FourierWindow.fit,window,signals,{0}))
local shifted=StreamingFourier.new(64)
for i=1,65 do StreamingFourier.update(shifted,1) end
assert(not pcall(FourierWindow.fit,window,{signals[1],shifted},{1}))
for _,options in ipairs({{windowCycles=0},{windowCycles=math.huge},
  {windowMinHalfLife=0.01},{windowMinHalfLife=2,windowMaxHalfLife=1}}) do
  assert(not pcall(FourierWindow.new,64,0.1,options))
end
assert(not pcall(FourierWindow.weight,window,1,-1))
assert(not pcall(FourierWindow.halfLife,window,33))
print('Frequency-dependent Fourier window contracts passed')
