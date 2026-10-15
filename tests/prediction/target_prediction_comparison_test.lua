local function near(actual,expected,tolerance)
  assert(math.abs(actual-expected)<(tolerance or 1e-7),tostring(actual)..' ~= '..tostring(expected))
end
local models={'fourier','linear','quadratic','circle','selector'}
-- Standalone lifecycle and independent target ownership, with no host system.
local comparison=TargetPredictionComparison.new({windowSize=16,horizons={0.5,1,2}})
local report
for step=0,80 do
  local stamp=step*0.1
  report=comparison:update({[42]=Vector3(2*stamp,-stamp,3*stamp),[7]=Vector3(100+stamp,4,0)},stamp)
end
assert(report.type=='target/prediction-comparison' and report.schemaVersion==1)
assert(report.byId[42].status=='comparing' and report.byId[42].matchedOrigins)
assert(comparison.states[42].predictor ~= comparison.states[7].predictor)
for _,row in ipairs(report.byId[42].horizons) do
  for _,model in ipairs(models) do
    assert(row.models[model].count>0 and row.models[model].count==row.models.fourier.count)
    near(row.models[model].rmse,0)
  end
  assert(#row.winners==5 and row.winner==nil) -- report ties honestly
end
local oldCount=report.byId[42].horizons[1].models.fourier.count
local repeated=comparison:update({[42]=Vector3(999,0,0),[7]=Vector3(108,4,0)},8)
assert(not repeated.byId[42].accepted and repeated.byId[42].reason=='duplicate-timestamp')
assert(repeated.byId[42].horizons[1].models.fourier.count==oldCount)
comparison:update({[42]=Vector3(16.2,-8.1,24.3),[7]=Vector3(108.1,4,0)},8.1)
assert(report.byId[42].horizons[1].models.fourier.count==oldCount) -- immutable prior snapshot
local preserved=comparison.states[7]
comparison:update({[7]=Vector3(108.2,4,0)},8.2)
assert(not comparison.states[42] and comparison.states[7]==preserved)
local reacquired=comparison:update({[42]=Vector3.zero,[7]=Vector3(108.3,4,0)},8.3)
assert(reacquired.byId[42].status=='warming-up')
local gap=comparison:update({[42]=Vector3.zero},10)
assert(gap.byId[42].reason=='observation-gap' and gap.byId[42].pendingCount==0)
assert(gap.byId[42].horizons[1].models.fourier.count==0)
local reset=comparison:update({[42]=Vector3.zero},0)
assert(reset.byId[42].reason=='clock-reset')
local empty=comparison:update({},0.1)
assert(next(comparison.states)==nil and next(empty.byId)==nil)
for _,horizons in ipairs({{},{0},{1,0.5},{1,1},{math.huge}}) do
  assert(not pcall(TargetPredictionComparison.new,{horizons=horizons}))
end
assert(not pcall(comparison.update,comparison,{[7]=Vector3(0/0,0,0)},1))
assert(not pcall(comparison.update,comparison,nil,1))
assert(not pcall(comparison.update,comparison,{},math.huge))

-- Comparison copies/forwards the normalized window policy to each target.
local windowOptions={windowSize=16,windowCycles=0.5,windowMinHalfLife=0.2,windowMaxHalfLife=1.2}
local configured=TargetPredictionComparison.new(windowOptions)
windowOptions.windowCycles=99
local configuredReport
for step=0,40 do
  local stamp=step*0.1
  configuredReport=configured:update({[2]=Vector3(math.cos(2*math.pi*step/16),0,math.sin(2*math.pi*step/16))},stamp)
end
local policy=configuredReport.byId[2].forecasts[1].models.fourier.window
near(policy.cycles,0.5); near(policy.minHalfLife,0.2); near(policy.maxHalfLife,1.2)
near(policy.components[1].halfLife,0.8)
assert(policy.method=='frequency-exponential')

-- Saved forecasts are scored by future observations, never by refitting.
local causal=TargetPredictionComparison.new({windowSize=4,sampleInterval=1,maxGap=2,horizons={2}})
local saved
for stamp=0,5 do
  saved=causal:update({[3]=Vector3(stamp,0,0)},stamp)
  assert(saved.byId[3].horizons[1].models.fourier.count==0)
end
local scored=causal:update({[3]=Vector3(20,0,0)},6).byId[3]
for _,model in ipairs(models) do
  local metric=scored.horizons[1].models[model]
  assert(metric.count==1)
  near(metric.meanError,14); near(metric.rmse,14); near(metric.maxError,14)
  assert(saved.byId[3].horizons[1].models[model].count==0)
end

-- Deterministic forward-only scenarios; all models share origins and horizons.
local omega = 2*math.pi/6.4
local cases = {
  {'stationary',function(t) return Vector3(100,20,-30) end},
  {'linear',function(t) return Vector3(100+7*t,20-t,3*t) end},
  {'quadratic',function(t) return Vector3(100+7*t+0.3*t*t,20+0.1*t*t,3*t) end},
  {'circle',function(t) return Vector3(30*math.cos(omega*t),20,30*math.sin(omega*t)) end},
  {'helix',function(t) return Vector3(30*math.cos(omega*t),20+4*t,30*math.sin(omega*t)) end},
  {'multi-harmonic',function(t)
    return Vector3(7*t+20*math.cos(omega*t),20+5*math.sin(3*omega*t),30*math.sin(2*omega*t))
  end},
  {'off-bin-circle',function(t) return Vector3(30*math.cos(1.37*t),20,30*math.sin(1.37*t)) end},
  {'short-arc-circle',function(t) return Vector3(60*math.cos(0.16*t),20,60*math.sin(0.16*t)) end},
  {'noisy-circle',function(t)
    return Vector3(30*math.cos(omega*t)+0.05*math.sin(53*t),20+0.05*math.cos(71*t),
      30*math.sin(omega*t)+0.05*math.sin(83*t))
  end},
  {'changing-turn',function(t)
    local angle=omega*t+0.015*t*t
    return Vector3(30*math.cos(angle),20,30*math.sin(angle))
  end},
  {'turn-onset',function(t)
    if t<=15 then return Vector3(5*t,20,0) end
    local s=t-15
    return Vector3(75+10*math.sin(0.5*s),20,10*(1-math.cos(0.5*s)))
  end},
}
local results = {}
for _,case in ipairs(cases) do
  local instance = TargetPredictionComparison.new({windowSize=64,sampleInterval=0.1,
    harmonics=4,horizons={0.5,1,2}})
  local outputs
  for step=0,320 do
    local stamp=step*0.1
    outputs=instance:update({[7]=case[2](stamp)},stamp)
    local product = outputs.byId[7]
    assert(product.pendingCount <= 3*23, 'unbounded pending forecasts')
    if step<64 then
      assert(product.horizons[1].models.fourier.count == 0)
      assert(not outputs.byId[7].forecasts[1].models.fourier.ready)
    end
  end
  local product=outputs.byId[7]
  results[case[1]]=product
  for _,row in ipairs(product.horizons) do
    assert(#row.winners > 0 and row.models.fourier.count > 100)
    for _,model in ipairs(models) do
      assert(row.models[model].count == row.models.fourier.count)
      assert(row.models[model].rmse >= 0 and row.models[model].rmse < math.huge)
    end
    if PREDICTION_REPORT then
      print(string.format('%-18s h=%3.1f n=%3d fourier=%9.4f linear=%9.4f quadratic=%9.4f circle=%9.4f selector=%9.4f winner=%s',
        case[1],row.horizon,row.models.fourier.count,row.models.fourier.rmse,
        row.models.linear.rmse,row.models.quadratic.rmse,row.models.circle.rmse,row.models.selector.rmse,table.concat(row.winners,",")))
    end
  end
end
for _,name in ipairs({'stationary','linear','circle','helix','multi-harmonic'}) do
  for _,row in ipairs(results[name].horizons) do near(row.models.fourier.rmse,0,1e-6) end
end
for _,row in ipairs(results.quadratic.horizons) do near(row.models.quadratic.rmse,0,1e-6) end
assert(results.circle.horizons[3].models.circle.rmse < results.circle.horizons[3].models.linear.rmse)
assert(results['short-arc-circle'].horizons[1].models.circle.rmse < results['short-arc-circle'].horizons[1].models.fourier.rmse)
assert(results['off-bin-circle'].horizons[3].models.circle.rmse < results['off-bin-circle'].horizons[3].models.fourier.rmse)

-- Jittered input and fractional due times exercise actual observation brackets.
local instance=TargetPredictionComparison.new({windowSize=16,sampleInterval=0.1,
  harmonics=3,horizons={0.135,0.43}})
for step=0,120 do
  local stamp=step*0.07+(step%2)*0.005
  local outputs=instance:update({[42]=Vector3(2*stamp,3*stamp,4*stamp)},stamp)
  local product=outputs.byId[42]
  for _,row in ipairs(product.horizons) do
    for _,model in ipairs(models) do if row.models[model].count>0 then near(row.models[model].rmse,0) end end
  end
end
print('Standalone target prediction comparison contracts passed')

-- Selection is causal: before the first matured forecast it uses linear.
local selectionTest=TargetPredictionComparison.new({windowSize=4,sampleInterval=0.1,horizons={1}})
for step=0,13 do
  local frame=selectionTest:update({[1]=Vector3(step*step,0,0)},step*0.1).byId[1]
  assert(frame.forecasts[1].models.selector.selectedModel=='linear')
end
local first=selectionTest:update({[1]=Vector3(196,0,0)},1.4).byId[1]
assert(first.forecasts[1].models.selector.selectedModel=='quadratic')
assert(first.horizons[1].models.selector.count==1)
assert(not pcall(TargetPredictionComparison.new,{selectorHalfLife=0}))
