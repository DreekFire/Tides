assert(not pcall(GetTargetPredictionComparisonReport))
local stamp, visible, logCount = 0, true, 0
local interface = {
  GetTimeSinceSpawn=function() return stamp end,
  GetNumberOfTargets=function() return visible and 2 or 0 end,
  GetTargetInfo=function(_,_,index)
    if index==1 then return {Valid=false} end
    return {Valid=true,Id=42,Position=Vector3(100+2*stamp,10-stamp,3*stamp)}
  end,
  Log=function(_,message) assert(type(message)=='string'); logCount=logCount+1 end,
}
local report
for step=0,170 do
  stamp=step*0.1
  report=Update(interface)
  assert(report==GetTargetPredictionComparisonReport())
end
assert(logCount>2 and report.byId[42].windowSize==128)
assert(report.byId[42].status=='comparing')
for _,row in ipairs(report.byId[42].horizons) do
  assert(row.models.fourier.count>0 and row.models.fourier.rmse<1e-7)
end
local count=report.byId[42].horizons[1].models.fourier.count
local duplicate=Update(interface)
assert(not duplicate.byId[42].accepted and duplicate.byId[42].horizons[1].models.fourier.count==count)
visible=false; stamp=stamp+0.1
assert(next(Update(interface).byId)==nil)
visible=true; stamp=stamp+0.1
assert(Update(interface).byId[42].status=='warming-up')
print('Standalone FtD comparison launcher contracts passed')
