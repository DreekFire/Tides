-- Standalone FtD entry point: paste Header.lua, this script, then Tides.lua.
-- Read-only target observations and periodic local logs; positions are world CoM.
local comparisonConfig = {
    windowSize = 128, sampleInterval = 0.1, harmonics = 4,
    minRelativePower = 0.01, maxGap = 0.5, horizons = {0.5, 1, 2},
    selectorHalfLife = 3, windowCycles = 1, windowMinHalfLife = 1.6, windowMaxHalfLife = 12.8,
}
local comparisonMainframe = 0
local comparisonLogInterval = 5
local comparison, lastReport, lastLogStamp

function GetTargetPredictionComparisonReport()
    assert(lastReport, "prediction comparison requires a successful Update first")
    return lastReport
end

function Update(I)
    if not comparison then comparison = TargetPredictionComparison.new(comparisonConfig) end
    local stamp, positions = I:GetTimeSinceSpawn(), {}
    for index = 0, I:GetNumberOfTargets(comparisonMainframe)-1 do
        local target = I:GetTargetInfo(comparisonMainframe, index)
        if target.Valid then positions[target.Id] = target.Position end
    end
    lastReport = comparison:update(positions, stamp)
    if not lastLogStamp or stamp < lastLogStamp or stamp-lastLogStamp >= comparisonLogInterval then
        lastLogStamp = stamp
        for id, target in pairs(lastReport.byId) do
            if target.status == "warming-up" then
                I:Log(string.format('Prediction target=%s warming up %d/%d increments',
                    tostring(id), target.sampleCount, target.windowSize))
            else
                for _, row in ipairs(target.horizons) do
                    if #row.winners > 0 then
                        I:Log(string.format('Prediction target=%s h=%.2fs n=%d RMSE Fourier=%.3f linear=%.3f quadratic=%.3f circle=%.3f selector=%.3f best=%s',
                            tostring(id), row.horizon, row.models.fourier.count,
                            row.models.fourier.rmse, row.models.linear.rmse,
                            row.models.quadratic.rmse, row.models.circle.rmse, row.models.selector.rmse,
                            table.concat(row.winners, ',')))
                    end
                end
            end
        end
    end
    return lastReport
end
