-- Standalone causal comparison. Dependencies: TargetPrediction, Vector3.
-- Every call supplies the complete visible target-id -> world-position map.
TargetPredictionComparison = {}
local models = {"fourier", "linear", "quadratic", "circle"}
local function finite(value)
    return type(value) == "number" and value == value and math.abs(value) < math.huge
end

local function newState(predictor, horizons)
    local metrics = {}
    for i in ipairs(horizons) do
        metrics[i] = {}
        for _, model in ipairs(models) do
            metrics[i][model] = {count = 0, sum = 0, sumSquared = 0, max = 0}
        end
    end
    return {predictor = predictor, epoch = predictor.epoch, pending = {}, metrics = metrics}
end

local function snapshot(self, state)
    local rows = {}
    for i, horizon in ipairs(self.horizons) do
        local row, best = {horizon = horizon, models = {}, winners = {}}, nil
        for _, model in ipairs(models) do
            local metric = state.metrics[i][model]
            local result = {count = metric.count}
            if metric.count > 0 then
                result.meanError = metric.sum / metric.count
                result.rmse = math.sqrt(metric.sumSquared / metric.count)
                result.maxError = metric.max
                if not best or result.rmse < best then best = result.rmse end
            end
            row.models[model] = result
        end
        if best then
            for _, model in ipairs(models) do
                if row.models[model].rmse <= best + 1e-6 then row.winners[#row.winners+1] = model end
            end
            if #row.winners == 1 then row.winner = row.winners[1] end
        end
        rows[i] = row
    end
    return rows
end

function TargetPredictionComparison.new(config)
    config = config or {}
    -- Validate model options once, before any observations arrive.
    local validated = TargetPrediction.new(config)
    local horizons = config.horizons or {0.5, 1, 2}
    assert(type(horizons) == "table" and #horizons > 0, "prediction horizons must be a nonempty list")
    local copied, previous = {}, 0
    for i, value in ipairs(horizons) do
        assert(finite(value) and value > previous,
            "prediction horizons must be finite, positive and strictly increasing")
        copied[i], previous = value, value
    end
    -- Retain normalized options rather than a mutable caller-owned config.
    return setmetatable({horizons = copied, interval = validated.interval, states = {}, options = {
        windowSize = validated.size, sampleInterval = validated.interval,
        harmonics = validated.harmonics, maxGap = validated.maxGap,
        minRelativePower = validated.minRelativePower,
        windowCycles = validated.window.cycles, windowMinHalfLife = validated.window.minHalfLife,
        windowMaxHalfLife = validated.window.maxHalfLife,
    }}, {__index = TargetPredictionComparison})
end

local function score(state, position, stamp)
    local pending = {}
    for _, saved in ipairs(state.pending) do
        if saved.due <= stamp then
            assert(state.stamp and saved.due >= state.stamp, "comparison missed its observation bracket")
            -- Both bounding observations have arrived. Never refit a saved forecast.
            local alpha = (saved.due - state.stamp) / (stamp - state.stamp)
            local actual = state.position + (position - state.position) * alpha
            for _, model in ipairs(models) do
                local delta = saved.models[model].position - actual
                local distance = math.sqrt(delta.x^2 + delta.y^2 + delta.z^2)
                local metric = state.metrics[saved.index][model]
                metric.count, metric.sum = metric.count + 1, metric.sum + distance
                metric.sumSquared = metric.sumSquared + distance^2
                metric.max = math.max(metric.max, distance)
            end
        else pending[#pending+1] = saved end
    end
    state.pending = pending
end

function TargetPredictionComparison:update(positions, stamp)
    assert(type(positions) == "table" and finite(stamp), "comparison requires a target position map and finite timestamp")
    for _, position in pairs(positions) do
        assert(position and finite(position.x) and finite(position.y) and finite(position.z),
            "comparison requires finite world-space target positions")
    end
    for id in pairs(self.states) do if not positions[id] then self.states[id] = nil end end
    local byId = {}
    for id, position in pairs(positions) do
        local state = self.states[id]
        local predictor = state and state.predictor or TargetPrediction.new(self.options)
        local accepted, reason = TargetPrediction.update(predictor, position, stamp)
        if not state or state.epoch ~= predictor.epoch then state = newState(predictor, self.horizons) end
        self.states[id] = state
        local forecasts, ready = {}, true
        for i, horizon in ipairs(self.horizons) do
            local estimates = {}
            for _, model in ipairs(models) do
                estimates[model] = TargetPrediction.predict(predictor, model, horizon)
                if not estimates[model].ready then ready = false end
            end
            forecasts[i] = {horizon = horizon, models = estimates}
        end
        if accepted then
            score(state, position, stamp)
            -- Matched origins: exclude warmup for every model, and issue at most
            -- once per resampling interval. Pending storage is horizon-bounded.
            if ready and (not state.lastIssued or stamp-state.lastIssued >= self.interval-1e-9) then
                for i, forecast in ipairs(forecasts) do
                    state.pending[#state.pending+1] = {index = i, due = stamp+forecast.horizon,
                        models = forecast.models}
                end
                state.lastIssued = stamp
            end
            state.stamp = stamp
            state.position = Vector3(position.x, position.y, position.z)
        end
        byId[id] = {
            type = "target/prediction-comparison", schemaVersion = 1, frame = "world", targetId = id,
            stamp = stamp, sourceStamp = predictor.stamp, epoch = state.epoch,
            accepted = accepted, reason = reason, status = ready and "comparing" or "warming-up",
            matchedOrigins = true, groundTruth = "timestamp-bracket-linear-interpolation",
            sampleCount = predictor.spectra.x.count, windowSize = predictor.size,
            sampleInterval = predictor.interval, pendingCount = #state.pending,
            forecasts = forecasts, horizons = snapshot(self, state),
        }
    end
    return {type = "target/prediction-comparison", schemaVersion = 1, frame = "world", stamp = stamp, byId = byId}
end
