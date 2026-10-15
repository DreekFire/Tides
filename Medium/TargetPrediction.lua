-- Canonical world-space motion models. Dependencies: StreamingFourier, FourierWindow, Vector3.
TargetPrediction = {}
local axes = {"x", "y", "z"}
local function finite(value)
    return type(value) == "number" and value == value and math.abs(value) < math.huge
end
local function dot(a, b) return a.x*b.x + a.y*b.y + a.z*b.z end
local function copy(v) return Vector3(v.x, v.y, v.z) end

function TargetPrediction.new(config)
    config = config or {}
    local size, interval = config.windowSize or 128, config.sampleInterval or 0.1
    assert(finite(size) and size >= 4 and size == math.floor(size), "prediction windowSize must be an integer >= 4")
    assert(finite(interval) and interval > 0, "prediction sampleInterval must be positive")
    local harmonics, fraction = config.harmonics or math.min(4, math.floor(size/2)), config.minRelativePower or 0.01
    assert(finite(harmonics) and harmonics >= 1 and harmonics == math.floor(harmonics)
        and harmonics <= math.floor(size/2), "prediction harmonics must fit the window")
    assert(finite(fraction) and fraction >= 0 and fraction <= 1, "prediction minRelativePower must be in [0,1]")
    local gap = config.maxGap or math.max(0.5, interval * 2)
    assert(finite(gap) and gap >= interval, "prediction maxGap must be >= sampleInterval")
    local state = {size = size, interval = interval, harmonics = harmonics,
        minRelativePower = fraction, maxGap = gap, epoch = 0, spectra = {}, observations = {},
        window = FourierWindow.new(size, interval, config)}
    for _, axis in ipairs(axes) do state.spectra[axis] = StreamingFourier.new(size) end
    return state
end

function TargetPrediction.reset(state)
    for _, axis in ipairs(axes) do StreamingFourier.reset(state.spectra[axis]) end
    state.observations, state.components = {}, nil
    state.stamp, state.gridStamp, state.gridPosition = nil, nil, nil
    state.velocity, state.acceleration, state.position, state.drift = nil, nil, nil, nil
    state.epoch = state.epoch + 1
end

-- Integrate Fourier coefficients of position increments, separating DC drift.
-- Position-domain power ranks frequencies jointly across all three axes.
local function fitSpectrum(state)
    local candidates = {}
    for k = 1, math.floor(state.size/2) do
        local w = 2 * math.pi * k / state.size
        local c, s = math.cos(w), math.sin(w)
        local denominator, power = 2 * (1-c), 0
        local scale = (state.size % 2 == 0 and k == state.size/2) and 1/state.size or 2/state.size
        for _, axis in ipairs(axes) do
            local spectrum = state.spectra[axis]
            local dr, di = spectrum.real[k+1], spectrum.imag[k+1]
            local lr, li = dr*c + di*s, di*c - dr*s
            local re = scale * (lr*(1-c) + li*s) / denominator
            local im = scale * (li*(1-c) - lr*s) / denominator
            power = power + re^2 + im^2
        end
        candidates[#candidates+1] = {bin = k, omega = w/state.interval, power = power}
    end
    table.sort(candidates, function(a, b)
        if a.power == b.power then return a.bin < b.bin end
        return a.power > b.power
    end)
    state.components = {}
    for i = 1, math.min(state.harmonics, #candidates) do
        local candidate = candidates[i]
        if candidate.power > 1e-18 and candidate.power >= candidates[1].power * state.minRelativePower then
            state.components[#state.components+1] = candidate
        end
    end
    local bins = {}
    for i, component in ipairs(state.components) do bins[i] = component.bin end
    local fit = FourierWindow.fit(state.window,
        {state.spectra.x, state.spectra.y, state.spectra.z}, bins)
    state.drift = Vector3(fit.dc[1], fit.dc[2], fit.dc[3]) / state.interval
    for i, component in ipairs(state.components) do
        local estimate = fit.components[i]
        local w = component.omega*state.interval
        local c, sn = math.cos(w), math.sin(w)
        local denominator, re, im = 2*(1-c), {}, {}
        for axis = 1, 3 do
            local a, b = estimate.cosine[axis], -estimate.sine[axis]
            re[axis] = (a*(1-c)+b*sn)/denominator
            im[axis] = (b*(1-c)-a*sn)/denominator
        end
        component.real = Vector3(re[1], re[2], re[3])
        component.imag = Vector3(im[1], im[2], im[3])
        component.halfLife = estimate.halfLife
    end
end

-- Accept actual timestamps, linearly resampling onto an anchored uniform grid.
-- Duplicates are rejected without mutation; backwards time or long gaps start
-- a new epoch. No spectrum is manufactured until N real increments exist.
function TargetPrediction.update(state, position, stamp)
    assert(finite(stamp), "prediction timestamp must be finite")
    for _, axis in ipairs(axes) do assert(position and finite(position[axis]), "prediction position must be finite") end
    if state.stamp and stamp == state.stamp then return false, "duplicate-timestamp" end
    local reason = "sampled"
    if state.stamp and (stamp < state.stamp or stamp - state.stamp > state.maxGap) then
        reason = stamp < state.stamp and "clock-reset" or "observation-gap"
        TargetPrediction.reset(state)
    end
    position = copy(position)
    local previous = state.observations[#state.observations]
    local changed = false
    if previous then
        local nextStamp = state.gridStamp + state.interval
        while nextStamp <= stamp + state.interval * 1e-8 do
            local alpha = math.min(1, (nextStamp - previous.stamp) / (stamp - previous.stamp))
            local point = previous.position + (position - previous.position) * alpha
            local delta = point - state.gridPosition
            for _, axis in ipairs(axes) do StreamingFourier.update(state.spectra[axis], delta[axis]) end
            state.gridStamp, state.gridPosition = nextStamp, point
            nextStamp, changed = nextStamp + state.interval, true
        end
    else
        state.gridStamp, state.gridPosition = stamp, position
    end
    local observations = state.observations
    observations[#observations+1] = {position = position, stamp = stamp}
    if #observations > 3 then table.remove(observations, 1) end
    state.position, state.stamp = position, stamp
    if #observations >= 2 then
        local a, b = observations[#observations-1], observations[#observations]
        state.velocity = (b.position - a.position) / (b.stamp - a.stamp)
        if #observations == 3 then
            local first = observations[1]
            local oldVelocity = (a.position - first.position) / (a.stamp - first.stamp)
            state.acceleration = (state.velocity - oldVelocity) * (2 / (b.stamp - first.stamp))
        end
    end
    if changed and state.spectra.x.count == state.size then fitSpectrum(state) end
    return true, reason
end

-- Constant-speed osculating circle uses the normal component of acceleration.
-- Its zero-curvature/zero-speed limits are explicitly linear/stationary.
function TargetPrediction.kinematic(position, velocity, acceleration, horizon, model)
    assert(finite(horizon) and horizon >= 0, "prediction horizon must be finite and nonnegative")
    if model == "linear" then return position + velocity * horizon end
    assert(model == "quadratic" or model == "circle", "unknown kinematic prediction model")
    if model == "quadratic" then return position + velocity*horizon + acceleration*(0.5*horizon^2) end
    local speed2 = dot(velocity, velocity)
    if speed2 <= 1e-18 then return copy(position) end
    local normal = acceleration - velocity * (dot(acceleration, velocity)/speed2)
    local omega2 = dot(normal, normal) / speed2
    if omega2 * horizon^2 < 1e-12 then
        return position + velocity*horizon + normal*(0.5*horizon^2)
    end
    local omega = math.sqrt(omega2)
    -- 2 sin^2(theta/2) avoids cancellation near zero.
    return position + velocity*(math.sin(omega*horizon)/omega)
        + normal*(2*math.sin(omega*horizon/2)^2/omega2)
end

function TargetPrediction.predict(state, model, horizon)
    assert(finite(horizon) and horizon >= 0, "prediction horizon must be finite and nonnegative")
    assert(model == "fourier" or model == "linear" or model == "quadratic" or model == "circle", "unknown prediction model")
    local result = {model = model, horizon = horizon, sourceStamp = state.stamp, ready = false}
    if not state.position then result.reason = "no-observations"; return result end
    if model == "fourier" then
        if not state.components then result.reason = "fourier-warmup"; return result end
        local offset = state.stamp - state.gridStamp
        local predicted = state.position + state.drift * horizon
        for _, component in ipairs(state.components) do
            local a, b = component.omega*offset, component.omega*(offset+horizon)
            predicted = predicted + component.real*(math.cos(b)-math.cos(a))
                - component.imag*(math.sin(b)-math.sin(a))
        end
        result.position, result.componentCount = predicted, #state.components
        result.frequencyResolution = 1 / (state.size * state.interval)
        result.window = {method = "frequency-exponential", cycles = state.window.cycles,
            minHalfLife = state.window.minHalfLife, maxHalfLife = state.window.maxHalfLife,
            dcHalfLife = FourierWindow.halfLife(state.window, 0), components = {}}
        for i, component in ipairs(state.components) do
            result.window.components[i] = {bin = component.bin,
                frequency = component.omega/(2*math.pi), halfLife = component.halfLife}
        end
    else
        if not state.velocity or (model ~= "linear" and not state.acceleration) then
            result.reason = "derivative-warmup"; return result
        end
        local velocity = state.velocity
        if model ~= "linear" then
            local observations = state.observations
            velocity = velocity + state.acceleration*((observations[3].stamp-observations[2].stamp)/2)
        end
        result.position = TargetPrediction.kinematic(state.position, velocity, state.acceleration, horizon, model)
    end
    result.ready = true
    return result
end
