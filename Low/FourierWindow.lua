-- Frequency-dependent exponential windows for harmonic regression.
-- Dependencies: NdArray, Linalg. No Vector3 or game API required.
FourierWindow = {}
local function finite(value)
    return type(value) == "number" and value == value and math.abs(value) < math.huge
end

function FourierWindow.new(size, interval, config)
    config = config or {}
    assert(finite(size) and size >= 4 and size == math.floor(size), "Fourier window size must be an integer >= 4")
    assert(finite(interval) and interval > 0, "Fourier window interval must be positive")
    local duration = size * interval
    local cycles = config.windowCycles or 1
    local minimum = config.windowMinHalfLife or duration/8
    local maximum = config.windowMaxHalfLife or duration
    assert(finite(cycles) and cycles > 0, "windowCycles must be positive")
    -- Bound the weight ratio to preserve the conditioning of the harmonic fit.
    assert(finite(minimum) and minimum >= duration/16, "windowMinHalfLife must be >= history duration/16")
    assert(finite(maximum) and maximum >= minimum, "windowMaxHalfLife must be >= windowMinHalfLife")
    return {size = size, interval = interval, cycles = cycles, minHalfLife = minimum, maxHalfLife = maximum}
end

function FourierWindow.halfLife(window, bin)
    assert(finite(bin) and bin >= 0 and bin <= math.floor(window.size/2)
        and bin == math.floor(bin), "Fourier window requires an in-range integer bin")
    if bin == 0 then return window.maxHalfLife end
    return math.max(window.minHalfLife, math.min(window.maxHalfLife,
        window.cycles * window.size * window.interval / bin))
end

function FourierWindow.weight(window, bin, age)
    assert(finite(age) and age >= 0, "Fourier window age must be finite and nonnegative")
    return 2^(-age/FourierWindow.halfLife(window, bin))
end

-- Fit increments at the latest grid sample: DC + A*cos(w*t) + B*sin(w*t).
-- Each requested bin gets its own weighted fit of the ENTIRE selected basis;
-- retain that bin's coefficients. This separates overlapping tapered bands.
-- All signals must be full, aligned StreamingFourier states. Results are fresh.
function FourierWindow.fit(window, signals, bins)
    assert(#signals > 0, "Fourier window requires at least one signal")
    for _, signal in ipairs(signals) do
        assert(signal.size == window.size and signal.count == window.size,
            "Fourier window requires full matching StreamingFourier histories")
        assert(signal.next == signals[1].next, "Fourier window histories must be aligned")
    end
    local columns, positions, seen = 1, {}, {}
    for i, bin in ipairs(bins) do
        FourierWindow.halfLife(window, bin)
        assert(bin > 0 and not seen[bin], "Fourier window bins must be distinct and positive")
        seen[bin] = true
        positions[i] = {cosine = columns+1}
        columns = columns+1
        if not (window.size % 2 == 0 and bin == window.size/2) then
            positions[i].sine = columns+1
            columns = columns+1
        end
    end
    local basis = {}
    for age = 0, window.size-1 do
        local row = {1}
        for i, bin in ipairs(bins) do
            local angle = -2*math.pi*bin*age/window.size
            row[positions[i].cosine] = math.cos(angle)
            if positions[i].sine then row[positions[i].sine] = math.sin(angle) end
        end
        basis[age+1] = row
    end
    local solutions = {}
    local function solve(bin)
        local halfLife = FourierWindow.halfLife(window, bin)
        if solutions[halfLife] then return solutions[halfLife] end
        local gram, rhs = NdArray.zeros({columns, columns}), NdArray.zeros({columns, #signals})
        for age = 0, window.size-1 do
            local row = basis[age+1]
            local weight = 2^(-age*window.interval/halfLife)
            for r = 1, columns do
                local weighted = weight*row[r]
                for c = 1, columns do
                    local offset = (r-1)*columns+c
                    gram.data[offset] = gram.data[offset] + weighted*row[c]
                end
                for s, signal in ipairs(signals) do
                    local index = (signal.next-2-age) % window.size+1
                    local offset = (r-1)*#signals+s
                    rhs.data[offset] = rhs.data[offset] + weighted*signal.samples[index]
                end
            end
        end
        -- Positive bounded weights and distinct Fourier columns give a full-rank
        -- Gram matrix. A broken contract fails here; there is no legacy fallback.
        local solution = Linalg.solveCholesky(Linalg.cholesky(gram), rhs)
        solutions[halfLife] = solution
        return solution
    end
    local function values(solution, row)
        local result = {}
        for s = 1, #signals do result[s] = row and solution.data[(row-1)*#signals+s] or 0 end
        return result
    end
    local result = {dc = values(solve(0), 1), components = {}}
    for i, bin in ipairs(bins) do
        local solution = solve(bin)
        result.components[i] = {bin = bin, halfLife = FourierWindow.halfLife(window, bin),
            cosine = values(solution, positions[i].cosine), sine = values(solution, positions[i].sine)}
    end
    return result
end
