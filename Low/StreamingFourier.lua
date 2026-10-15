StreamingFourier = {}

-- Real, uniformly sampled input; rectangular window. Bins k=0..floor(N/2)
-- are stored at k+1, with unnormalized negative-exponent DFT coefficients.
-- During warmup the window is left-padded with zeros. Output tables are reused.
function StreamingFourier.new(size)
    assert(type(size) == "number" and size >= 1 and size < math.huge
        and size == math.floor(size), "window size must be a positive integer")
    local state = {size = size, count = 0, next = 1, samples = {},
        real = {}, imag = {}, cosine = {}, sine = {}}
    for i = 1, size do state.samples[i] = 0 end
    for k = 0, math.floor(size / 2) do
        local angle = 2 * math.pi * k / size
        state.cosine[k + 1] = math.cos(angle)
        state.sine[k + 1] = math.sin(angle)
        state.real[k + 1], state.imag[k + 1] = 0, 0
    end
    return state
end

-- O(N) time per sample and O(N) storage. Read real/imag immediately or copy
-- them before the next update. Returns true once a complete window is present.
function StreamingFourier.update(state, sample)
    assert(type(sample) == "number" and sample == sample
        and math.abs(sample) < math.huge, "sample must be a finite real number")
    local delta = sample - state.samples[state.next]
    state.samples[state.next] = sample
    state.next = state.next % state.size + 1
    state.count = math.min(state.count + 1, state.size)
    for i = 1, #state.real do
        local re, im = state.real[i] + delta, state.imag[i]
        local c, s = state.cosine[i], state.sine[i]
        state.real[i], state.imag[i] = c * re - s * im, s * re + c * im
    end
    -- These bins are strictly real; suppress accumulated roundoff leakage.
    state.imag[1] = 0
    if state.size % 2 == 0 then state.imag[#state.imag] = 0 end
    return state.count == state.size
end

-- Start a new independent stream while retaining allocations and twiddles.
-- As with recursive transforms, roundoff accumulates over very long streams.
function StreamingFourier.reset(state)
    for i = 1, state.size do state.samples[i] = 0 end
    for i = 1, #state.real do state.real[i], state.imag[i] = 0, 0 end
    state.next, state.count = 1, 0
end
