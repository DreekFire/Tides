local function close(actual, expected)
    assert(math.abs(actual - expected) < 1e-8,
        tostring(actual) .. " differs from " .. tostring(expected))
end
for _, size in ipairs({1, 2, 7, 16, 31}) do
    local state = StreamingFourier.new(size)
    local history = {}
    local real, imag = state.real, state.imag
    for step = 1, 10000 do
        local sample = math.sin(step * 0.31) + math.cos(step * 0.7) + 0.25
        if step <= size then sample = step == 1 and 1 or 0 end
        history[#history + 1] = sample
        if #history > size then table.remove(history, 1) end
        assert(StreamingFourier.update(state, sample) == (step >= size))
        assert(state.real == real and state.imag == imag)
        if step < 2 * size or step % 101 == 0 then
            for k = 0, math.floor(size / 2) do
                local re, im = 0, 0
                for j = 0, size - 1 do
                    local value = history[j + 1 - (size - #history)] or 0
                    local angle = -2 * math.pi * k * j / size
                    re, im = re + value * math.cos(angle), im + value * math.sin(angle)
                end
                close(state.real[k + 1], re)
                close(state.imag[k + 1], im)
            end
        end
    end
    StreamingFourier.reset(state)
    assert(state.count == 0 and state.next == 1)
    for i = 1, #state.real do close(state.real[i], 0); close(state.imag[i], 0) end
    for i = 1, size do StreamingFourier.update(state, 2) end
    close(state.real[1], 2 * size)
    for i = 2, #state.real do close(state.real[i], 0); close(state.imag[i], 0) end
    for _, bad in ipairs({0/0, math.huge, -math.huge, '1', false}) do
        assert(not pcall(StreamingFourier.update, state, bad))
        close(state.real[1], 2 * size)
    end
end
for _, bad in ipairs({0, -1, 1.5, math.huge, 0/0, '8', false}) do
    assert(not pcall(StreamingFourier.new, bad))
end
local tone = StreamingFourier.new(16)
for j = 0, 15 do StreamingFourier.update(tone, math.sin(2 * math.pi * 3 * j / 16)) end
close(tone.real[4], 0)
close(tone.imag[4], -8)
print('StreamingFourier contracts passed')
