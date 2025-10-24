local coroutines = {}
local skip = {}
-- list your modules in Init, not here
local modules = {}
local inited = false

-- module conflict debugging
local axesUsed = {}
local currentRunning = ""
local overrides = {
    Log = function(I, msg)
        I:Log(string.format("%s: %s", currentRunning, msg))
    end
    TellAiThatWeAreTakingControl = function(I)
        I:Log(string.format("Module %s is disabling AI control", currentRunning))
        I:TellAiThatWeAreTakingControl()
    end,
    -- all propulsion requests
    RequestControl = function(I, mode, type, drive)
        if axesUsed[type] == nil then
            axesUsed[type] = {currentRunning}
        else
            axesUsed[type][#axesUsed[type] + 1] = currentRunning
        end
        I:RequestControl(mode, type, drive)
    end,
    -- component setting
    -- weapon setting
    -- subobject setting
}


function Init(I)
    modules = {
        -- list your modules here, not above
        {name="MyModule", func=MyModule, retry=false}
    }

    for i, mod in ipairs(modules) do
        coroutines[i] = coroutine.create(mod.func)
    end
end

function Update(I)
    if not inited then
        Init(I)
        inited = true
    end
    for i, co in ipairs(coroutines) do
        local mod = modules[i]
        local name = mod.name
        if not skip[name] then
            local success, msg = coroutine.resume(co, I)
            if not success then
                I:Log(name..' encountered an error.')
                if mod.retry then
                    co = coroutine.create(mod.func)
                    skip[name] = false
                else
                    skip[name] = true
                end
            else if msg ~= nil then
                I:Log(name..': '..msg)
            end
        end
    end
    if next(skip) ~= nil then
        I:Log("Modules had errors")
        for name, s in pairs(skip) do
            if s then
                I:Log(name..": skipped")
            else
                I:Log(name..": retried")
            end
        end
    end
end

function MyModule(I)
    -- paste script here

    while true do
        Update(I)
        coroutine.yield()
    end
end