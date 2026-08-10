-- paste modules here
local function MyModule(IModule)
    -- The build script must emit this local before the pasted child script.
    -- It keeps the child's ordinary global `function Update(I)` syntax local
    -- to this module wrapper when _ALE/fenv is unavailable.
    local Update
    testGlobal = "MyModule"
    local testLocal = "MyModule"
    -- paste script here
    function Update(I)
        I:Log("Hi, I'm a module!")
        I:Log("MyModule test global: "..testGlobal)
        I:Log("MyModule test local: "..testLocal)
    end

    while true do
        Update(IModule)
        IModule = coroutine.yield()
    end
end


local function ThyModule(IModule)
    -- The build script must emit a fresh local for every pasted child script.
    local Update
    -- paste script here
    testGlobal = "ThyModule"
    local testLocal = "ThyModule"
    function Update(I)
        I:Log("Wow, another module?")
        I:Log("ThyModule test global: "..testGlobal)
        I:Log("ThyModule test local: "..testLocal)
    end

    while true do
        Update(IModule)
        IModule = coroutine.yield()
    end
end


-- Settings
-- Each redundant Lua box should have a unique, positive luaIdx (i.e. 1, 2, 3, ...). They will come online if all lower index Lua boxes are destroyed.
local luaIdx = 1
-- Whether to stop running if any module encounters an error
local stopOnError = true
-- Whether to stop running if two modules try to control the same thing
local stopOnConflict = false
local modules = {
    {name="MyModule", func=MyModule, retry=0, forceClearLogs=false},
    {name="ThyModule", func=ThyModule, retry=0, forceClearLogs=false},
}

-- Miscellaneous Variables
local coroutines = {}
local tries = {}
-- local logs = {}
local inited = false

-- module conflict debugging
local out = {
    propulsion={},
    components={},
    missiles={},
    weapons={},
    subobjects={},
}

local currentRunning = ""
local stopped = false
-- local currentLogs
local IOverride
local setOverrides
local checkConflicts
local clearConflicts

local Util = {}
local StringUtil = {}
testGlobal = "main"
local testLocal = "main"


local function Init(I)
    for name, _ in pairs(out) do
        out[name] = Util.defaultdict(function() return {} end, true)
    end

    local global
    if _ALE then
        global = getfenv(1)
    end
    for i, mod in ipairs(modules) do
        if global then
            local env = {}
            setmetatable(env, { __index = global })
            setfenv(mod.func, env)
        end
        local name = mod.name
        coroutines[i] = coroutine.create(mod.func)
        tries[name] = mod.retry + 1
        -- logs[name] = {}
    end

    IOverride = setOverrides(I)
    currentRunning = 'ModuleManager'
end

function Update(I)
    if stopped then return end
    if not inited then
        Init(I)
        inited = true
    end
    -- Redundant Lua system. Prevents this box from running until higher priority boxes have all been destroyed.

    IOverride:RequestCustomAxis("luaActive", 2 ^ (-luaIdx))
    if IOverride:GetCustomAxis("luaActive") > 2 ^ (1 - luaIdx) then     
        return
    end
    for i, co in ipairs(coroutines) do
        local mod = modules[i]
        local name = mod.name
        if tries[name] <= 0 then
            IOverride:Log('skipping module: '..name)
        elseif tries[name] <= mod.retry then
            IOverride:Log('retrying module: '..name)
        end
        if tries[name] > 0 then
            currentRunning = name
            IOverride:Log('running module: '..name)
            local success, msg = coroutine.resume(co, IOverride, coroutine)
            currentRunning = 'ModuleManager'
            if not success then
                if stopOnError then stopped = true end
                IOverride:Log(name..' encountered an error: '..msg)
                tries[name] = tries[name] - 1
                if tries[name] > 0 then
                    coroutines[i] = coroutine.create(mod.func)
                end
            elseif msg ~= nil then
                IOverride:Log(name..': '..msg)
            end
        end
    end
    I:Log("main test global: "..testGlobal)
    I:Log("main test local: "..testLocal)
    if checkConflicts(I, out) and stopOnConflict then
        stopped = true
    end
    clearConflicts()
end

function setOverrides(J)
    local stimIdx = 14
    local moveFortress = 29
    local overrides = {
        -- Log = function(I, msg)
        --     currentLogs[#currentLogs] = msg
        -- end,
        TellAiThatWeAreTakingControl = function(I, ret)
            I:Log(string.format("Module %s is disabling AI control", currentRunning))
        end,
        -- all propulsion requests
        AddPropulsionRequest = function(I, ret, type, value)
            out.propulsion[type][currentRunning] = true
        end,
        RequestComplexControllerStimulus = function(I, ret, stim)
            out.propulsion[stim + stimIdx][currentRunning] = true
        end,
        MoveFortress = function(I, ret, ...)
            out.propulsion[moveFortress][currentRunning] = true
        end,
        -- missile control
        GetMissileInfo = function(I, ret, transceiver_idx, missile_idx)
            for i, part in ipairs(ret.Parts) do
                local sendRegister = part.SendRegister
                part.SendRegister = function(index, value)
                    out.missiles[transceiver_idx][missile_idx][currentRunning] = true
                    sendRegister(index, value)
                end
            end
        end,
        -- weapon control
        AimWeaponInDirection = function(I, ret, weapon_idx, ...)
            out.weapons.main[weapon_idx][currentRunning] = true
        end,
        AimWeaponInDirectionOnSubConstruct = function(I, ret, subconstruct_uid, weapon_idx, ...)
            out.weapons[subconstruct_uid][weapon_idx][currentRunning] = true
        end,
        FireWeapon = function(I, ret, weapon_idx, ...)
            out.weapons.main[weapon_idx][currentRunning] = true
        end,
        FireWeaponOnSubConstruct = function(I, ret, subconstruct_uid, weapon_idx, ...)
            out.weapons[subconstruct_uid][weapon_idx][currentRunning] = true
        end,
        -- subobject control
        SetSpin = function(I, ret, subconstruct_uid, ...)
            out.subobjects[subconstruct_uid][currentRunning] = true
        end,
    }
    overrides.SetPiston = overrides.SetSpin
    overrides.SetPropulsionRequest = overrides.AddPropulsionRequest
    overrides.RequestCustomAxis = overrides.AddPropulsionRequest

    local matchOverrides = {
        {
            prefix='SetLua', suffix='', func=function(I, ret, transceiver_idx, missile_idx, ...)
                out.missiles[transceiver_idx][missile_idx][currentRunning] = true
            end,
        },
        {
            prefix='Component_Set', suffix='All', func=function(I, ret, type, ...)
                out.components[type].all[currentRunning] = true
            end,
        },
        {
            prefix='Component_Set', suffix='', func=function(I, ret, type, block_idx, ...)
                out.components[type][block_idx][currentRunning] = true
            end,
        },
    }

    local newI = {}
    for k, v in pairs(getmetatable(J)) do
        local override = nil
        if type(v) == "function"and type(k) == "string" and not k:match("^__") then
            local orig = v
            if overrides[k] then
                override = overrides[k]
            else
                for i, over in ipairs(matchOverrides) do
                    if k:sub(1, #over.prefix) == over.prefix and k:sub(#k - #over.suffix + 1, #k) == over.suffix then
                        override = over.func
                        break
                    end
                end
            end
            if override then
                newI[k] = function(_, ...)
                    local ret = orig(J, ...)
                    override(J, ret, ...)
                    return ret
                end
            else
                newI[k] = function(_, ...)
                    return orig(J, ...)
                end
            end
        end
    end

    return setmetatable(newI, { __index = J })
end

function checkConflicts(I, tab, prefix)
    if not prefix then prefix = '' end
    local conflict = false
    local conflicting = {}
    for k, v in pairs(tab) do
        if type(v) == "table" then
            local childConflict = checkConflicts(I, v, prefix..'/'..k)
            conflict = childConflict or conflict
        else
            conflicting[#conflicting + 1] = v
        end
    end
    if #conflicting > 1 then
        I:Log(string.format("Multiple modules attempting to write to %s: %s",
            prefix,
            StringUtil.Join(conflicting, ', ')))
        return true
    end
    return conflict
end

function clearConflicts()
    for name, _ in pairs(out) do
        out[name] = Util.defaultdict(function() return {} end, true)
    end
end


function Util.defaultdict(factory, nested)
    local meta = {}
    meta.__index = function(t, k)
        local v = factory()
        if nested then
            setmetatable(v, meta)
        end
        rawset(t, k, v)
        return v
    end
    return setmetatable(factory(), meta)
end

function StringUtil.Join(tab, delimiter, start, stop)
  start = start or 1
  stop = stop or #tab
  delimiter = delimiter or ','
  if stop - start > 8 then
    local l = StringUtil.Join(tab, delimiter, start, math.floor(stop * 0.5))
    local r = StringUtil.Join(tab, delimiter, math.floor(stop * 0.5) + 1, stop)
    return l..delimiter..r
  end
  local str = ''
  for i=start, stop do
    str = str..(tostring(tab[i])..delimiter)
  end
  return str
end
