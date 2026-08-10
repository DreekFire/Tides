-- Test double for the FtD interface used by ModuleExample.lua.

function assert_true(condition, message)
    if not condition then
        error(message or "assertion failed", 2)
    end
end

function assert_equal(actual, expected, message)
    if actual ~= expected then
        error((message or "values differ") .. ": expected " .. tostring(expected) .. ", got " .. tostring(actual), 2)
    end
end

local MockInterface = {}
MockInterface.__index = MockInterface

function MockInterface.new()
    return setmetatable({
        custom_axes = {},
        hull_weapons = {
            [0] = { CustomName = "shared_hull_weapon", WeaponType = "SimpleWeapon" },
        },
        subconstructs = {
            [101] = { CustomName = "shared_turret", IsTurret = true },
            [202] = { CustomName = "shared_subobject", IsTurret = false },
        },
        subconstruct_weapons = {
            [101] = { [0] = { CustomName = "shared_turret_weapon", WeaponType = "CIWS" } },
            [202] = { [0] = { CustomName = "shared_subobject_weapon", WeaponType = "Laser" } },
        },
        components = {
            [10] = { [0] = { CustomName = "shared_shield", enabled = false } },
        },
        calls = {},
        logs = {},
    }, MockInterface)
end

function MockInterface:record_call(method)
    self.calls[#self.calls + 1] = method
end

function MockInterface:Log(message)
    self:record_call("Log")
    self.logs[#self.logs + 1] = message
end

function MockInterface:RequestCustomAxis(name, value)
    self:record_call("RequestCustomAxis")
    self.custom_axes[name] = value
end

function MockInterface:GetCustomAxis(name)
    self:record_call("GetCustomAxis")
    return self.custom_axes[name] or 0
end

function MockInterface:GetAllSubConstructs()
    self:record_call("GetAllSubConstructs")
    return { 101, 202 }
end

function MockInterface:GetSubConstructInfo(identifier)
    self:record_call("GetSubConstructInfo")
    return self.subconstructs[identifier]
end

function MockInterface:GetWeaponCount()
    self:record_call("GetWeaponCount")
    return 1
end

function MockInterface:GetWeaponInfo(index)
    self:record_call("GetWeaponInfo")
    return self.hull_weapons[index]
end

function MockInterface:GetWeaponCountOnSubConstruct(identifier)
    self:record_call("GetWeaponCountOnSubConstruct")
    return self.subconstruct_weapons[identifier] and 1 or 0
end

function MockInterface:GetWeaponInfoOnSubConstruct(identifier, index)
    self:record_call("GetWeaponInfoOnSubConstruct")
    return self.subconstruct_weapons[identifier][index]
end

function MockInterface:AimWeaponInDirection(index, x, y, z, slot)
    self:record_call("AimWeaponInDirection")
end

function MockInterface:FireWeapon(index, slot)
    self:record_call("FireWeapon")
end

function MockInterface:AimWeaponInDirectionOnSubConstruct(identifier, index, x, y, z, slot)
    self:record_call("AimWeaponInDirectionOnSubConstruct")
end

function MockInterface:FireWeaponOnSubConstruct(identifier, index, slot)
    self:record_call("FireWeaponOnSubConstruct")
end

function MockInterface:Component_GetCount(component_type)
    self:record_call("Component_GetCount")
    return self.components[component_type] and 1 or 0
end

function MockInterface:Component_GetBlockInfo(component_type, index)
    self:record_call("Component_GetBlockInfo")
    return self.components[component_type][index]
end

function MockInterface:Component_GetBoolLogic(component_type, index)
    self:record_call("Component_GetBoolLogic")
    return self.components[component_type][index].enabled
end

function MockInterface:Component_SetBoolLogic(component_type, index, value)
    self:record_call("Component_SetBoolLogic")
    self.components[component_type][index].enabled = value
end


-- BEGIN MODULE EXAMPLE
-- paste modules here
local function MyModule(IModule)
    -- The build script must emit this local before the pasted child script.
    -- It keeps the child's ordinary global `function Update(I)` syntax local
    -- to this module wrapper when _ALE/fenv is unavailable.
    local Update
    testGlobal = "MyModule"
    local testLocal = "MyModule"
    -- BEGIN PASTED CHILD: module_my.lua
-- Ordinary FtD child script pasted into MyModule.

local script_name = "MyModule"
local ticks = 0
local weapon_index = 0
local turret_uid = 101
local subobject_uid = 202
local component_type = 10
local component_index = 0

function Update(I)
    ticks = ticks + 1
    assert_equal(script_name, "MyModule")
    assert_equal(I:GetWeaponCount(), 1)
    assert_equal(I:GetWeaponInfo(weapon_index).CustomName, "shared_hull_weapon")
    assert_equal(I:GetSubConstructInfo(turret_uid).CustomName, "shared_turret")
    assert_true(I:GetSubConstructInfo(turret_uid).IsTurret)
    assert_equal(I:GetSubConstructInfo(subobject_uid).CustomName, "shared_subobject")
    assert_true(not I:GetSubConstructInfo(subobject_uid).IsTurret)
    assert_equal(I:GetWeaponCountOnSubConstruct(turret_uid), 1)
    assert_equal(I:GetWeaponCountOnSubConstruct(subobject_uid), 1)
    assert_equal(I:GetWeaponInfoOnSubConstruct(turret_uid, weapon_index).WeaponType, "CIWS")
    assert_equal(I:GetWeaponInfoOnSubConstruct(subobject_uid, weapon_index).WeaponType, "Laser")
    assert_equal(I:Component_GetCount(component_type), 1)
    assert_equal(I:Component_GetBlockInfo(component_type, component_index).CustomName, "shared_shield")
    assert_true(type(I:Component_GetBoolLogic(component_type, component_index)) == "boolean")

    I:AimWeaponInDirection(weapon_index, 1, 0, 0, 0)
    I:AimWeaponInDirectionOnSubConstruct(turret_uid, weapon_index, 0, 1, 0, 0)
    I:FireWeaponOnSubConstruct(subobject_uid, weapon_index, 0)
    I:Component_SetBoolLogic(component_type, component_index, true)
    assert_true(ticks <= 2, "module-local tick state was not isolated")
end
    -- END PASTED CHILD: module_my.lua
    while true do
        Update(IModule)
        IModule = coroutine.yield()
    end
end


local function ThyModule(IModule)
    -- The build script must emit a fresh local for every pasted child script.
    local Update
    -- BEGIN PASTED CHILD: module_thy.lua
-- Ordinary FtD child script pasted into ThyModule.

local script_name = "ThyModule"
local ticks = 0
local weapon_index = 0
local turret_uid = 101
local subobject_uid = 202
local component_type = 10
local component_index = 0

function Update(I)
    ticks = ticks + 1
    assert_equal(script_name, "ThyModule")
    assert_equal(I:GetWeaponCount(), 1)
    assert_equal(I:GetWeaponInfo(weapon_index).CustomName, "shared_hull_weapon")
    assert_equal(I:GetSubConstructInfo(turret_uid).CustomName, "shared_turret")
    assert_true(I:GetSubConstructInfo(turret_uid).IsTurret)
    assert_equal(I:GetSubConstructInfo(subobject_uid).CustomName, "shared_subobject")
    assert_true(not I:GetSubConstructInfo(subobject_uid).IsTurret)
    assert_equal(I:GetWeaponCountOnSubConstruct(turret_uid), 1)
    assert_equal(I:GetWeaponCountOnSubConstruct(subobject_uid), 1)
    assert_equal(I:GetWeaponInfoOnSubConstruct(turret_uid, weapon_index).WeaponType, "CIWS")
    assert_equal(I:GetWeaponInfoOnSubConstruct(subobject_uid, weapon_index).WeaponType, "Laser")
    assert_equal(I:Component_GetCount(component_type), 1)
    assert_equal(I:Component_GetBlockInfo(component_type, component_index).CustomName, "shared_shield")
    assert_true(type(I:Component_GetBoolLogic(component_type, component_index)) == "boolean")

    I:FireWeapon(weapon_index, 0)
    I:FireWeaponOnSubConstruct(turret_uid, weapon_index, 0)
    I:AimWeaponInDirectionOnSubConstruct(subobject_uid, weapon_index, 0, 0, 1, 0)
    I:Component_SetBoolLogic(component_type, component_index, false)
    assert_true(ticks <= 2, "module-local tick state was not isolated")
end
    -- END PASTED CHILD: module_thy.lua
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


-- BEGIN MODULE TEST ASSERTIONS
local test_interface = MockInterface.new()

Update(test_interface)
Update(test_interface)

local function count_calls(method)
    local count = 0
    for _, actual in ipairs(test_interface.calls) do
        if actual == method then
            count = count + 1
        end
    end
    return count
end

assert_equal(count_calls("FireWeapon"), 2, "FireWeapon was not called once per tick")
assert_equal(count_calls("FireWeaponOnSubConstruct"), 4,
    "FireWeaponOnSubConstruct was not called for both shared subconstruct weapons")
assert_equal(count_calls("AimWeaponInDirection"), 2, "AimWeaponInDirection was not called once per tick")
assert_equal(count_calls("AimWeaponInDirectionOnSubConstruct"), 4,
    "AimWeaponInDirectionOnSubConstruct was not called for both shared subconstruct weapons")

local function assert_log_contains(fragment)
    for _, message in ipairs(test_interface.logs) do
        if string.find(message, fragment, 1, true) then
            return
        end
    end
    error("missing manager conflict log containing: " .. fragment, 2)
end

assert_log_contains("/weapons/main/0")
assert_log_contains("/weapons/101/0")
assert_log_contains("/weapons/202/0")
assert_log_contains("/components/10/0")

print("PASS: ModuleExample isolated child-script integration")
