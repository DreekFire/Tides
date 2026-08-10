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
        missile_parts = {
            {
                SendRegister = function(_, index, value)
                    -- The manager replaces this method to track missile writes.
                end,
            },
        },
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

function MockInterface:TellAiThatWeAreTakingControl()
    self:record_call("TellAiThatWeAreTakingControl")
end

function MockInterface:AddPropulsionRequest(request_type, value)
    self:record_call("AddPropulsionRequest")
end

function MockInterface:RequestComplexControllerStimulus(stimulus)
    self:record_call("RequestComplexControllerStimulus")
end

function MockInterface:MoveFortress(x, y, z)
    self:record_call("MoveFortress")
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

function MockInterface:GetMissileInfo(transceiver_index, missile_index)
    self:record_call("GetMissileInfo")
    return { Parts = self.missile_parts }
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

function MockInterface:SetSpin(identifier, speed)
    self:record_call("SetSpin")
end

function MockInterface:SetPiston(identifier, speed)
    self:record_call("SetPiston")
end

function MockInterface:SetPropulsionRequest(request_type, value)
    self:record_call("SetPropulsionRequest")
end

function MockInterface:SetLuaMain(transceiver_index, missile_index, value)
    self:record_call("SetLuaMain")
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

function MockInterface:Component_SetBoolLogicAll(component_type, value)
    self:record_call("Component_SetBoolLogicAll")
end



-- BEGIN MODULE EXAMPLE
-- paste modules here
-- Conflict reporting mode is intentionally declared before the pasted module
-- bodies so test children can inspect the active registry during a tick.
local conflictMode = "warnOnConflict"
local out = {
    propulsion={},
    components={},
    missiles={},
    weapons={},
    subobjects={},
}
local readers = {
    propulsion={},
    components={},
    missiles={},
    weapons={},
    subobjects={},
}

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

if _ALE then
    module_global_probe = script_name
end

function Update(I)
    if _TEST_MODULE_ERROR then
        error("intentional module failure")
    end
    ticks = ticks + 1
    assert_equal(script_name, "MyModule")
    assert_true(type(I:GetCustomAxis("childAxis")) == "number")
    assert_equal(#I:GetAllSubConstructs(), 2)
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
    local missile = I:GetMissileInfo(0, 0)

    I:TellAiThatWeAreTakingControl()
    I:AddPropulsionRequest(1, 0.5)
    I:RequestComplexControllerStimulus(0, 1)
    I:MoveFortress(1, 2, 3)
    I:SetPropulsionRequest(2, 0.25)
    I:RequestCustomAxis("childAxis", 0.5)
    I:SetSpin(subobject_uid, 1)
    I:SetPiston(subobject_uid, 2)
    I:SetLuaMain(0, 0, true)
    missile.Parts[1].SendRegister(1, true)
    I:AimWeaponInDirection(weapon_index, 1, 0, 0, 0)
    I:AimWeaponInDirectionOnSubConstruct(turret_uid, weapon_index, 0, 1, 0, 0)
    I:FireWeaponOnSubConstruct(subobject_uid, weapon_index, 0)
    I:Component_SetBoolLogic(component_type, component_index, true)
    I:Component_SetBoolLogicAll(component_type, true)

    if conflictMode == "off" then
        assert_true(next(out.propulsion) == nil)
        assert_true(next(readers.weapons) == nil)
    else
        assert_true(out.propulsion[1][script_name])
        assert_true(out.propulsion[14][script_name])
        assert_true(out.propulsion[29][script_name])
        assert_true(out.propulsion[2][script_name])
        assert_true(out.propulsion.childAxis[script_name])
        assert_true(readers.propulsion.childAxis[script_name])
        assert_true(out.missiles[0][0][script_name])
        assert_true(readers.missiles[0][0][script_name])
        assert_true(out.weapons.main[weapon_index][script_name])
        assert_true(out.weapons[turret_uid][weapon_index][script_name])
        assert_true(out.weapons[subobject_uid][weapon_index][script_name])
        assert_true(readers.weapons.main.all[script_name])
        assert_true(readers.weapons.main[weapon_index][script_name])
        assert_true(readers.weapons[turret_uid].all[script_name])
        assert_true(readers.weapons[turret_uid][weapon_index][script_name])
        assert_true(readers.subobjects.all[script_name])
        assert_true(readers.subobjects[turret_uid][script_name])
        assert_true(readers.components[component_type].all[script_name])
        assert_true(readers.components[component_type][component_index][script_name])
        assert_true(out.components[component_type].all[script_name])
        assert_true(out.components[component_type][component_index][script_name])
    end
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

if _ALE then
    module_global_probe = script_name
end

function Update(I)
    if _TEST_MODULE_ERROR then
        error("intentional module failure")
    end
    ticks = ticks + 1
    assert_equal(script_name, "ThyModule")
    assert_true(type(I:GetCustomAxis("childAxis")) == "number")
    assert_equal(#I:GetAllSubConstructs(), 2)
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
    local missile = I:GetMissileInfo(0, 0)

    I:TellAiThatWeAreTakingControl()
    I:AddPropulsionRequest(1, 0.5)
    I:RequestComplexControllerStimulus(0, 1)
    I:MoveFortress(1, 2, 3)
    I:SetPropulsionRequest(2, 0.25)
    I:RequestCustomAxis("childAxis", 0.5)
    I:SetSpin(subobject_uid, 1)
    I:SetPiston(subobject_uid, 2)
    I:SetLuaMain(0, 0, true)
    missile.Parts[1].SendRegister(1, true)
    I:FireWeapon(weapon_index, 0)
    I:FireWeaponOnSubConstruct(turret_uid, weapon_index, 0)
    I:AimWeaponInDirectionOnSubConstruct(subobject_uid, weapon_index, 0, 0, 1, 0)
    I:Component_SetBoolLogic(component_type, component_index, false)
    I:Component_SetBoolLogicAll(component_type, false)

    if conflictMode == "off" then
        assert_true(next(out.propulsion) == nil)
        assert_true(next(readers.weapons) == nil)
    else
        assert_true(out.propulsion[1][script_name])
        assert_true(out.propulsion[14][script_name])
        assert_true(out.propulsion[29][script_name])
        assert_true(out.propulsion[2][script_name])
        assert_true(out.propulsion.childAxis[script_name])
        assert_true(readers.propulsion.childAxis[script_name])
        assert_true(out.missiles[0][0][script_name])
        assert_true(readers.missiles[0][0][script_name])
        assert_true(out.weapons.main[weapon_index][script_name])
        assert_true(out.weapons[turret_uid][weapon_index][script_name])
        assert_true(out.weapons[subobject_uid][weapon_index][script_name])
        assert_true(readers.weapons.main.all[script_name])
        assert_true(readers.weapons.main[weapon_index][script_name])
        assert_true(readers.weapons[turret_uid].all[script_name])
        assert_true(readers.weapons[turret_uid][weapon_index][script_name])
        assert_true(readers.subobjects.all[script_name])
        assert_true(readers.subobjects[turret_uid][script_name])
        assert_true(readers.components[component_type].all[script_name])
        assert_true(readers.components[component_type][component_index][script_name])
        assert_true(out.components[component_type].all[script_name])
        assert_true(out.components[component_type][component_index][script_name])
    end
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
-- Whether a reported conflict should stop running modules.
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

local function track(registry, ...)
    if conflictMode == "off" then return end
    local node = registry
    for i = 1, select("#", ...) do
        node = node[select(i, ...)]
    end
    node[currentRunning] = true
end


local function Init(I)
    for name, _ in pairs(out) do
        out[name] = Util.defaultdict(function() return {} end, true)
        readers[name] = Util.defaultdict(function() return {} end, true)
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
    local conflict = conflictMode ~= "off" and checkConflicts(I, out)
    if conflict and conflictMode == "warnOnConflict" and stopOnConflict then
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
            track(out, "propulsion", type)
        end,
        RequestComplexControllerStimulus = function(I, ret, stim)
            track(out, "propulsion", stim + stimIdx)
        end,
        MoveFortress = function(I, ret, ...)
            track(out, "propulsion", moveFortress)
        end,
        GetCustomAxis = function(I, ret, name)
            track(readers, "propulsion", name)
        end,
        GetAllSubConstructs = function(I, ret)
            track(readers, "subobjects", "all")
        end,
        GetSubConstructInfo = function(I, ret, subconstruct_uid)
            track(readers, "subobjects", subconstruct_uid)
        end,
        GetWeaponCount = function(I, ret)
            track(readers, "weapons", "main", "all")
        end,
        GetWeaponInfo = function(I, ret, weapon_idx)
            track(readers, "weapons", "main", weapon_idx)
        end,
        GetWeaponCountOnSubConstruct = function(I, ret, subconstruct_uid)
            track(readers, "weapons", subconstruct_uid, "all")
        end,
        GetWeaponInfoOnSubConstruct = function(I, ret, subconstruct_uid, weapon_idx)
            track(readers, "weapons", subconstruct_uid, weapon_idx)
        end,
        Component_GetCount = function(I, ret, type)
            track(readers, "components", type, "all")
        end,
        Component_GetBlockInfo = function(I, ret, type, block_idx)
            track(readers, "components", type, block_idx)
        end,
        Component_GetBoolLogic = function(I, ret, type, block_idx)
            track(readers, "components", type, block_idx)
        end,
        -- missile control
        GetMissileInfo = function(I, ret, transceiver_idx, missile_idx)
            track(readers, "missiles", transceiver_idx, missile_idx)
            for i, part in ipairs(ret.Parts) do
                local sendRegister = part.SendRegister
                part.SendRegister = function(index, value)
                    track(out, "missiles", transceiver_idx, missile_idx)
                    sendRegister(index, value)
                end
            end
        end,
        -- weapon control
        AimWeaponInDirection = function(I, ret, weapon_idx, ...)
            track(out, "weapons", "main", weapon_idx)
        end,
        AimWeaponInDirectionOnSubConstruct = function(I, ret, subconstruct_uid, weapon_idx, ...)
            track(out, "weapons", subconstruct_uid, weapon_idx)
        end,
        FireWeapon = function(I, ret, weapon_idx, ...)
            track(out, "weapons", "main", weapon_idx)
        end,
        FireWeaponOnSubConstruct = function(I, ret, subconstruct_uid, weapon_idx, ...)
            track(out, "weapons", subconstruct_uid, weapon_idx)
        end,
        -- subobject control
        SetSpin = function(I, ret, subconstruct_uid, ...)
            track(out, "subobjects", subconstruct_uid)
        end,
    }
    overrides.SetPiston = overrides.SetSpin
    overrides.SetPropulsionRequest = overrides.AddPropulsionRequest
    overrides.RequestCustomAxis = overrides.AddPropulsionRequest

    local matchOverrides = {
        {
            prefix='SetLua', suffix='', func=function(I, ret, transceiver_idx, missile_idx, ...)
                track(out, "missiles", transceiver_idx, missile_idx)
            end,
        },
        {
            prefix='Component_Set', suffix='All', func=function(I, ret, type, ...)
                track(out, "components", type, "all")
            end,
        },
        {
            prefix='Component_Set', suffix='', func=function(I, ret, type, block_idx, ...)
                track(out, "components", type, block_idx)
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
        if conflictMode == "warnOnConflict" then
            I:Log(string.format("Multiple modules attempting to write to %s: %s",
                prefix,
                StringUtil.Join(conflicting, ', ')))
        end
        return true
    end
    return conflict
end

function clearConflicts()
    for name, _ in pairs(out) do
        out[name] = Util.defaultdict(function() return {} end, true)
        readers[name] = Util.defaultdict(function() return {} end, true)
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

local function count_calls(method)
    local count = 0
    for _, actual in ipairs(test_interface.calls) do
        if actual == method then
            count = count + 1
        end
    end
    return count
end

local function has_log(fragment)
    for _, message in ipairs(test_interface.logs) do
        if string.find(message, fragment, 1, true) then
            return true
        end
    end
    return false
end

local function assert_log_contains(fragment)
    assert_true(has_log(fragment), "missing manager conflict log containing: " .. fragment)
end

local function assert_no_conflict_logs()
    for _, message in ipairs(test_interface.logs) do
        assert_true(not string.find(message, "Multiple modules attempting to write", 1, true),
            "conflict warning was emitted in silent/off mode")
    end
end

local function registry_empty(registry)
    for _, value in pairs(registry) do
        if type(value) ~= "table" or not registry_empty(value) then
            return false
        end
    end
    return true
end

if _TEST_MODULE_ERROR then
    Update(test_interface)
    assert_true(stopped, "stopOnError did not stop after a child exception")
    local calls_after_error = #test_interface.calls
    Update(test_interface)
    assert_equal(#test_interface.calls, calls_after_error,
        "stopped manager continued after a child exception")
    assert_true(has_log("MyModule encountered an error"))
    print("PASS: ModuleExample child exception handling")
else
    Update(test_interface)
    Update(test_interface)

    local expected_ticks = 2
    if conflictMode == "warnOnConflict" and stopOnConflict then
        expected_ticks = 1
    end

    assert_equal(count_calls("FireWeapon"), expected_ticks,
        "FireWeapon was not called once per active tick")
    assert_equal(count_calls("FireWeaponOnSubConstruct"), expected_ticks * 2,
        "FireWeaponOnSubConstruct was not called for both shared subconstruct weapons")
    assert_equal(count_calls("AimWeaponInDirection"), expected_ticks,
        "AimWeaponInDirection was not called once per active tick")
    assert_equal(count_calls("AimWeaponInDirectionOnSubConstruct"), expected_ticks * 2,
        "AimWeaponInDirectionOnSubConstruct was not called for both shared subconstruct weapons")
    assert_true(has_log("is disabling AI control"),
        "TellAiThatWeAreTakingControl override did not run")

    if conflictMode == "warnOnConflict" then
        assert_log_contains("/propulsion/1")
        assert_log_contains("/propulsion/14")
        assert_log_contains("/propulsion/29")
        assert_log_contains("/propulsion/2")
        assert_log_contains("/propulsion/childAxis")
        assert_log_contains("/missiles/0/0")
        assert_log_contains("/weapons/main/0")
        assert_log_contains("/weapons/101/0")
        assert_log_contains("/weapons/202/0")
        assert_log_contains("/components/10/all")
        assert_log_contains("/components/10/0")
        assert_log_contains("/subobjects/202")
    else
        assert_no_conflict_logs()
    end

    if conflictMode == "silent" or conflictMode == "off" then
        assert_true(not stopped, conflictMode .. " mode unexpectedly stopped the manager")
    end
    assert_true(registry_empty(out), "writer registry was not cleared after the tick")
    assert_true(registry_empty(readers), "reader registry was not cleared after the tick")

    if _ALE ~= nil then
        assert_true(type(setfenv) == "function", "ALE mode must provide setfenv")
        assert_true(type(getfenv) == "function", "ALE mode must provide getfenv")
        for _, mod in ipairs(modules) do
            assert_equal(getfenv(mod.func).module_global_probe, mod.name,
                "ALE global state leaked between module contexts")
        end
        assert_true(_G.module_global_probe == nil,
            "ALE global probe escaped its module context")
    end

    print("PASS: ModuleExample reader/writer registries and conflict modes")
end
