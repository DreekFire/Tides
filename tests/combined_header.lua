-- Test harness header. The build script places each child test after this file.
-- This is deliberately a test double, not the Tides manager implementation.

local unpack_args = table.unpack or unpack

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

local registered_scripts = {}

function register_script(name, callback)
    assert_true(type(name) == "string", "script names must be strings")
    assert_true(type(callback) == "function", "scripts must register a callback")
    registered_scripts[#registered_scripts + 1] = { name = name, callback = callback }
end

local MockInterface = {}
MockInterface.__index = MockInterface

function MockInterface.new()
    return setmetatable({
        hull_weapons = {
            [0] = { CustomName = "shared_hull_weapon", WeaponType = "SimpleWeapon" }
        },
        subconstructs = {
            [101] = { CustomName = "shared_turret", IsTurret = true },
            [202] = { CustomName = "shared_subobject", IsTurret = false }
        },
        subconstruct_weapons = {
            [101] = {
                [0] = { CustomName = "shared_turret_weapon", WeaponType = "CIWS" }
            },
            [202] = {
                [0] = { CustomName = "shared_subobject_weapon", WeaponType = "Laser" }
            }
        },
        components = {
            [10] = {
                [0] = { CustomName = "shared_shield", enabled = false }
            }
        },
        access = {},
        calls = {}
    }, MockInterface)
end

function MockInterface:record_call(method)
    self.calls[#self.calls + 1] = method
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

function MockInterface:AimWeaponInDirection(index, x, y, z, slot)
    self:record_call("AimWeaponInDirection")
    self.last_aim = { index = index, x = x, y = y, z = z, slot = slot }
end

function MockInterface:FireWeapon(index, slot)
    self:record_call("FireWeapon")
    self.last_fire = { index = index, slot = slot }
end

function MockInterface:GetWeaponCountOnSubConstruct(identifier)
    self:record_call("GetWeaponCountOnSubConstruct")
    local weapons = self.subconstruct_weapons[identifier]
    return weapons and 1 or 0
end

function MockInterface:GetWeaponInfoOnSubConstruct(identifier, index)
    self:record_call("GetWeaponInfoOnSubConstruct")
    return self.subconstruct_weapons[identifier][index]
end

function MockInterface:AimWeaponInDirectionOnSubConstruct(identifier, index, x, y, z, slot)
    self:record_call("AimWeaponInDirectionOnSubConstruct")
    self.last_subconstruct_aim = {
        identifier = identifier, index = index, x = x, y = y, z = z, slot = slot
    }
end

function MockInterface:FireWeaponOnSubConstruct(identifier, index, slot)
    self:record_call("FireWeaponOnSubConstruct")
    self.last_subconstruct_fire = { identifier = identifier, index = index, slot = slot }
end

function MockInterface:Component_GetCount(component_type)
    self:record_call("Component_GetCount")
    local components = self.components[component_type]
    return components and 1 or 0
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

function MockInterface:Log(message)
    self:record_call("Log")
    self.last_log = message
end

local function add_unique(list, value)
    for _, existing in ipairs(list) do
        if existing == value then
            return
        end
    end
    list[#list + 1] = value
end

function MockInterface:record_access(script_name, operation, endpoint)
    local item = self.access[endpoint]
    if not item then
        item = { readers = {}, writers = {} }
        self.access[endpoint] = item
    end
    add_unique(item[operation == "read" and "readers" or "writers"], script_name)
end

function MockInterface:assert_access(endpoint, expected_readers, expected_writers)
    local item = self.access[endpoint]
    assert_true(item ~= nil, "endpoint was never accessed: " .. endpoint)
    assert_equal(#item.readers, #expected_readers, "reader count for " .. endpoint)
    assert_equal(#item.writers, #expected_writers, "writer count for " .. endpoint)
    for _, name in ipairs(expected_readers) do
        local found = false
        for _, actual in ipairs(item.readers) do
            found = found or actual == name
        end
        assert_true(found, "missing reader " .. name .. " for " .. endpoint)
    end
    for _, name in ipairs(expected_writers) do
        local found = false
        for _, actual in ipairs(item.writers) do
            found = found or actual == name
        end
        assert_true(found, "missing writer " .. name .. " for " .. endpoint)
    end
end

local function endpoint_for(method, args)
    if method == "GetAllSubConstructs" then
        return "subconstructs", "read"
    elseif method == "GetSubConstructInfo" then
        return "subconstruct:" .. tostring(args[1]), "read"
    elseif method == "GetWeaponCount" or method == "GetWeaponInfo" then
        return "weapon:hull:" .. tostring(args[1] or "all"), "read"
    elseif method == "AimWeaponInDirection" or method == "FireWeapon" then
        return "weapon:hull:" .. tostring(args[1]), "write"
    elseif method == "GetWeaponCountOnSubConstruct" then
        return "weapon:subconstruct:" .. tostring(args[1]) .. ":all", "read"
    elseif method == "GetWeaponInfoOnSubConstruct" then
        return "weapon:subconstruct:" .. tostring(args[1]) .. ":" .. tostring(args[2]), "read"
    elseif method == "AimWeaponInDirectionOnSubConstruct" or method == "FireWeaponOnSubConstruct" then
        return "weapon:subconstruct:" .. tostring(args[1]) .. ":" .. tostring(args[2]), "write"
    elseif method == "Component_GetCount" then
        return "component:" .. tostring(args[1]) .. ":all", "read"
    elseif method == "Component_GetBlockInfo" or method == "Component_GetBoolLogic" or method == "Component_SetBoolLogic" then
        return "component:" .. tostring(args[1]) .. ":" .. tostring(args[2]), method == "Component_SetBoolLogic" and "write" or "read"
    end
    return nil, nil
end

local function interface_for(host, script_name)
    return setmetatable({}, {
        __index = function(_, method)
            local original = host[method]
            if type(original) ~= "function" then
                return original
            end
            return function(_, ...)
                local args = { ... }
                local endpoint, operation = endpoint_for(method, args)
                if endpoint then
                    host:record_access(script_name, operation, endpoint)
                end
                return original(host, unpack_args(args))
            end
        end
    })
end

local function run_registered_scripts(tick_count)
    local host = MockInterface.new()
    local contexts = {}
    for _, entry in ipairs(registered_scripts) do
        contexts[entry.name] = { name = entry.name, ticks = 0 }
    end
    for tick = 1, tick_count do
        for _, entry in ipairs(registered_scripts) do
            local context = contexts[entry.name]
            context.tick = tick
            context.ticks = context.ticks + 1
            entry.callback(interface_for(host, entry.name), context)
        end
    end
    return host, contexts
end
