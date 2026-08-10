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
