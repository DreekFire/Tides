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
