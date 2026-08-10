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
