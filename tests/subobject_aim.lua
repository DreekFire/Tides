do
    local endpoint = "weapon:subconstruct:202:0"
    local weapon_index = 0
    local component_type = 10
    local component_index = 0
    local shared_state = { owner = "subobject_aim", updates = 0 }

    register_script("subobject_aim", function(I, context)
        shared_state.updates = shared_state.updates + 1
        assert_equal(shared_state.owner, "subobject_aim")
        assert_equal(context.tick, shared_state.updates)

        assert_equal(I:GetSubConstructInfo(202).CustomName, "shared_subobject")
        assert_true(not I:GetSubConstructInfo(202).IsTurret, "expected a non-turret subobject")
        assert_equal(I:GetWeaponCountOnSubConstruct(202), 1)
        assert_equal(I:GetWeaponInfoOnSubConstruct(202, weapon_index).WeaponType, "Laser")
        I:AimWeaponInDirectionOnSubConstruct(202, weapon_index, 0, 0, 1, 0)

        context.state = shared_state
    end)
end
