do
    local endpoint = "weapon:subconstruct:101:0"
    local weapon_index = 0
    local component_type = 10
    local component_index = 0
    local shared_state = { owner = "turret_aim", updates = 0 }

    register_script("turret_aim", function(I, context)
        shared_state.updates = shared_state.updates + 1
        assert_equal(shared_state.owner, "turret_aim")
        assert_equal(context.tick, shared_state.updates)

        local subconstructs = I:GetAllSubConstructs()
        assert_equal(#subconstructs, 2)
        assert_equal(I:GetSubConstructInfo(101).CustomName, "shared_turret")
        assert_true(I:GetSubConstructInfo(101).IsTurret, "expected a turret subconstruct")
        assert_equal(I:GetWeaponCountOnSubConstruct(101), 1)
        assert_equal(I:GetWeaponInfoOnSubConstruct(101, weapon_index).WeaponType, "CIWS")
        I:AimWeaponInDirectionOnSubConstruct(101, weapon_index, 0, 1, 0, 0)

        context.state = shared_state
    end)
end
