do
    local endpoint = "weapon:subconstruct:101:0"
    local weapon_index = 0
    local component_type = 10
    local component_index = 0
    local shared_state = { owner = "turret_fire", updates = 0 }

    register_script("turret_fire", function(I, context)
        shared_state.updates = shared_state.updates + 1
        assert_equal(shared_state.owner, "turret_fire")
        assert_equal(context.tick, shared_state.updates)

        assert_equal(I:GetSubConstructInfo(101).CustomName, "shared_turret")
        assert_equal(I:GetWeaponInfoOnSubConstruct(101, weapon_index).CustomName, "shared_turret_weapon")
        I:FireWeaponOnSubConstruct(101, weapon_index, 0)

        context.state = shared_state
    end)
end
