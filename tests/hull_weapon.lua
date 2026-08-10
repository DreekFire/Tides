do
    local endpoint = "weapon:hull:0"
    local weapon_index = 0
    local component_type = 10
    local component_index = 0
    local shared_state = { owner = "hull_weapon", updates = 0 }

    register_script("hull_weapon", function(I, context)
        shared_state.updates = shared_state.updates + 1
        assert_equal(shared_state.owner, "hull_weapon")
        assert_equal(context.tick, shared_state.updates)

        assert_equal(I:GetWeaponCount(), 1)
        assert_equal(I:GetWeaponInfo(weapon_index).CustomName, "shared_hull_weapon")
        I:AimWeaponInDirection(weapon_index, 1, 0, 0, 0)
        I:FireWeapon(weapon_index, 0)

        context.state = shared_state
    end)
end
