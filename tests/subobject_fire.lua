do
    local endpoint = "weapon:subconstruct:202:0"
    local weapon_index = 0
    local component_type = 10
    local component_index = 0
    local shared_state = { owner = "subobject_fire", updates = 0 }

    register_script("subobject_fire", function(I, context)
        shared_state.updates = shared_state.updates + 1
        assert_equal(shared_state.owner, "subobject_fire")
        assert_equal(context.tick, shared_state.updates)

        assert_equal(I:GetSubConstructInfo(202).CustomName, "shared_subobject")
        assert_equal(I:GetWeaponInfoOnSubConstruct(202, weapon_index).CustomName, "shared_subobject_weapon")
        I:FireWeaponOnSubConstruct(202, weapon_index, 0)

        context.state = shared_state
    end)
end
