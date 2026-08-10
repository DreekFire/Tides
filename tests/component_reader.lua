do
    local endpoint = "component:10:0"
    local weapon_index = 0
    local component_type = 10
    local component_index = 0
    local shared_state = { owner = "component_reader", updates = 0 }

    register_script("component_reader", function(I, context)
        shared_state.updates = shared_state.updates + 1
        assert_equal(shared_state.owner, "component_reader")
        assert_equal(context.tick, shared_state.updates)

        assert_equal(I:Component_GetBlockInfo(component_type, component_index).CustomName, "shared_shield")
        local enabled = I:Component_GetBoolLogic(component_type, component_index)
        assert_true(type(enabled) == "boolean", "component bool endpoint returned a non-boolean value")

        context.state = shared_state
    end)
end
