do
    local endpoint = "component:10:0"
    local weapon_index = 0
    local component_type = 10
    local component_index = 0
    local shared_state = { owner = "component_writer", updates = 0 }

    register_script("component_writer", function(I, context)
        shared_state.updates = shared_state.updates + 1
        assert_equal(shared_state.owner, "component_writer")
        assert_equal(context.tick, shared_state.updates)

        assert_equal(I:Component_GetCount(component_type), 1)
        assert_equal(I:Component_GetBlockInfo(component_type, component_index).CustomName, "shared_shield")
        I:Component_SetBoolLogic(component_type, component_index, true)
        assert_true(I:Component_GetBoolLogic(component_type, component_index))

        context.state = shared_state
    end)
end
