local test_interface, test_contexts = run_registered_scripts(2)

for _, name in ipairs({
    "turret_aim", "turret_fire", "subobject_aim", "subobject_fire",
    "hull_weapon", "component_writer", "component_reader"
}) do
    assert_equal(test_contexts[name].ticks, 2, "script did not run twice: " .. name)
    assert_equal(test_contexts[name].state.owner, name, "local state leaked into " .. name)
end

test_interface:assert_access("weapon:subconstruct:101:0", { "turret_aim", "turret_fire" }, { "turret_aim", "turret_fire" })
test_interface:assert_access("weapon:subconstruct:202:0", { "subobject_aim", "subobject_fire" }, { "subobject_aim", "subobject_fire" })
test_interface:assert_access("weapon:hull:0", { "hull_weapon" }, { "hull_weapon" })
test_interface:assert_access("component:10:0", { "component_writer", "component_reader" }, { "component_writer" })

if _ALE ~= nil then
    assert_true(type(setfenv) == "function", "ALE mode must provide setfenv")
    assert_true(type(getfenv) == "function", "ALE mode must provide getfenv")

    local function make_global_script(value)
        local environment = { expected_value = value }
        setmetatable(environment, { __index = _G })
        local function script()
            shared_global_probe = expected_value
            return shared_global_probe
        end
        setfenv(script, environment)
        return script, environment
    end

    local first_script, first_environment = make_global_script("first")
    local second_script, second_environment = make_global_script("second")
    assert_equal(first_script(), "first")
    assert_equal(second_script(), "second")
    assert_equal(getfenv(first_script).shared_global_probe, "first")
    assert_equal(getfenv(second_script).shared_global_probe, "second")
    assert_true(first_environment.shared_global_probe ~= second_environment.shared_global_probe,
        "ALE global contexts shadowed one another")
    assert_true(_G.shared_global_probe == nil, "ALE global probe escaped its script context")
end

print("PASS: isolated multi-script FtD endpoint contract")
