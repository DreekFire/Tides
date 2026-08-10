local test_interface = MockInterface.new()

local function count_calls(method)
    local count = 0
    for _, actual in ipairs(test_interface.calls) do
        if actual == method then
            count = count + 1
        end
    end
    return count
end

local function has_log(fragment)
    for _, message in ipairs(test_interface.logs) do
        if string.find(message, fragment, 1, true) then
            return true
        end
    end
    return false
end

local function assert_log_contains(fragment)
    assert_true(has_log(fragment), "missing manager conflict log containing: " .. fragment)
end

local function assert_no_conflict_logs()
    for _, message in ipairs(test_interface.logs) do
        assert_true(not string.find(message, "Multiple modules attempting to write", 1, true),
            "conflict warning was emitted in silent/off mode")
    end
end

local function registry_empty(registry)
    for _, value in pairs(registry) do
        if type(value) ~= "table" or not registry_empty(value) then
            return false
        end
    end
    return true
end

if _TEST_MODULE_ERROR then
    Update(test_interface)
    assert_true(stopped, "stopOnError did not stop after a child exception")
    local calls_after_error = #test_interface.calls
    Update(test_interface)
    assert_equal(#test_interface.calls, calls_after_error,
        "stopped manager continued after a child exception")
    assert_true(has_log("MyModule encountered an error"))
    print("PASS: ModuleExample child exception handling")
else
    Update(test_interface)
    Update(test_interface)

    local expected_ticks = 2
    if conflictMode == "warnOnConflict" and stopOnConflict then
        expected_ticks = 1
    end

    assert_equal(count_calls("FireWeapon"), expected_ticks,
        "FireWeapon was not called once per active tick")
    assert_equal(count_calls("FireWeaponOnSubConstruct"), expected_ticks * 2,
        "FireWeaponOnSubConstruct was not called for both shared subconstruct weapons")
    assert_equal(count_calls("AimWeaponInDirection"), expected_ticks,
        "AimWeaponInDirection was not called once per active tick")
    assert_equal(count_calls("AimWeaponInDirectionOnSubConstruct"), expected_ticks * 2,
        "AimWeaponInDirectionOnSubConstruct was not called for both shared subconstruct weapons")
    assert_true(has_log("is disabling AI control"),
        "TellAiThatWeAreTakingControl override did not run")

    if conflictMode == "warnOnConflict" then
        assert_log_contains("/propulsion/1")
        assert_log_contains("/propulsion/14")
        assert_log_contains("/propulsion/29")
        assert_log_contains("/propulsion/2")
        assert_log_contains("/propulsion/childAxis")
        assert_log_contains("/missiles/0/0")
        assert_log_contains("/weapons/main/0")
        assert_log_contains("/weapons/101/0")
        assert_log_contains("/weapons/202/0")
        assert_log_contains("/components/10/all")
        assert_log_contains("/components/10/0")
        assert_log_contains("/subobjects/202")
    else
        assert_no_conflict_logs()
    end

    if conflictMode == "silent" or conflictMode == "off" then
        assert_true(not stopped, conflictMode .. " mode unexpectedly stopped the manager")
    end
    assert_true(registry_empty(out), "writer registry was not cleared after the tick")
    assert_true(registry_empty(readers), "reader registry was not cleared after the tick")

    if _ALE ~= nil then
        assert_true(type(setfenv) == "function", "ALE mode must provide setfenv")
        assert_true(type(getfenv) == "function", "ALE mode must provide getfenv")
        for _, mod in ipairs(modules) do
            assert_equal(getfenv(mod.func).module_global_probe, mod.name,
                "ALE global state leaked between module contexts")
        end
        assert_true(_G.module_global_probe == nil,
            "ALE global probe escaped its module context")
    end

    print("PASS: ModuleExample reader/writer registries and conflict modes")
end
