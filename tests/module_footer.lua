local test_interface = MockInterface.new()

Update(test_interface)
Update(test_interface)

local function count_calls(method)
    local count = 0
    for _, actual in ipairs(test_interface.calls) do
        if actual == method then
            count = count + 1
        end
    end
    return count
end

assert_equal(count_calls("FireWeapon"), 2, "FireWeapon was not called once per tick")
assert_equal(count_calls("FireWeaponOnSubConstruct"), 4,
    "FireWeaponOnSubConstruct was not called for both shared subconstruct weapons")
assert_equal(count_calls("AimWeaponInDirection"), 2, "AimWeaponInDirection was not called once per tick")
assert_equal(count_calls("AimWeaponInDirectionOnSubConstruct"), 4,
    "AimWeaponInDirectionOnSubConstruct was not called for both shared subconstruct weapons")

local function assert_log_contains(fragment)
    for _, message in ipairs(test_interface.logs) do
        if string.find(message, fragment, 1, true) then
            return
        end
    end
    error("missing manager conflict log containing: " .. fragment, 2)
end

assert_log_contains("/weapons/main/0")
assert_log_contains("/weapons/101/0")
assert_log_contains("/weapons/202/0")
assert_log_contains("/components/10/0")

print("PASS: ModuleExample isolated child-script integration")
