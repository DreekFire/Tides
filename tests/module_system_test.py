#!/usr/bin/env python3
"""Regression tests for Counterbattery's output-based module scheduler."""

from pathlib import Path

from lupa import LuaRuntime


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "Full" / "Counterbattery" / "Pipeline" / "ModuleSystem.lua"


def main() -> None:
    lua = LuaRuntime(unpack_returned_tuples=True)
    system = lua.execute(SOURCE.read_text(encoding="utf-8") + "\nreturn CounterbatteryModuleSystem")

    defs = lua.table_from([
        lua.table_from({"name": "consumer", "requires": lua.table_from(["a"]),
                        "provides": lua.table_from(["b"]),
                        "update": lua.eval("function(c) return c.outputs.a + 1 end")}),
        lua.table_from({"name": "producer", "provides": lua.table_from(["a"]),
                        "update": lua.eval("function(c) return 41 end")}),
    ])
    instance = system.new(defs)
    names = instance.orderedNames(instance)
    assert [names[index] for index in range(1, 3)] == ["producer", "consumer"]
    outputs = instance.update(instance, lua.table_from())
    assert outputs["b"] == 42

    def assert_error(definitions, fragment):
        try:
            system.new(lua.table_from(definitions))
        except Exception as error:
            assert fragment in str(error), (fragment, error)
        else:
            raise AssertionError("expected scheduler construction to fail")

    assert_error([
        lua.table_from({"name": "consumer", "requires": lua.table_from(["missing"]),
                        "update": lua.eval("function() end")}),
    ], "requires unavailable output")
    assert_error([
        lua.table_from({"name": "a", "requires": lua.table_from(["b"]),
                        "provides": lua.table_from(["a-out"]), "update": lua.eval("function() end")}),
        lua.table_from({"name": "b", "requires": lua.table_from(["a-out"]),
                        "provides": lua.table_from(["b"]), "update": lua.eval("function() end")}),
    ], "circular module dependency")
    print("Counterbattery module scheduler: ok")


if __name__ == "__main__":
    main()
