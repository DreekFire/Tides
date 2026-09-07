-- Dependency-aware module scheduler used by the modular Counterbattery.
--
-- A module declares the products it provides and the products it consumes.
-- Enabled modules are topologically sorted once at construction time.  A
-- missing prerequisite and a dependency cycle are configuration errors, not
-- runtime ordering accidents.
local CounterbatteryModuleSystem = {}

local function cycleMessage(stack, name)
  local start = 1
  for index, value in ipairs(stack) do
    if value == name then start = index break end
  end
  local path = {}
  for index = start, #stack do path[#path + 1] = stack[index] end
  path[#path + 1] = name
  return "circular module dependency: " .. table.concat(path, " -> ")
end

local function normalizeDefinition(definition)
  assert(type(definition) == "table", "module definition must be a table")
  assert(type(definition.name) == "string" and definition.name ~= "",
    "module definition requires a name")
  assert(type(definition.update) == "function",
    "module " .. definition.name .. " requires an update function")
  return {
    name = definition.name,
    enabled = definition.enabled ~= false,
    requires = definition.requires or {},
    provides = definition.provides or {},
    update = definition.update,
  }
end

function CounterbatteryModuleSystem.new(definitions)
  local system = {
    definitions = {},
    modules = {},
    order = {},
  }
  local provider = {}

  for _, raw in ipairs(definitions or {}) do
    local definition = normalizeDefinition(raw)
    assert(not system.definitions[definition.name],
      "duplicate module name: " .. definition.name)
    system.definitions[definition.name] = definition
    if definition.enabled then
      system.modules[#system.modules + 1] = definition
      for _, output in ipairs(definition.provides) do
        assert(type(output) == "string" and output ~= "",
          "module " .. definition.name .. " has an invalid output")
        assert(not provider[output], "multiple enabled modules provide output: " .. output)
        provider[output] = definition.name
      end
    end
  end

  local state = {}
  local stack = {}
  local function visit(name)
    if state[name] == 2 then return end
    if state[name] == 1 then error(cycleMessage(stack, name), 0) end
    local definition = system.definitions[name]
    state[name] = 1
    stack[#stack + 1] = name
    for _, requirement in ipairs(definition.requires) do
      local dependency = provider[requirement]
      assert(dependency,
        "module " .. name .. " requires unavailable output: " .. requirement)
      visit(dependency)
    end
    stack[#stack] = nil
    state[name] = 2
    system.order[#system.order + 1] = definition
  end

  for _, definition in ipairs(system.modules) do visit(definition.name) end
  return setmetatable(system, {__index = CounterbatteryModuleSystem})
end

function CounterbatteryModuleSystem:update(context)
  context = context or {}
  context.outputs = context.outputs or {}
  for _, definition in ipairs(self.order) do
    local output = definition.update(context)
    if output ~= nil then
      assert(#definition.provides == 1,
        "module " .. definition.name .. " returned an output but provides multiple products")
      context.outputs[definition.provides[1]] = output
    end
  end
  return context.outputs
end

function CounterbatteryModuleSystem:isEnabled(name)
  return self.definitions[name] ~= nil and self.definitions[name].enabled
end

function CounterbatteryModuleSystem:orderedNames()
  local names = {}
  for _, definition in ipairs(self.order) do names[#names + 1] = definition.name end
  return names
end

_G.CounterbatteryModuleSystem = CounterbatteryModuleSystem
