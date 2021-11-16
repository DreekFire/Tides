-- dependencies: LinkedList
local Accumulator = {}
-- Accumulation is the discrete time equivalent to integration

--[[
  Arguments:
    window - the time period over which to accumulate values
    decay - the rate at which values decay. Values are multiplied
            by decay^t, where t is their age in seconds.
            lower decay means value decays faster, giving recent
            values more importance.
  Returns:
    acc - an accumulator struct
]]--
function Accumulator.Accumulator(window, decay)
  local acc = {}
  acc.decay = decay
  acc.window = window
  acc.time = 0
  acc.weight = 0
  if window > 0 then
    acc.vals = LinkedList.LinkedList()
    acc.times = LinkedList.LinkedList()
  end
  return acc
end

--[[
  Updates the accumulator by giving it a new value,
  and returns the state of the accumulator.

  Arguments:
    acc - the accumulator struct to update
    value - the value to accumulate
    time - the time over which the value is observed.
           essentially weights the value.
           set to 1 for unweighted average.
  Returns:
    value - the total accumulated value
    weight - the total time, affected by decay.
             value / weight will give an average
             of the values accumulated.
]]--
function Accumulator.update(acc, value, time)
  local decayFac = Mathf.Pow(acc.decay, time)
  if not acc.value then
    -- instead of setting it to some initial value like 0
    -- initializing here lets us be flexible with the type of value
    -- for example, accumulate vectors instead of real numbers
    acc.value = value * time
  else
    acc.value = acc.value * decayFac
    acc.value = acc.value + value * time
  end
  acc.time = acc.time + time
  acc.weight = acc.weight * decayFac
  acc.weight = acc.weight + time
  if acc.window > 0 then
    LinkedList.pushFront(acc.vals, value)
    LinkedList.pushFront(acc.times, time)
    while acc.time > acc.window do
      local t = LinkedList.popBack(acc.times)
      acc.time = acc.time - t
      local d = Mathf.Pow(acc.decay, acc.time)
      acc.weight = acc.weight - t * d
      acc.value = acc.value - LinkedList.popBack(acc.vals) * t * d
    end
  end
  return acc.value, acc.weight
end

--[[
  Returns the state of the accumulator without
  giving it a new value. Does not change the state
  of the accumulator.

  Arguments:
    acc - the accumulator struct to update
  Returns:
    value - the total accumulated value
    weight - the total time, affected by decay.
             value / weight will give an average
             of the values accumulated.
]]--
function Accumulator.get(acc)
  return acc.value, acc.weight
end
