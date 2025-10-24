-- dependencies: LinkedList
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

-- Differ is short for differencer and is the discrete time equivalent of taking the derivative

--[[
  Arguments:
    initial - the initial value of the differ
  Returns:
    differ - a differ struct
]]--
function Differ.Differ(initial)
  local differ = {}
  differ.lastVal = initial
  differ.diff = nil
  return differ
end

--[[
  Updates the differ by giving it a new value,
  and returns the change between the new value
  and the old value stored by the differ.

  Arguments:
    differ - the differ struct to update
    value - the new value
  Returns:
    diff - value minus the previous value
           stored by the differ
]]--
function Differ.update(differ, value)
  if differ.lastVal then
    differ.diff = value - differ.lastVal
    differ.lastVal = value
  end
  differ.lastVal = value
  return differ.diff
end

--[[
  Returns the state of the differ without
  giving it a new value. Does not change the state
  of the differ.

  Arguments:
    differ - the differ struct to update
  Returns:
    diff - the previous change observed
           by the differ
]]--
function Differ.get(differ)
  return differ.diff
end

--[[
WIP
function Graph.Graph()
  local g = {
    verts = {}
  }
  return g
end
]]--

-- A priority queue is a data structure which retrieves the smallest (or largest) value stored
-- A heap priority queue is a commonly used efficient implementation of a priority queue
-- These functions implement a min-heap, which retrieves the smallest value
-- If you need the largest one, you can just store the negative of the values or provide a custom comparator

function Heapq.Heapq(initial, comparator)
  local heapq = {}
  heapq.data = initial
  heapq.comp = comparator or function(a, b)
    return a < b
  end
  local n = #heapq.data
  heapq.size = n
  for idx = math.floor(n / 2), 1, -1 do
    Heapq.siftDown(heapq, idx)
  end
  return heapq
end

function Heapq.siftDown(heapq, position)
  local heaped = false
  local s = position
  local n = #heapq.data
  while not heaped do
    heaped = true
    local left = 2 * s
    local right = 2 * s + 1
    local argmin = s
    if left <= n and heapq.comp(heapq.data[left], heapq.data[argmin]) then
      argmin = left
      heaped = false
    end
    if right <= n and heapq.comp(heapq.data[right], heapq.data[argmin]) then
      argmin = right
      heaped = false
    end
    if not heaped then
      local swap = heapq.data[argmin]
      heapq.data[argmin] = heapq.data[s]
      heapq.data[s] = swap
      s = argmin
    end
  end
end

function Heapq.siftUp(heapq, position)
  local heaped = false
  local s = position
  while not heaped do
    heaped = true
    local parent = math.floor(s / 2)
    if heapq.comp(heapq.data[s], heapq.data[parent]) then
      local swap = heapq.data[parent]
      heapq.data[parent] = heapq.data[s]
      heapq.data[s] = swap
      s = parent
      heaped = false
    end
  end
end

function Heapq.insert(heapq, item)
  heapq.data[heapq.size + 1] = item
  heapq.size = heapq.size + 1
  Heapq.siftUp(heapq, heapq.size)
end

function Heapq.pop(heapq)
  local res = heapq.data[1]
  heapq.data[1] = heapq.data[heapq.size]
  heapq.data[heapq.size] = nil
  heapq.size = heapq.size - 1
  Heapq.siftDown(heapq, 1)
  return res
end

function Heapq.peek(heapq)
  return heapq.data[1]
end

function Heapq.size(heapq)
  return heapq.size
end

-- A LinkedList is a chain of elements where each element contains a reference to the next one in the list.
-- This is a doubly linked list so each element also contains a reference to the previous element.

-- Creates a new linked list.
function LinkedList.LinkedList()
  local llelem = {}
  llelem.value = nil
  llelem.next = llelem
  llelem.prev = llelem
  return llelem
end

-- Adds value to the front of lst.
function LinkedList.pushFront(lst, value)
  local llelem = {}
  llelem.value = value
  LinkedList.connect(llelem, lst.next)
  LinkedList.connect(lst, llelem)
end

-- Adds value to the back of lst.
function LinkedList.pushBack(lst, value)
  local llelem = {}
  llelem.value = value
  LinkedList.connect(lst.prev, llelem)
  LinkedList.connect(llelem, lst)
end

-- Removes and returns the value at the front of lst.
-- Returns nil if empty.
function LinkedList.popFront(lst)
  local val = lst.next.value
  LinkedList.connect(lst, lst.next.next)
  return val
end

-- Removes and returns the value at the back of lst.
-- Returns nil if empty.
function LinkedList.popBack(lst)
  local val = lst.prev.value
  LinkedList.connect(lst.prev.prev, lst)
  return val
end

-- Returns the value at the front of lst. Does not remove it.
-- Returns nil if empty.
function LinkedList.peekFront(lst)
  return lst.next.val
end

-- Returns the value at the back of lst. Does not remove it.
-- Returns nil if empty.
function LinkedList.peekBack(lst)
  return lst.prev.val
end

-- Joins two list elements. Internal use, you probably don't need this.
function LinkedList.connect(e1, e2)
  e1.next = e2
  e2.prev = e1
end

-- Converts a linked list into a lua list (a table with numeric keys).
function LinkedList.toArray(lst)
  local i = 1
  local arr = {}
  local llelem = lst.next
  while llelem ~= lst do
    arr[i] = llelem.value
    llelem = llelem.next
  end
  return arr
end

--[[
  Arguments:
    from - a vector
    to - another vector
    planeNormal - the normal vector to a plane
  Returns:
    signedAngle - the angle between the projections
                  of from and to onto the plane
                  normal to planeNormal, where
                  clockwise (looking from the
                  direction of planeNormal) is
                  positive.
                  Differs from Vector3.SignedAngle
                  because that function does not
                  project the vectors onto the plane.
--]]
function MathUtil.angleOnPlane(from, to, planeNormal)
  local f = Vector3.ProjectOnPlane(from, planeNormal)
  local t = Vector3.ProjectOnPlane(to, planeNormal)
  return Vector3.SignedAngle(f, t, planeNormal)
end

--[[
  Arguments:
    set - an iterator
    comp (optional) - a comparison function which
                        takes 2 arguments and
                        returns true if the first
                        argument is less than the
                        second and false otherwise.
                        Uses < operator if empty
  Returns:
    min - the minimum value in set
            if there are multiple
            then it returns the first
--]]
function MathUtil.min(set, comp)
  local min = nil
  comp = comp or function(a, b) return a < b end
  for ele in set do
    if not min or comp(ele, min) then
      min = ele
    end
  end
  return min
end

--[[
  Same as min but returns the maximum
--]]
function MathUtil.max(set, comp)
  local max = nil
  comp = comp or function(a, b) return a < b end
  for ele in set do
    if not max or comp(max, ele) then
      max = ele
    end
  end
  return max
end

--[[
  This function comes in 3 variants
  Returns:
    range(stop):
      equivalent to range(0, stop, 1)

    range(start, stop):
      equivalent to range(start, stop, 1)

    range(start, stop, step):
      iter - an iterator that goes from start to
              stop - 1 in increments of step
--]]
function MathUtil.range(a, b, c)
  local start, stop = a, b
  local step
  if not a then return end
  if not b then
    start = 0
    stop = a
    step = start < stop and 1 or -1
  elseif c then
    step = c
  end
  return function(_, last)
    local next = last + step
    if next == stop then return nil end
    return next
  end, nil, start - step
end

--[[
  From MHebes' answer to StackOverflow question 35572435
  Returns a shuffled copy of lst without modifying lst.
  lst must not have holes (nil values followed by non-nil).
  Arguments:
    lst - list to shuffle
    inplace - whether to shuffle lst or create a copy
  Returns:
    s - shuffled list
--]]
function MathUtil.shuffle(lst, inplace)
  local s = inplace and lst or {}
  if not inplace then
    for i = 1, #lst do s[i] = lst[i] end
  end
  for i = #lst, 2, -1 do
    local j = math.random(i)
    s[i], s[j] = s[j], s[i]
  end
  return s
end

function MathUtil.combine(a, b, fun)
  if #a == #b then
    local res = {}
    for k, v in pairs(a) do
      res[k] = fun(k, v, b[k])
    end
    return res
  end
end

--[[
  Law of Cosines: a^2 = b^2 + c^2 - 2bc * cos(A)
  Law of Sines: a / sin(A) = b / sin(B) = c / sin(C)

  a, b, c are lengths of sides in a triangle,
  A, B, C are the angles opposite the corresponding side

  All angles are in degrees.
--]]

--[[
  Arguments:
    a, b, c - side lengths of a triangle
  Returns:
    A, B, C - the angles opposite sides a, b, and c, respectively
                nil if the triangle is invalid
--]]
function MathUtil.angleSSS(a, b, c)
  if (a + b < c) or (a + c < b) or (b + c < a) then return nil end
  local A = math.deg(math.acos((b * b + c * c - a * a) / (2 * b * c)))
  local B, C = MathUtil.angleSAS(b, A, c)
  return A, B ,C
end

--[[
  Arguments:
    a - the length of a side
    C - the included angle
    b - the length of another side
  Returns:
    c - the length of the third side
--]]
function MathUtil.sideSAS(a, C, b)
  local csq = a * a + b * b - 2 * a * b * math.cos(math.rad(C))
  return math.sqrt(csq)
end

--[[
  Arguments:
    a - the length of a side
    C - the included angle
    b - the length of another side
  Returns:
    A, B - the angles opposite sides a and b, respectively
            nil if undefined (a == b and C == 0)
--]]
function MathUtil.angleSAS(a, C, b)
  local c = MathUtil.sideSAS(a, C, b)
  if MathUtil.isZero(c) then return nil end
  local A, B
  if a < b then
    A = MathUtil.angleLoSin(c, a, C)
    B = 180 - A - C
  else
    B = MathUtil.angleLoSin(c, b, C)
    A = 180 - B - C
  end
  return A, B
end

--[[
  Arguments:
    a - the length of a side
    b - the length of another side
    A - the angle opposite side a
  Returns:
    c1, c2 - both possibilities for the length of the third side.
              one or both may be nil. c1 may be negative
--]]
function MathUtil.sideSSA(a, b, A)
  local q0 = b * b - a * a
  local q1 = -2 * b * math.cos(math.rad(A))
  local c1, c2 = MathUtil.solveQuadratic(1, q1, q0)
  if not c2 then return c1, c2 end
  if c1 < c2 then return c1, c2 end
  return c2, c1
end

--[[
  Arguments:
    a - the length of a side
    b - the length of another side
    A - the angle opposite side a
  Returns:
    B1, C1 - the first possibility for the second and third angle
    B2, C2 - the second possibility for the second and third angle
--]]
function MathUtil.angleSSA(a, b, A)
  local c1, c2 = MathUtil.sideSSA(a, b, A)
  if not c1 then return nil end
  local B1, C1 = MathUtil.angleSAS(b, A, c1)
  if not c2 then return B1, C1 end
  local B2, C2 = MathUtil.angleSAS(b, A, c2)
  return B1, C1, B2, C2
end

--[[
  Arguments:
    A - an angle in the triangle
    B - another angle in the triangle
    a - the side opposite angle A
  Returns:
    B1, C1 - the first possibility for the second and third side
    B2, C2 - the second possibility for the second and third side
--]]
function MathUtil.sideAAS(A, B, a)
  local C = 180 - A - B
  local b = MathUtil.sideLoSin(A, B, a)
  local c = MathUtil.sideLoSin(A, C, a)
  return b, c
end

-- there is no angleAAS because you can just subtract A and B from 180

--[[
  Arguments:
    a - one side in the triangle
    A - the angle opposite side a
    B - another angle in the triangle
  Returns:
    b - the side opposite angle B
--]]
function MathUtil.sideLoSin(a, A, B)
  return a * math.sin(math.rad(B)) / math.sin(math.rad(A))
end

--[[
  Arguments:
    a - one side in the triangle
    b - another side in the triangle
    A - the angle opposite side a
  Returns:
    B - the angle opposite side b
]]
function MathUtil.angleLoSin(a, b, A)
  return math.deg(math.asin(b * math.sin(math.rad(A)) / a))
end

--[[
  Arguments:
    v1 - the vector which defines the axis of the cone
    v2 - the vector to clamp
    angle - the angle of the cone in degrees
  Returns:
    v - the vector within angle of v1 with the same
        magnitude as v1 which is closest to v2
]]--
function MathUtil.clampCone(v1, v2, angle)
  local offAngle = math.min(angle, Vector3.Angle(v1, v2))
  local axis = Vector3.Cross(v1, v2)
  return Quaternion.AngleAxis(offAngle, axis) * v1
end

--[[
  Arguments:
    fn - the function to find the roots of
           must be scalar valued and take one scalar input
           if it returns nil (i.e. undefined), iteration will stop
    df - the derivative of fn. If the derivative is unknown,
           leave this blank to use a secant approximation
           if it returns nil (i.e. undefined), iteration will stop
    guess - the initial guess to start iterating at
    eps - when two consecutive iterations give solutions that
            differ by less than eps, finish iteration
            default 1e-5
    iterlimit - maximum number of iterations to run
                  default 100
  Returns:
    guess - the value obtained by iteration
    exceed - whether the maximum number of iterations was exceeded
]]
function MathUtil.newton(fn, df, guess, eps, iterlimit, dx)
  eps = eps or 1e-5
  dx = dx or (10 * eps)
  iterlimit = iterlimit or 25
  df = df or function(x) return (fn(x + dx) - fn(x)) / dx end
  guess = guess or 0
  local change = eps + 1
  local iters = 0
  while change > eps and iters < iterlimit do
    local current = fn(guess)
    local derivative = df(guess)
    if not current or not derivative then return nil end
    change = -current / derivative
    guess = guess + change
    iters = iters + 1
  end
  if iters < iterlimit then
    return guess, false
  end
  return guess, true
end

--[[
  formula from wikipedia
  Arguments:
    fn - the function to find zeros of. Must take a scalar
      input and return a scalar
    a - the lower bound to search
    b - the upper bound to search
      fn(a) and fn(b) must differ in sign, i.e.
      fn(a) * fn(b) <= 0
    eps - the maximum allowed error |x_itp - x*| where x*
      is the true root and x_itp is our output
    iterlimit - the maximum iterations to run
      guaranteed at most n_1/2 + n0 iterations,
      n_1/2 is the guaranteed upper bound for bisection method
      n_1/2 = ceiling(log2((b - a) / eps))
      n0 is a parameter which we have set to 1
      default 25
  Returns:
    x_itp - the approximate root produced by the ITP method
    exceed - whether or not iterlimit was exceeded. If this
      is false, x_itp is guaranteed to be within eps of some
      x* where fn(x*) = 0
]]
function MathUtil.ITP(fn, a, b, eps, iterlimit)
  if fn(a) * fn(b) > 0 then
    return nil
  end
  if fn(a) > fn(b) then
    fn = function(x)
      return -fn(x)
    end
  end
  eps = eps or 1e-5
  iterlimit = iterlimit or 25
  -- hardwired parameters
  local k1 = 0.2 / (b - a)
  local k2 = 2
  local n0 = 1

  local n_bisect = math.ceil(math.log((b - a) / (2 * eps), 2))
  local n_max = n_bisect + n0

  local iters = iterlimit
  for j=1, iterlimit do
    -- Interpolate (I)
    local fa = fn(a)
    local fb = fn(b)
    local base = fa - fb
    if base == 0 then return a end
    local x_bisect = 0.5 * (a + b)
    local x_falsi = (b * fa + a * fb) / base
    -- FtD Lua is a bit faulty so we need this duplicate line to prevent skipped instructions from causing problems
    if x_bisect < a or x_bisect > b then
      x_bisect = 0.5 * (a + b)
    end

    -- Truncate (T)
    local diff = x_bisect - x_falsi
    local delta = k1 * math.abs(b - a) ^ k2
    local sigma = diff > 0 and 1 or (diff == 0 and 0 or -1)
    local x_t = delta <= math.abs(diff) and x_falsi + sigma * delta or x_bisect

    -- Project (P)
    local rho_k = eps * 2 ^ (n_max - j) - 0.5 * (b - a)
    local x_p = math.abs(x_t - x_bisect) <= rho_k and x_t or x_bisect - sigma * rho_k

    -- Update
    local y_p = fn(x_p)
    if y_p > 0 then
      b = x_p
    elseif y_p < 0 then
      a = x_p
    else
      return x_p, j == iterlimit
    end
    if b - a < 2 * eps then
      iters = j
      break
    end
  end
  local fa = fn(a)
  local fb = fn(b)
  local base = fb - fa
  if base ~= 0 then return (a * fb - b * fa) / base, iters == iterlimit end
  return a, iters == iterlimit
end

-- thanks to Ribtoks and Frédéric van der Plancke on StackOverflow for algorithm
function MathUtil.binomCoeffs(depth, calc_all)
  if calc_all then
    local coeffs = {}
    -- WIP
  else
    local coeffs = {}
    coeffs[1] = 1
    for k=1,depth do
      coeffs[k+1] = (coeffs[k] * (depth - k)) / (k + 1)
    end
    return coeffs
  end
end

--[[
  Use Descartes' rule of signs to estimate how many real roots greater than some offset
  a polynomial has. The actual number of roots is the number given by the rule of signs
  minus some multiple of 2, including 0.
  Running this twice with different offsets and taking the difference can give you an estimate
  for the number of roots in an interval.
  Arguments:
    coeffs - the coefficients of a polynomial
    offset - any real number, see return values for purpose
  Returns:
    roots - the possible number of real roots greater than offset
            actual number of roots is less than this by an even number
            i.e. if this returns 3, there are either 3 or 1 roots greater than offset
]]
function MathUtil.ruleOfSigns(coeffs, offset)
  -- expand the polynomial with u = x + offset
  local shiftedCoeffs = {}
  local nCoeffs = #coeffs
  for i = 1, nCoeffs do
    shiftedCoeffs[i] = coeffs[nCoeffs - i + 1]
  end
  if offset ~= 0 then
    -- use Taylor expansion to efficiently get coefficients of shifted polynomial
    local tempCoeffs = {}
    for i = 1, nCoeffs do
      tempCoeffs[i] = (i - 1) * coeffs[nCoeffs - i + 1]
    end
    local factor = 1
    for i = 2, nCoeffs do
      local power_x = offset ^ (i - 1)
      for j = 1, nCoeffs - i + 1 do
        local shiftIdx = i + j - 1
        shiftedCoeffs[j] = shiftedCoeffs[j] + factor * tempCoeffs[shiftIdx] * power_x
        tempCoeffs[shiftIdx] = tempCoeffs[shiftIdx] * (j - 1)
      end
      factor = factor / i
    end
  end
  -- remove coefficients that are zero
  local nonZeroCoeffs = {}
  local n = 1
  for i, coeff in ipairs(shiftedCoeffs) do
    if coeff ~= 0 then
      nonZeroCoeffs[n] = coeff
      n = n + 1
    end
  end
  local count = 0
  for i = 1, #nonZeroCoeffs - 1 do
    if (nonZeroCoeffs[i] * nonZeroCoeffs[i + 1]) < 0 then
      count = count + 1
    end
  end
  return count
end

function MathUtil.cache(fn)
  local c = {}
  local mt = getmetatable(c) or {}
  function mt.__index(tab, x)
    local val = fn(x)
    tab[x] = val
    return val
  end
  setmetatable(c, mt)
  return function(a)
    return c[a]
  end
end

function MathUtil.lerp(fn, start, stop, step, extrap)
  local vals = {}
  for i=1, math.floor((stop - start) / step) + 1 do
    vals[i] = fn(start + i * step)
  end
  vals.start = start
  vals.stop = stop
  vals.step = step
  vals.lval = extrap and vals[1] or nil
  vals.rval = extrap and vals[#vals] or nil
  return function(x)
    if x >= vals.stop then return vals.rval end
    if x <= vals.start then return vals.lval end
    local i = (x - vals.start) / vals.step
    local fac = i % 1
    i = math.floor(i)
    return (1 - fac) * vals[i] + fac * vals[i + 1]
  end
end

function MathUtil._factorial(n)
  if n < 2 then return 1 end
  return MathUtil._factorial(n - 1)
end

MathUtil.factorial = MathUtil.cache(MathUtil._factorial)

--[[
function MathUtil.rfft(data, lerp)
  local cosLerp
  if lerp == nil then
    lerp = 0.05
  elseif type(lerp) == "function" then
    cosLerp = lerp
  else
    cosLerp = MathUtil.lerp(function(x) return math.cos(x * math.pi) end, 0, 2, lerp)
  end
  local n = #data
  if data % 2 == 0 and n > 32 then
    local ldata = {}
    local rdata = {}
    local lres = MathUtil.rfft(ldata, cosLerp)
    local rres = MathUtil.rfft(rdata, cosLerp)
    -- WIP
  else
    local res = {}
    for w=1, n do
      res[w] = 0
    end
    for i=1, n do
      for w=1, n do
        local theta = (((w - 1) * (i - 1)) / n) % 2
        res[w] = res[w] + data[i] * cosLerp(theta)
      end
    end
  end
end
--]]

-- code from lua-polynomials by piqey (John Kushmer) on github
-- modified to fix a missed branch in solveQuartic
-- https://github.com/piqey/lua-polynomials
-- GNU General Public License, version 3

MathUtil.eps = 1e-9

-- checks if d is close enough to 0 to be considered 0 (for our purposes)
function MathUtil.isZero(d)
  return (d > -MathUtil.eps and d < MathUtil.eps)
end

-- sets the tolerance for a number to be considered zero
function MathUtil.setTolerance(eps)
  MathUtil.eps = eps
end

-- fixes an issue with math.pow that returns nan when the result should be a real number
function MathUtil.cuberoot(x)
  -- roots of negative numbers are ill-behaved
  -- (1/3) gets converted to a float and is approximated
  -- so Lua rejects it because it doesn't realize we want the exact cube root
  return (x > 0) and (x ^ (1 / 3)) or -(math.abs(x) ^ (1 / 3))
end

function MathUtil.solveQuadratic(c0, c1, c2)
  local s0, s1

  local p, q, D

  -- x^2 + 2px + q = 0
  p = c1 / (2 * c0)
  q = c2 / c0

  D = p * p - q

  if MathUtil.isZero(D) then
    s0 = -p
    return s0
  elseif (D < 0) then
    return
  else -- if (D > 0)
    local sqrt_D = math.sqrt(D)

    s0 = sqrt_D - p
    s1 = -sqrt_D - p
    return s0, s1
  end
end

function MathUtil.solveCubic(c0, c1, c2, c3)
  local s0, s1, s2

  local num, sub
  local A, B, C
  local sq_A, p, q
  local cb_p, D

  -- normal form: x^3 + Ax^2 + Bx + C = 0
  A = c1 / c0
  B = c2 / c0
  C = c3 / c0

  -- substitute x = y - A/3 to eliminate quadric term: y^3 + 3py + 2q = 0
  sq_A = A * A
  p = (1 / 3) * (-(1 / 3) * sq_A + B)
  q = 0.5 * ((2 / 27) * A * sq_A - (1 / 3) * A * B + C)

  -- use Cardano's formula
    -- 27 and 4 factors are already included in p, q
  cb_p = p * p * p
  D = q * q + cb_p

  if MathUtil.isZero(D) then
    if MathUtil.isZero(q) then -- one triple solution
      s0 = 0
      num = 1
      --return s0
    else -- one single and one double solution
      local u = MathUtil.cuberoot(-q)
      s0 = 2 * u
      -- s1 = u^3 + v^3, u^3 = v^3 = -q/2
      -- u and v are the real cube root of -q/2
        -- times the 1st and 2nd cuberoots of unity
      -- sum of 1st and 2nd cube roots of unity is just -1
      s1 = -u
      num = 2
      --return s0, s1
    end
  elseif (D < 0) then -- Casus irreducibilis: three real solutions
    -- sign of q is flipped relative to wikipedia formula because
    -- sqrt(-cb_p) = sqrt(p^2) * sqrt(-p) = |p| sqrt(-p)
    -- D < 0 guarantees p < 0 so |p| = -p
    local phi = (1 / 3) * math.acos(-q / math.sqrt(-cb_p))
    local t = 2 * math.sqrt(-p)

    s0 = t * math.cos(phi)
    s1 = -t * math.cos(phi + math.pi / 3)
    s2 = -t * math.cos(phi - math.pi / 3)
    num = 3
    --return s0, s1, s2
  else -- one real solution
    local sqrt_D = math.sqrt(D)
    local u = MathUtil.cuberoot(sqrt_D - q)
    local v = -MathUtil.cuberoot(sqrt_D + q)

    s0 = u + v
    num = 1

    --return s0
  end

  -- resubstitute
  sub = (1 / 3) * A

  if (num > 0) then s0 = s0 - sub end
  if (num > 1) then s1 = s1 - sub end
  if (num > 2) then s2 = s2 - sub end

  return s0, s1, s2
end

function MathUtil.solveQuartic(c0, c1, c2, c3, c4)
  local s0, s1, s2, s3

  local coeffs = {}
  local z, u, v, sub
  local A, B, C, D
  local sq_A, p, q, r
  local num = 0

  -- normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0
  A = c1 / c0
  B = c2 / c0
  C = c3 / c0
  D = c4 / c0

  -- substitute x = y - A/4 to eliminate cubic term: x^4 + px^2 + qx + r = 0
  sq_A = A * A
  p = -0.375 * sq_A + B
  q = 0.125 * sq_A * A - 0.5 * A * B + C
  r = -(3 / 256) * sq_A * sq_A + 0.0625 * sq_A * B - 0.25 * A * C + D

  if MathUtil.isZero(r) then
    -- no absolute term: y(y^3 + py + q) = 0
    coeffs[3] = q
    coeffs[2] = p
    coeffs[1] = 0
    coeffs[0] = 1

    local results = {MathUtil.solveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3])}
    num = #results
    s0, s1, s2 = results[1], results[2], results[3]
  elseif MathUtil.isZero(q) then
    -- special case to avoid divide by zero in general formula
    -- if q is zero then y^4 + py^2 + q = 0, which is a biquadratic
    local biquad_results = {MathUtil.solveQuadratic(1, p, r)}
    if biquad_results[1] >= 0 then
      s0 = -math.sqrt(biquad_results[1])
      s1 = math.sqrt(biquad_results[1])
      num = 2
    end
    if biquad_results[2] >= 0 then
      if num == 0 then
        -- this case shouldn't happen since solveQuadratic returns roots
          -- in decreasing order, but best not to assume root ordering
        s0 = -math.sqrt(biquad_results[2])
        s1 = math.sqrt(biquad_results[2])
        num = 2
      else
        s2 = -math.sqrt(biquad_results[2])
        s3 = math.sqrt(biquad_results[2])
        num = 4
      end
    end
  else
    -- solve the resolvent cubic …
    coeffs[3] = 0.5 * r * p - 0.125 * q * q
    coeffs[2] = -r
    coeffs[1] = -0.5 * p
    coeffs[0] = 1

    s0, s1, s2 = MathUtil.solveCubic(coeffs[0], coeffs[1], coeffs[2], coeffs[3])

    -- … and take the one real solution …
    z = s0

    -- … to build two quadric equations
    u = z * z - r
    v = 2 * z - p

    if MathUtil.isZero(u) then
      u = 0
    elseif (u > 0) then
      u = math.sqrt(u)
    else
      return
    end

    if MathUtil.isZero(v) then
      v = 0
    elseif (v > 0) then
      v = math.sqrt(v)
    else
      return
    end

    coeffs[2] = z - u
    coeffs[1] = q < 0 and -v or v
    coeffs[0] = 1

    do
      local results = {MathUtil.solveQuadratic(coeffs[0], coeffs[1], coeffs[2])}
      num = #results
      s0, s1 = results[1], results[2]
    end

    coeffs[2] = z + u
    coeffs[1] = q < 0 and v or -v
    coeffs[0] = 1

    if (num == 0) then
      local results = {MathUtil.solveQuadratic(coeffs[0], coeffs[1], coeffs[2])}
      num = num + #results
      s0, s1 = results[1], results[2]
    end

    if (num == 1) then
      local results = {MathUtil.solveQuadratic(coeffs[0], coeffs[1], coeffs[2])}
      num = num + #results
      s1, s2 = results[1], results[2]
    end

    if (num == 2) then
      local results = {MathUtil.solveQuadratic(coeffs[0], coeffs[1], coeffs[2])}
      num = num + #results
      s2, s3 = results[1], results[2]
    end
  end

  -- resubstitute
  sub = 0.25 * A

  if (num > 0) then s0 = s0 - sub end
  if (num > 1) then s1 = s1 - sub end
  if (num > 2) then s2 = s2 - sub end
  if (num > 3) then s3 = s3 - sub end

  return s0, s1, s2, s3
end

-- Why all these -1s? Because apparently FtD's Lua interpreter can't correctly handle loops which start from 0 (is my working theory)

function Matrix3.Matrix3(vals)
  local mat = {}
  for i=1,9 do
    mat[i] = vals[i]
  end
  setmetatable(mat, Matrix3.meta)
  return mat
end

function Matrix3.get(mat, row, col)
  return mat[(row - 1) * 3 + col]
end

function Matrix3.set(mat, row, col, val)
  mat[(row - 1) * 3 + col] = val
end

function Matrix3.scalarmul(mat, s)
  local ret = {}
  for i=1,9 do
    ret[i] = s * mat[i]
  end
  return Matrix3.Matrix3(ret)
end

function Matrix3.vecmul(mat, vec)
  local ret = Vector3.zero
  for row=1,3 do
    local val = 0
    for col=1,3 do
      val = val + vec[col] * mat[(row - 1) * 3 + col]
    end
    ret[row] = val
  end
  return ret
end

function Matrix3.matmul(mat1, mat2)
  local ret = {}
  for i=1,3 do
    for j=1,3 do
      local val = 0
      for k=1,3 do
        val = val + mat1[(i - 1) * 3 + k] * mat2[(k - 1) * 3 + j]
      end
      ret[(i - 1) * 3 + j] = val
    end
  end
  return Matrix3.Matrix3(ret)
end

function Matrix3.mul(a, b)
  if getmetatable(a) ~= Matrix3.meta then
    if type(a) == "table" and a.x then return Matrix3.vecmul(Matrix3.transpose(b), a) end
    return Matrix3.mul(b, a)
  end
  if getmetatable(b) == Matrix3.meta then
    return Matrix3.matmul(a, b)
  end
  if type(b) == "table" and b.x then
    return Matrix3.vecmul(a, b)
  end
  return Matrix3.scalarmul(a, b)
end

function Matrix3.quadform(vec1, mat, vec2)
  return Vector3.Dot(vec1, mat * vec2)
end

function Matrix3.Identity()
  return Matrix3.Matrix3({
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
  })
end

function Matrix3.Zero()
  return Matrix3.Matrix3({
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
  })
end

function Matrix3.pow(mat, pow)
  local prod = mat
  local temp = mat
  while true do
    pow = math.floor(pow / 2)
    if pow % 2 == 1 then
      prod = Matrix3.matmul(mat, temp)
    end
    if pow >= 2 then
      temp = Matrix3.matmul(temp, temp)
    else
      break
    end
  end
  return prod
end

function Matrix3.add(mat1, mat2)
  local ret = {}
  for i=1,9 do
    ret[i] = mat1[i] + mat2[i]
  end
  return Matrix3.Matrix3(ret)
end

function Matrix3.hadamard(mat1, mat2)
  local ret = {}
  for i=1,9 do
    ret[i] = mat1[i] * mat2[i]
  end
  return Matrix3.Matrix3(ret)
end

function Matrix3.transpose(mat)
  local transposed = {}
  for row=1,3 do
    for col=1,3 do
      transposed[(col - 1) * 3 + row] = mat[(row - 1) * 3 + col ]
    end
  end
  return Matrix3.Matrix3(transposed)
end

function Matrix3.determinant(mat)
  local sum1 = 0
  local sum2 = 0
  for d=1,3 do
    local diag1 = 1
    local diag2 = 1
    for s=1,3 do
      diag1 = diag1 * mat[(s - 1) * 3 + (s + d - 2) % 3 + 1]
      diag2 = diag2 * mat[(s - 1) * 3 + (-s + d) % 3 + 1]
    end
    sum1 = sum1 + diag1
    sum2 = sum2 + diag2
  end
  return sum1 - sum2
end

function Matrix3.adjugate(mat)
  local adj = {}
  for row=1,3 do
    for col=1,3 do
      local diag1 = 1
      local diag2 = 1
      for i=1,2 do
        diag1 = diag1 * mat[(row + i - 1) % 3 * 3 + (col + i - 1) % 3 + 1]
        diag2 = diag2 * mat[(row + i - 1) % 3 * 3 + (col - i - 1) % 3 + 1]
      end
      adj[(col - 1) * 3 + row] = diag1 - diag2
    end
  end
  return Matrix3.Matrix3(adj)
end

function Matrix3.inverse(mat)
  local det = Matrix3.determinant(mat)
  if MathUtil.isZero(det) then return end
  local adj = Matrix3.adjugate(mat)
  return (1 / det) * adj 
end

function Matrix3.tostring(mat)
  local str = string.format("%f, %f, %f\n%f, %f, %f\n%f, %f, %f", unpack(mat))
  return str
end

Matrix3.meta = {
  __add = Matrix3.add,
  __mul = Matrix3.mul,
  __unm = function(m) return Matrix3.scalarmul(m, -1) end,
  __pow = Matrix3.pow,
  __tostring = Matrix3.tostring,
}

--[[
  A ringbuffer is an efficient FIFO buffer in the form of a circular
  array. This allows for fast removal from the ends (though the
  convention is that removal occurs from the head and addition occurs
  at the tail) and fast access of random elements in the array.
]]--

-- Creates a new ring buffer with specified capacity.
function RingBuffer.RingBuffer(capacity)
  local rb = {}
  rb.buf = {}
  rb.capacity = capacity
  rb.size = 0
  rb.head = 1
  local mt = getmetatable(rb) or {}
  mt.__index = RingBuffer.get
  setmetatable(rb, mt)
  return rb
end

-- Checks whether a RingBuffer is full
function RingBuffer.isFull(rb)
  return rb.size >= rb.capacity
end

-- Sets the size of the RingBuffer
-- Equivalent to filling beginning with nils
-- todo: make compatible with capacity
--[[function RingBuffer.setSize(rb, size)
  rb.size = size
end--]]

-- Adds a value to the tail of the RingBuffer.
function RingBuffer.push(rb, value)
  rb.buf[(rb.head + rb.size - 1) % rb.capacity + 1] = value
  if rb.size == rb.capacity then
    rb.head = rb.head % rb.capacity + 1
  else
    rb.size = rb.size + 1
  end
end

-- Removes and returns a value from the head of the RingBuffer.
-- Returns nil if empty.
function RingBuffer.pop(rb)
  if rb.size == 0 then return nil end
  local val = rb.buf[rb.head]
  rb.buf[rb.head] = nil
  rb.head = rb.head % rb.capacity + 1
  rb.size = rb.size - 1
  return val
end

-- Gets a value at a particular index in the RingBuffer
function RingBuffer.get(rb, idx)
  if type(idx) ~= "number" or math.floor(idx) ~= idx then return nil end
  if idx < 1 or idx > rb.size then return nil end
  return rb.buf[(rb.head + idx - 2) % rb.capacity + 1]
end

function Search.interpolatedSearch(list, left, right, target, findClosest, iterLim)
    iterLim = iterLim or 50
    local a, b, split
    local totalIter = 0
    while right > left do
      a = list[left]
      if a == target then return left end
      if a > target then return findClosest and left or nil end
      b = list[right]
      if b == target then return right end
      if b < target then return findClosest and right or nil end
      split = math.floor((target - a) / (b - a) * (right - left) + left)
      split = math.min(math.max(split, left + 1), right - 1)
      if list[split] == target then return split end
      if target < list[split] then
        if findClosest and math.abs(list[split - 1] - target) > math.abs(list[split] - target) then
          return split
        end
        right = split - 1
      else
        if findClosest and math.abs(list[split + 1] - target) > math.abs(list[split] - target) then
          return split
        end
        left = split + 1
      end
      totalIter = totalIter + 1
      if totalIter > iterLim then
        break
      end
    end
    return findClosest and left or nil
  end

function Stats.Distribution(vars)
  local distr = { n = 0, vars = vars }
  if vars then
    local mean = {}
    local cov = {}
    local nVars = #vars
    for i, v in ipairs(vars) do
      mean[v] = 0
      for j=1, nVars do
        cov[(i - 1) * nVars + j] = 0
      end
    end
    distr.mean = mean
    distr.cov = Matrix3.Matrix3(cov)
  else
    distr.mean = 0
    distr.cov = 0
  end
  return distr
end

function Stats.updateDistribution(distr, sample, weight)
  local oldN = distr.n
  weight = weight or 1
  distr.n = distr.n + weight

  if distr.vars then
    local oldMeans = {}
    local nVars = distr.vars and #distr.vars or 1
    for i, v in ipairs(distr.vars) do
      -- Update mean
      oldMeans[i] = distr.mean[v]
      local mu = oldMeans[i] + sample[v] * weight / distr.n

      -- Update covariance
      for j=i, nVars do
        local v2 = distr.vars[j]
        local mu2 = distr.mean[v2]
        local upd = (weight or 1) * (sample[v] - mu) * (sample[v2] - mu2)
        distr.cov[(i - 1) * nVars + j] = (distr.cov[(i - 1) * nVars + j] * oldN + upd) / distr.n
        distr.cov[(j - 1) * nVars + i] = distr.cov[(i - 1) * nVars + j]
      end

      distr.mean[v] = mu
    end
  else
    local mu = distr.mean + sample * weight / distr.n
    distr.cov = (distr.cov * oldN + weight * (sample - mu) * (sample - distr.mean)) / distr.n

    distr.mean = mu
  end
end

function Stats.updateDistributionBatched(distr, samples, weights)
  if #samples == 0 then return end
  local sT = {}
  local nVars = distr.vars and #distr.vars or 1

  local wSum = 0
  for j=1, #samples do
    wSum = wSum + (weights and weights[j] or 1)
  end
  distr.n = distr.n + wSum

  local oldN = distr.n
  if distr.vars then
    for i, var in ipairs(distr.vars) do
      local col = {}
      for j=1, #samples do
        col[j] = samples[j][var]
      end
      sT[i] = col
    end

    for i, v in ipairs(distr.vars) do
      -- Update mean
      local sum = 0
      for j, s in ipairs(sT[i]) do
        sum = sum + s * (weights and weights[j] or 1)
      end
      local mu = distr.mean[v] + sum / distr.n

      -- Update covariance
      for j=i, nVars do
        local mu2 = distr.mean[distr.vars[j]]
        sum = 0
        for s=1, #samples do
          sum = sum + (weights and weights[s] or 1) * (sT[i][s] - mu) * (sT[j][s] - mu2)
        end
        distr.cov[(i - 1) * nVars + j] = (distr.cov[(i - 1) * nVars + j] * oldN + sum) / distr.n
        distr.cov[(j - 1) * nVars + i] = distr.cov[(i - 1) * nVars + j]
      end

      distr.mean[v] = mu
    end
  else
    local sum = 0
    for i, s in ipairs(samples) do
      sum = sum + s * (weights and weights[i] or 1)
    end

    local mu = distr.mean + sum / distr.n

    sum = 0
    for i, s in ipairs(samples) do
      sum = sum + (weights and weights[i] or 1) * (s - mu) * (s - distr.mean)
    end
    distr.cov = (distr.cov * oldN + sum) / distr.n
  end
end

function Stats.mean(distr)
  return distr.mean
end

function Stats.covariance(distr)
  return distr.cov
end

function Stats.namedCovariance(distr, var1, var2)
  if not distr.vars then return distr.cov end
  local nVars = distr.vars and #distr.vars or 1
  for i=1,nVars do
    if distr.vars[i] == var1 then
      for j=1,nVars do
        if distr.vars[j] == var2 then
          return distr.cov[(i - 1) * nVars + j]
        end
      end
    end
  end
end

--[[
  Returns:
    z - a normally distributed random number
--]]
function Stats.normal()
  local z, z1 = Stats.boxMuller()
  return z
end

--[[
  Arguments:
    z - a z-score
  Returns:
    d - the standard normal probability density function at z
]]
function Stats.normalPDF(z)
  return math.exp(-0.5 * z * z) / math.sqrt(2 * math.pi)
end

--[[
  Arguments:
    z - a z-score
  Returns:
    p - the p-value corresponding to that z-score. Approximately calculated using Zelen and Severo (1964) approximation
--]]
function Stats.normalCDF(z)
  local b0 = 0.2316419
  local b1 = 0.319381530
  local b2 = -0.356563782
  local b3 = 1.781477937
  local b4 = -1.821255978
  local b5 = 1.330274429
  local t = 1 / (1 + b0 * z)
  return 1 - Stats.normalPDF(z) * (b1 * t + b2 * t^2 + b3 * t^3 + b4 * t^4 + b5 * t^5)
end

--[[
  Arguments:
    p - a p-value
  Returns:
    z - the z-score corresponding to that p-value. Approximately calculated using Shore (1982) approximation
--]]
function Stats.inverseNorm(p)
  local pRight = p >= 0.5 and p or -p
  local z = 5.55556 * (1 - ((1 - pRight) / pRight) ^ 0.1186) -- Shore (1982) approximation for normal quantile
  if p < 0.5 then z = -z end
  return z
end

--[[
  Returns:
    z1, z2 - two normally distributed random numbers
--]]
function Stats.boxMuller()
  local u1 = math.random()
  local u2 = math.random() -- idk what algorithm math.random uses, but it might be an LCG since it's simple and common
  u2 = math.random() -- it's also not very high-quality, which also describes LCGs. Box-Muller performs poorly when the
  u2 = math.random() -- input comes from two consecutive numbers of an LCG, so we throw away values in between.
  local r = math.sqrt(-2 * math.log(u1))
  local theta = 2 * math.pi * u2
  return r * math.cos(theta), r * math.sin(theta)
end

VectorN.mt = getmetatable({}) or {}
VectorN.mt.__add = function(a, b)
  local aInt = type(a) == "number"
  local bInt = type(b) == "number"
  if not aInt and bInt then return b + a end
  if aInt and not bInt then
    return Stats.combine(a, b, function(k, x, y) return a + y end)
  else
    return Stats.combine(a, b, function(k, x, y) return x + y end)
  end
end

VectorN.mt.__sub = function(a, b)
  return a + (-b)
end

VectorN.mt.__mul = function(a, b)
  local aInt = type(a) == "number"
  local bInt = type(b) == "number"
  if not aInt and bInt then return b * a end
  if aInt and not bInt then
    local res = {}
    for k, v in pairs(b) do
      res[k] = a * v
    end
    return res
  else
    return Stats.combine(a, b, function(k, x, y) return x * y end)
  end
end

VectorN.mt.__div = function(a, b)
  local aInt = type(a) == "number"
  local bInt = type(b) == "number"
  if not aInt and bInt then return a * (1 / b) end
  if aInt and not bInt then
    local res = {}
    for k, v in pairs(b) do
      res[k] = a / v
    end
    return res
  else
    return Stats.combine(a, b, function(k, x, y) return x / y end)
  end
end

VectorN.mt.__unm = function(a)
  local res = {}
  for k, v in pairs(a) do
    res[k] = -v
  end
  return res
end

function VectorN.VectorN(lst)
  local vec = {}
  for k, v in pairs(lst) do
    if type(v) == "table" then
      vec[k] = VectorN.VectorN(v)
    else
      vec[k] = v
    end
  end
  setmetatable(vec, VectorN.mt)
  return vec
end

-- dependencies: Accumulator, LinkedList

function Control.PID(kP, kI, kD, IDecay, IWindow, period)
  local pid = {}
  pid.kP = kP
  pid.kI = kI
  pid.kD = kD
  pid.Iacc = Accumulator.Accumulator(IDecay, IWindow)
  if period and period ~= 0 then
    pid.period = period
  end
  return pid
end

function Control.processPID(ctrl, e, time)
  e = ctrl.period and (e + ctrl.period / 2) % ctrl.period - ctrl.period / 2 or e
  local p = ctrl.kP * e
  local i, wt = Accumulator.update(ctrl.Iacc, e, time)
  i = ctrl.kI * i / wt
  local d = ctrl.kD * (e - (ctrl.lastError or e)) / time
  ctrl.lastError = e
  return p + i + d
end

function Control.FF(coeffs, period)
  local ff = {}
  ff.coeffs = coeffs
  ff.degree = #coeffs - 1
  if period and period ~= 0 then
    ff.period = period
  end
  ff.derivs = {}
  return ff
end

function Control.processFF(ctrl, target, time)
  -- initial value zero but of the same type as target
  local response = 0 * target
  local newVal = target
  -- placeholder non-nil value
  local lastVal = target
  for i = 1, ctrl.degree + 1 do
    lastVal = ctrl.derivs[i]
    ctrl.derivs[i] = newVal
    response = response + ctrl.coeffs[i] * newVal
    if lastVal then
      local diff = newVal - lastVal
      if i == 1 and ctrl.period then
        diff = (diff + ctrl.period / 2) % ctrl.period - ctrl.period / 2
      end
      newVal = diff / time
    else
      break
    end
  end
  return response
end

--[[
  Arguments:
    gCoords - a point in global coordinates
    pos - the reference position in global coordinates
    orient - the rotation of the reference frame.
             Can be obtained using Quaternion.LookRotation(I:GetConstructForwardVector(), I:GetConstructUpVector())
  Returns:
    lCoords - the point in local coordinates
]]--
function Nav.toLocal(gCoords, pos, orient)
  local relCoords = gCoords - pos
  return Quaternion.Inverse(orient) * relCoords
end

--[[
  Arguments:
    lCoords - a point in local coordinates
    pos - the reference position in global coordinates
    orient - the rotation of the reference frame.
             Can be obtained using Quaternion.LookRotation(I:GetConstructForwardVector(), I:GetConstructUpVector())
  Returns:
    gCoords - the point in global coordinates
]]--
function Nav.toGlobal(lCoords, pos, orient)
  local relCoords = orient * lCoords
  return relCoords + pos
end

-- Converts Cartesian coordinates to polar coordinates
-- Returns a vector3 with elements (radius, azimuth, elevation)
function Nav.cartToPol(coords)
  local r = coords.magnitude
  -- clockwise is positive
  local theta = Vector3.SignedAngle(Vector3.forward, coords, Vector3.up)
  -- zero is horizontal, up is positive
  local phi = 90 - Vector3.Angle(Vector3.up, coords)
  return Vector3(r, theta, phi)
end

-- Converts Cartesian coordinates to cylindrical coordinates
-- Returns a vector3 with elements (radius, azimuth, height)
function Nav.cartToCyl(coords)
  local coordsH = Vector3(coords.x, 0, coords.z)
  local rho = coordsH.magnitude
  local phi = Vector3.SignedAngle(Vector3.forward, coords, Vector3.up)
  local z = coords.y
  return Vector3(rho, phi, z)
end

-- Converts polar coordinates to cartesian coordinates
function Nav.polToCart(coords)
  local r, theta, phi = coords.x, coords.y, coords.z
  local x = Mathf.Sin(theta) * Mathf.Cos(phi)
  local y = Mathf.Sin(phi)
  local z = Mathf.Cos(theta) * Mathf.Cos(phi)
  return r * Vector3(x, y, z)
end

-- Converts cylindrical coordinates to cartesian coordinates
function Nav.cylToCart(coords)
  local rho, phi, zCyl = coords.x, coords.y, coords.z
  local x = rho * Mathf.Sin(phi)
  local y = zCyl
  local z = rho * Mathf.Cos(phi)
  return Vector3(x, y, z)
end

--[[
WIP
function Nav.aStar()

end

function Nav.marchingSquares()

end

function Nav.visGraph()

end
--]]


-- dependencies: MathUtil

-- Returns the direction in which the weapon should fire to hit a target
-- Assumes both target and projectile travel in a straight line (i.e. no gravity)
-- targetVel should be relative to the gun for projectiles, absolute for missiles
-- Also use this for TPG guidance on missiles
--[[
  Arguments:
    relPos - position of the target relative to muzzle
    targetVel - absolute velocity of the target
    muzzle - muzzle velocity of the projectile
  Returns:
    intercept - the position of the target at time of intercept
    interceptTime - the time at which intercept occurs
]]
function Targeting.firstOrderTargeting(relPos, targetVel, muzzle)
  if targetVel.sqrMagnitude == 0 then
    return relPos.normalized
  end
  local closest = relPos - Vector3.Project(relPos, targetVel)
  local timeAlongLine = Vector3.Dot(targetVel, relPos - closest) / targetVel.sqrMagnitude
  -- by pythagorean theorem, intercept time t occurs when
  --   (t + timeAlongLine)^2 * targetVel.sqrMagnitude + closest.sqrMagnitude = (muzzle * t)^2
  local a, b = MathUtil.solveQuadratic(targetVel.sqrMagnitude - muzzle * muzzle,
                                                2 * timeAlongLine * targetVel.sqrMagnitude,
                                                closest.sqrMagnitude + timeAlongLine * timeAlongLine * targetVel.sqrMagnitude)
  local interceptTime = nil
  if a and a >= 0 then interceptTime = a end
  if b and b >= 0 and b < a then interceptTime = b end
  if interceptTime then
    return relPos + interceptTime * targetVel, interceptTime
  end
end

--[[
  Arguments:
    relPos - position of the target relative to own vehicle
    relVel - velocity of the target relative to own vehicle
    accel - acceleration of the target relative to the projectile (absolute target accel - gravity)
    muzzle - muzzle velocity of the projectile
    minRange - the minimum distance of intercept (see return values)
    maxRange - the maximum distance of intercept (see return values)
    guess - [optional] initial guess for intercept time
  Returns:
    intercept - the position of the target at time of intercept if between minRange and maxRange
      nil otherwise
      Range is measured in effective range, which is how far the projectile would've
      traveled if gravity didn't exist.
    interceptTime - the time at which intercept occurs
]]
function Targeting.secondOrderTargetingNewton(relPos, relVel, accel, muzzle, minRange, maxRange, guess)
  local diff = 10000
  local lastT = 0
  local iters = 0
  if not guess then guess = 0 end
  local newPos = relPos + guess * relVel + 0.5 * guess * guess * accel
  while math.abs(diff) > 0.001 and iters < 10 do
    local t = newPos.magnitude / muzzle
    diff = t - lastT
    lastT = t
    iters = iters + 1
    newPos = relPos + t * relVel + 0.5 * t * t * accel
  end
  return newPos, lastT, iters
end

Targeting.secondOrderTargeting = Targeting.secondOrderTargetingNewton

-- Same arguments and return values as secondOrderTargetingNewton.
-- This version uses ITP, and is guaranteed to return an answer if one exists, unlike Newton's method.
-- However, it is much more complicated and, for most scenarios found in FtD, probably slower.
function Targeting.secondOrderTargetingITP(relPos, relVel, accel, muzzle, minRange, maxRange, guess)
  if not guess then guess = 0 end
  local a = -0.25 * accel.sqrMagnitude
  local b = -Vector3.Dot(relVel, accel)
  local c = -(relVel.sqrMagnitude - muzzle * muzzle + Vector3.Dot(relPos, accel))
  local d = -2 * Vector3.Dot(relPos, relVel)
  local e = -relPos.sqrMagnitude
  local t

  local g = accel.magnitude
  local v_T = relVel.magnitude
  local d_0 = relPos.magnitude
  local t1a, t1b = MathUtil.solveQuadratic(0.5 * g, v_T + muzzle, -d_0)
  local t1 = math.max(t1a, t1b)
  local t2

  -- todo: use rule of signs to check for positive roots before solving quadratic
  local coeffs = {0.5 * g, v_T - muzzle, d_0}
  if MathUtil.ruleOfSigns(coeffs, 0) == 2 then
    local t2a, t2b = MathUtil.solveQuadratic(coeffs[1], coeffs[2], coeffs[3])
    if t2a then
      t2 = math.min(t2a, t2b)
    end
  end
  if not t2 or t2 < t1 then
    local s0, s1, s2 = MathUtil.solveCubic(4 * a, 3 * b, 2 * c, d)
    if not s2 then
      if s0 > t1 then
        t2 = s0
      end
    else
      local t2a = math.min(s0, s2)
      local t2b = math.max(s0, s2)
      if t2a > t1 then
        t2 = t2a
      elseif t2b > t1 then
        t2 = t2b
      end
    end
    if not t2 then
      return
    end
  end

  local function poly(x)
    -- Horner's method of evaluating polynomials is slightly faster
    return e + x * (d + x * (c + x * (b + x * a)))
  end
  if guess > t1 and t2 > guess then
    if poly(t1) * poly(t) < 0 then
      t2 = guess
    else
      t1 = guess
    end
  end
  t = MathUtil.ITP(poly, t1, t2, 1e-4, 25)

  if not t then return end
  if t >= t1 and t <= t2 then
    local intercept = relPos + relVel * t + 0.5 * accel * t * t
    if intercept.magnitude >= minRange and intercept.magnitude <= maxRange then
      return intercept, t
    end
  end
end

--[[
  Modified version of IPPN guidance law from
  "An Improvement in Three-Dimensional Pure
  Proportional Navigation Guidance",
  by Hyo-Sang Shin and Ke-Bo Li
  with added acceleration term for augmentation.
  I have no idea if this is better than IPPN alone
  or even theoretically correct because I do not
  know how to perform the relevant mathematical
  analyses.
]]
function Targeting.AIPPN(gain, relPos, missileVel, targetVel, targetAccel)
  local relVel = targetVel - missileVel
  local closingRate = Vector3.Dot(-relVel, relPos.normalized)
  if closingRate <= 0 then closingRate = 10 end
  local tgo = relPos.magnitude / closingRate

  local losRot = Vector3.Cross(relPos, relVel) / relPos.sqrMagnitude
  local aRot = Vector3.Cross(relPos, targetAccel) / relPos.sqrMagnitude * tgo / 2
  local omega = losRot + aRot
  local losMotion = Vector3.Cross(omega, relPos.normalized)

  local accelDir = Vector3.ProjectOnPlane(losMotion, missileVel).normalized
  local accelMag = gain * missileVel.magnitude * omega.magnitude

  return accelMag * accelDir
end

--[[
  Augmented True Proportional Navigation
  guidance law which FTD missiles use
  when APN guidance is equipped
  (if my understanding is correct)

  True proportional guidance commands
  acceleration perpendicular to line of
  sight between missile and target
  instead of perpendicular to missile
  velocity like pure proportional
  guidance does.
--]]
function Targeting.ATPN(gain, relPos, missileVel, targetVel, targetAccel)
  local relVel = targetVel - missileVel
  local closingRate = -Vector3.Dot(relVel, relPos.normalized)
  if closingRate <= 0 then closingRate = 10 end

  local losRot = Vector3.Cross(relPos, relVel) / relPos.sqrMagnitude
  local losMotion = Vector3.Cross(losRot, relPos.normalized)

  local orthoAccel = Vector3.ProjectOnPlane(targetAccel, relPos)

  return gain * closingRate * losMotion + 0.5 * gain * targetAccel
end

--[[
  Arguments:
    I - the variable passed to Update
    name - the name of the weapons to search for
           can be set by pressing shift-N in build mode
    count - the number of weapons to search for
    mode - whether to search for weapons on the main hull
           or on subconstructs:
           0 = hull, 1 = subconstructs, 2 = both
  Returns:
    weapons - a list of tables with elements subIdx and wpnIdx
              subIdx is the ID of the subconstruct the weapon
              is mounted on
              wpnIdx is the index of the weapon

  NOTE: returns a list even if you specify count = 1
]]--
function BlockUtil.getWeaponsByName(I, name, count, mode)
  if DEBUG then I:Log("searching for "..name) end
  local weapons = {}
  count = count or -1
  local c = count
  if not mode or mode == 0 or mode == 2 then
    for i=0, I:GetWeaponCount() - 1 do
      if c == 0 then break end
      if I:GetWeaponBlockInfo(i).CustomName == name then
        table.insert(weapons, {subIdx = nil, wpnIdx = i})
        if DEBUG then I:Log("found weapon "..name.." on hull, type "..I:GetWeaponInfo(i).WeaponType) end
        c = c - 1
      end
    end
  end
  if not mode or mode == 1 or mode == 2 then
    local subcs = I:GetAllSubConstructs()
    for idx=1, #subcs do -- for some reason not an actual table so can't use pairs
      local sub = subcs[idx]
      for i=0, I:GetWeaponCountOnSubConstruct(sub) - 1 do
        if c == 0 then break end
        if I:GetWeaponBlockInfoOnSubConstruct(sub, i).CustomName == name then
          table.insert(weapons, {subIdx = sub, wpnIdx = i})
          if DEBUG then I:Log("found weapon "..name.." on subobj "..sub..", type "..I:GetWeaponInfoOnSubConstruct(sub, i).WeaponType) end
          c = c - 1
        end
      end
    end
  end
  if DEBUG then I:Log("weapon count: "..#weapons) end
  return weapons
end

--[[
  Arguments:
    I - the variable passed to Update
    name - the name of the subconstructs to search for
           can be set by pressing shift-N in build mode
    count - the number of subconstructs to search for
  Returns:
    subobjs - a list of subconstruct IDs where each ID
              corresponds to a subconstruct with a matching name
]]--
function BlockUtil.getSubConstructsByName(I, name, count)
	if DEBUG then I:Log("searching for "..name) end
  local subcs = I:GetAllSubConstructs()
  local subobjs = {}
  count = count or -1
  local c = count
  for idx=1, #subcs do -- for some reason not an actual table so can't use pairs
    local sub = subcs[idx]
    if c == 0 then break end
    if I:GetSubConstructInfo(sub).CustomName == name then
      table.insert(subobjs, sub)
      if DEBUG then I:Log("found subobj "..name) end
      c = c - 1
    end
  end
  if DEBUG then I:Log("subobj count: "..#subobjs) end
  return subobjs
end

--[[
  Arguments:
    I - the variable passed to Update
    type - the type of block to search for
           see help menu in lua block for list
    name - the name of the components to search for
           can be set by pressing shift-N in build mode
    count - the number of blocks to search for
  Returns:
    comps - a list of block indices where each index
              corresponds to a block with a matching name
]]--
function BlockUtil.getBlocksByName(I, name, id, count)
	if DEBUG then I:Log("searching for "..name) end
  local comps = {}
  count = count or -1
  local c = count
  for idx=0, I:Component_GetCount(id) - 1 do
    if c == 0 then break end
    if I:Component_GetBlockInfo(id, idx).CustomName == name then
      table.insert(comps, idx)
      if DEBUG then I:Log("found component "..name) end
      c = c - 1
    end
  end
  if DEBUG then I:Log("component count: "..#comps) end
  return comps
end

--[[
  If you have many names to search for, instead of
  searching for each name individually, you can populate
  a table containing all blocks grouped by their name.

  Unnamed blocks will be placed in the array proprtion of
  the table (i.e. t[1], t[2], ...)

  For example:
  {
    3, 14, 15
    "main": {2, 7, 18}
    "secondary": {1, 4}
  }

  Arguments:
    I - the variable passed to Update
    mode - the same as getWeaponsByName
  Returns:
    weapons - a table with weapons grouped by name. Each weapon
              is in the format {subIdx, wpnIdx}.
              See getWeaponsByName for more
]]--
function BlockUtil.populateWeaponsByName(I, mode)
	if DEBUG then I:Log("populating all weapons, mode "..mode) end
  local weapons = {}
  for idx=0, I:GetWeaponCount() - 1 do
    local name = I:Component_GetBlockInfo(type, idx).CustomName
    if name and name ~= '' then
      weapons[name] = weapons[name] or {}
      table.insert(weapons[name], {subIdx = nil, wpnIdx = idx})
      if DEBUG then I:Log("found weapon "..name.." on hull, type "..I:GetWeaponInfo(idx).WeaponType) end
    else
      table.insert(weapons, {subIdx = nil, wpnIdx = idx})
      if DEBUG then I:Log("found unnamed weapon on hull, type "..I:GetWeaponInfo(idx).WeaponType) end
    end
  end
  if not mode or mode == 1 or mode == 2 then
    local subcs = I:GetAllSubConstructs()
    for idx=1, #subcs do
      local sub = subcs[idx]
      for i=0, I:GetWeaponCountOnSubConstruct(sub) - 1 do
        local name = I:Component_GetBlockInfo(type, i).CustomName
        if name and name ~= '' then
          weapons[name] = weapons[name] or {}
          table.insert(weapons[name], {subIdx = sub, wpnIdx = i})
          if DEBUG then I:Log("found weapon "..name.." on subobj "..sub..", type "..I:GetWeaponInfoOnSubConstruct(sub, i).WeaponType) end
        else
          table.insert(weapons, {subIdx = sub, wpnIdx = i})
          if DEBUG then I:Log("found unnamed weapon on subobj "..sub..", type "..I:GetWeaponInfoOnSubConstruct(sub, i).WeaponType) end
        end
      end
    end
  end
  if DEBUG then
    local count = 0
    for k, v in pairs(weapons) do
      if type(v) == "table" then
        count = count + #v
      else
        count = count + 1 end
    end
    I:Log("weapon count: "..count)
  end
  return weapons
end

--[[
  Arguments:
    I - the variable passed to Update
  Returns:
    subobjs - a table with names as keys and a list of subconstructs
              with that name as the value. See populateWeaponsByName
]]--
function BlockUtil.populateSubConstructsByName(I)
	if DEBUG then I:Log("populating all subconstructs") end
  local subcs = I:GetAllSubConstructs()
  local subobjs = {}
  for idx=1, #subcs do -- for some reason not an actual table so can't use pairs
    local sub = subcs[idx]
    local name = I:GetSubConstructInfo(sub).CustomName
    if name and name ~= '' then
      subobjs[name] = subobjs[name] or {}
      table.insert(subobjs[name], sub)
      if DEBUG then I:Log("found subobj "..name) end
    else
      table.insert(subobjs, sub)
      if DEBUG then I:Log("found unnamed subobj") end
    end
  end
  if DEBUG then
    local count = 0
    for k, v in pairs(subobjs) do
      if type(v) == "table" then
        count = count + #v
      else
        count = count + 1 end
    end
    I:Log("subobject count: "..count)
  end
  return subobjs
end

--[[
  Arguments:
    I - the variable passed to Update
    id - the type of block to search for
         see help menu in lua block for list
  Returns:
    comps - a table with names as keys and a list of indices
            corresponding to blocks with that name as the value
            See populateWeaponsByName
]]--
function BlockUtil.populateBlocksByName(I, id)
	if DEBUG then I:Log("populating all blocks of type "..id) end
  local comps = {}
  for idx=0, I:Component_GetCount(id) - 1 do
    local name = I:Component_GetBlockInfo(id, idx).CustomName
    if name and name ~= '' then
      comps[name] = comps[name] or {}
      table.insert(comps[name], idx)
      if DEBUG then I:Log("found component "..name) end
    else
      table.insert(comps, idx)
      if DEBUG then I:Log("found unnamed component of type "..id) end
    end
  end
  if DEBUG then
    local count = 0
    for k, v in pairs(comps) do
      if type(v) == "table" then
        count = count + #v
      else
        count = count + 1 end
    end
    I:Log("component count: "..count)
  end
  return comps
end

--[[
  FtD's lua API has different functions depending on whether the weapon is on
  a subconstruct or not, resulting in duplicated code. This condenses them
  into one function.

  Arguments:
    I - the variable passed to Update
    weapon - a table containing subIdx and wpnIdx fields
             subIdx is the subconstruct identifier of the subconstruct the
               weapon is on, or nil if it is on the main hull
             wpnIdx is the weapon index
  Returns:
    info - the weapon info for the specified weapon
]]--
function BlockUtil.getWeaponInfo(I, weapon)
  if weapon.subIdx then
    return I:GetWeaponInfoOnSubConstruct(weapon.subIdx, weapon.wpnIdx)
  end
  return I:GetWeaponInfo(weapon.wpnIdx)
end

--[[
  Arguments:
    see getWeaponInfo
  Returns:
    weaponBlockInfo - the BlockInfo corresponding to the weapon
]]--
function BlockUtil.getWeaponBlockInfo(I, weapon)
  if weapon.subIdx then
    return I:GetWeaponBlockInfoOnSubConstruct(weapon.subIdx, weapon.wpnIdx)
  end
  return I:GetWeaponBlockInfo(weapon.wpnIdx)
end

--[[
  Arguments:
    I, weapon - see getWeaponInfo
    dir - the direction to aim in as a Vector3 in world space
    slot - the weapon slot to control
]]--
function BlockUtil.aimWeapon(I, weapon, dir, slot)
  if weapon.subIdx then
    I:AimWeaponInDirectionOnSubConstruct(weapon.subIdx, weapon.wpnIdx, dir.x, dir.y, dir.z, slot)
  else
    I:AimWeaponInDirection(weapon.wpnIdx, dir.x, dir.y, dir.z, slot)
  end
end

--[[
  Arguments:
    I, weapon, slot - see aimWeapon
  Returns:
    fired - whether the weapon fired or not
]]--
function BlockUtil.fireWeapon(I, weapon, slot)
  if weapon.subIdx then
    return I:FireWeaponOnSubConstruct(weapon.subIdx, weapon.wpnIdx, slot)
  end
  return I:FireWeapon(weapon.wpnIdx, slot)
end

--[[
  Arguments:
    I - the variable passed to Update
    mainframe - the mainframe index to use
    priorityFunc (optional) - a function that takes I and a
                                target and returns its priority.
                                Uses AI priority if empty
  Returns:
    target - the highest priority target
    priority - the priority of that target
--]]
function Combat.pickTarget(I, mainframe, priorityFunc)
  priorityFunc = priorityFunc or function(_, tar) return tar.Priority end
  local target, priority
  for i in MathUtil.range(I:GetNumberOfTargets(mainframe)) do
    local tar = I:GetTargetInfo(mainframe, i)
    local newPri = priorityFunc(I, tar)
    if not target or newPri > priority then
      target = tar
      priority = newPri
    end
  end
  return target
end

--[[
  Arguments:
    I - the variable passed to Update
    direction - the global direction of fire
    wepId - the ID of the weapon
    subObjId (optional) - if the weapon is on a subobject,
                            the ID of the subobject. If it
                            is on nested subobjects, the one
                            directly connected to the weapon
  Returns:
    valid - whether or not the direction is within the firing constraints
--]]
function Combat.CheckConstraints(I, direction, wepId, subObjId)
  local con
  if subObjId then
    con = I:GetWeaponConstraintsOnSubConstruct(subObjId, wepId)
  else
    con = I:GetWeaponConstraints(wepId)
  end
  if not con or not con.Valid then return true end
  local fore = I:GetConstructForwardVector()
  local up = I:GetConstructUpVector()
  local constructRot = Quaternion.LookRotation(fore, up)
  direction = Quaternion.Inverse(constructRot) * direction
  if con.InParentConstructSpace and subObjId then
    local rot = I:GetSubConstructInfo(subObjId).localRotation
    direction = Quaternion.inverse(rot) * direction
  end
  local azi = MathUtil.angleOnPlane(Vector3.forward, direction, Vector3.up)
  local aziDir = direction
  aziDir.y = 0
  local ele = Mathf.Atan2(direction.y, aziDir.magnitude)
  local aziValid = azi > con.MinAzimuth and azi < con.MaxAzimuth
  local eleValid = ele > con.MinElevation and ele < con.MaxElevation
  if con.FlipAzimuth then aziValid = not aziValid end
  if aziValid and eleValid then return true end
  azi = azi + 180
  ele = 180 - ele
  if ele > 180 then ele = ele - 360 end
  if ele < -180 then ele = ele + 360 end
  aziValid = azi > con.MinAzimuth and azi < con.MaxAzimuth
  eleValid = ele > con.MinElevation and ele < con.MaxElevation
  if con.FlipAzimuth then aziValid = not aziValid end
  if aziValid and eleValid then return true end
  return false
end

function StringUtil.LogVector(I, vec, label)
  I:Log(label.."("..vec.x..", "..vec.y..", "..vec.z..")")
end

--[[
does not work because the Lua interpreter used by From The Depths
does not support table.concat

function StringUtil.LogTable(I, tab, label, depth, indent)
  depth = depth or 0
  indent = indent or string.rep("  ", depth)
  local lines = {}
  for k, v in pairs(tab) do
    if type(v) == "table" then
      lines.insert(k..": "..StringUtil.LogTable(I, v, "", depth + 1, indent))
    else
      lines.insert(k..": "..v)
    end
  end
  return label.."{\n"..indent..table.concat(lines, "\n"..indent).."\n}"
end
]]
