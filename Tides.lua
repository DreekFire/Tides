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

Linalg = Linalg or {}

-- Generic NdArray linear algebra.  Axes and indices exposed by this module
-- are zero-based, while Lua tables holding permutation entries remain
-- ordinary one-based tables.

function Linalg.setArrayBackend(backend)
    Linalg.Array = backend
end

local function matrixShape(matrix)
    assert(matrix and matrix.shape and matrix.data, "expected an NdArray")
    assert(#matrix.shape == 2, "expected a two-dimensional array")
    return matrix.shape[1], matrix.shape[2]
end

local function at(matrix, row, col)
    return matrix.data[(matrix.offset or 0) + row * matrix.stride[1]
        + col * matrix.stride[2] + 1]
end

local function put(matrix, row, col, value)
    matrix.data[(matrix.offset or 0) + row * matrix.stride[1]
        + col * matrix.stride[2] + 1] = value
end

local function identity(size)
    return NdArray.eye(size)
end

local function swapRows(matrix, first, second, columns)
    if first == second then return end
    for col = 0, columns - 1 do
        local left, right = at(matrix, first, col), at(matrix, second, col)
        put(matrix, first, col, right)
        put(matrix, second, col, left)
    end
end

function Linalg.dot(a, b)
    assert(#a.shape == 1 and #b.shape == 1, "dot currently requires vectors")
    assert(a.shape[1] == b.shape[1], "dot dimensions do not match")
    local result = NdArray.matMul(a, b)
    return result
end

function Linalg.norm(a)
    assert(a and a.shape, "norm expects an NdArray")
    local sum = 0
    for _, values in NdArray.nditer(a, nil, "C") do sum = sum + values[1] * values[1] end
    return math.sqrt(sum)
end

-- Partial-pivoted Doolittle factorization.  P is returned as a permutation
-- matrix, so the identity is P*A = L*U.
function Linalg.lu(matrix, pivoting)
    local rows, cols = matrixShape(matrix)
    assert(rows == cols, "lu currently requires a square matrix")
    pivoting = pivoting ~= false
    local n, u, l, permutation = rows, NdArray.copy(matrix), identity(rows), {}
    for row = 0, n - 1 do permutation[row + 1] = row end
    for col = 0, n - 1 do
        local pivotRow, pivot = col, math.abs(at(u, col, col))
        if pivoting then
            for row = col + 1, n - 1 do
                local candidate = math.abs(at(u, row, col))
                if candidate > pivot then pivot, pivotRow = candidate, row end
            end
        end
        assert(pivot > 1e-14, "lu encountered a singular matrix")
        if pivotRow ~= col then
            swapRows(u, col, pivotRow, n)
            permutation[col + 1], permutation[pivotRow + 1] = permutation[pivotRow + 1], permutation[col + 1]
            for previous = 0, col - 1 do
                local left, right = at(l, col, previous), at(l, pivotRow, previous)
                put(l, col, previous, right)
                put(l, pivotRow, previous, left)
            end
        end
        local diagonal = at(u, col, col)
        for row = col + 1, n - 1 do
            local multiplier = at(u, row, col) / diagonal
            put(l, row, col, multiplier)
            put(u, row, col, 0)
            for current = col + 1, n - 1 do
                put(u, row, current, at(u, row, current) - multiplier * at(u, col, current))
            end
        end
    end
    local p = NdArray.zeros({n, n})
    for row = 0, n - 1 do put(p, row, permutation[row + 1], 1) end
    return p, l, u
end

local function permutationRows(p, n)
    if type(p) == "table" and p.shape then
        local rows, cols = matrixShape(p)
        assert(rows == n and cols == n, "permutation matrix dimensions do not match")
        local result = {}
        for row = 0, n - 1 do
            local selected
            for col = 0, n - 1 do if at(p, row, col) ~= 0 then selected = col; break end end
            assert(selected ~= nil, "invalid permutation matrix")
            result[row + 1] = selected
        end
        return result
    end
    assert(type(p) == "table" and #p == n, "invalid permutation")
    return p
end

function Linalg.solveLU(p, l, u, b)
    local n = matrixShape(l)
    local ur, uc = matrixShape(u)
    assert(ur == n and uc == n, "LU dimensions do not match")
    assert(#b.shape == 1 and b.shape[1] == n, "solveLU currently requires a vector right hand side")
    local permutation = permutationRows(p, n)
    local y, x = {}, {}
    for row = 0, n - 1 do
        local value = b.data[(b.offset or 0) + permutation[row + 1] * b.stride[1] + 1]
        for col = 0, row - 1 do value = value - at(l, row, col) * y[col + 1] end
        y[row + 1] = value
    end
    for row = n - 1, 0, -1 do
        local value = y[row + 1]
        for col = row + 1, n - 1 do value = value - at(u, row, col) * x[col + 1] end
        x[row + 1] = value / at(u, row, row)
    end
    return NdArray.array(x)
end

-- Householder QR.  Q is square (m by m), R is m by n, and A = Q*R.
function Linalg.qr(matrix)
    local rows, cols = matrixShape(matrix)
    local q, r = identity(rows), NdArray.copy(matrix)
    local limit = math.min(rows, cols)
    for k = 0, limit - 1 do
        local length, norm = rows - k, 0
        for i = 0, length - 1 do
            local value = at(r, k + i, k)
            norm = norm + value * value
        end
        norm = math.sqrt(norm)
        if norm > 1e-14 then
            local v, alpha, squared = {}, (at(r, k, k) >= 0 and -norm or norm), 0
            for i = 0, length - 1 do
                v[i + 1] = at(r, k + i, k)
                if i == 0 then v[i + 1] = v[i + 1] - alpha end
                squared = squared + v[i + 1] * v[i + 1]
            end
            local beta = 2 / squared
            for col = k, cols - 1 do
                local projection = 0
                for i = 0, length - 1 do projection = projection + v[i + 1] * at(r, k + i, col) end
                projection = beta * projection
                for i = 0, length - 1 do put(r, k + i, col, at(r, k + i, col) - projection * v[i + 1]) end
            end
            for row = 0, rows - 1 do
                local projection = 0
                for i = 0, length - 1 do projection = projection + at(q, row, k + i) * v[i + 1] end
                projection = beta * projection
                for i = 0, length - 1 do put(q, row, k + i, at(q, row, k + i) - projection * v[i + 1]) end
            end
        end
    end
    return q, r
end

function Linalg.solveQR(q, r, b)
    local rows, cols = matrixShape(r)
    local qr, qc = matrixShape(q)
    assert(qr == qc and qr == rows and cols == rows, "solveQR currently requires a square factorization")
    assert(#b.shape == 1 and b.shape[1] == rows, "solveQR currently requires a vector right hand side")
    local y, x = {}, {}
    for row = 0, rows - 1 do
        local value = 0
        for col = 0, rows - 1 do value = value + at(q, col, row) * b.data[(b.offset or 0) + col * b.stride[1] + 1] end
        y[row + 1] = value
    end
    for row = rows - 1, 0, -1 do
        local value = y[row + 1]
        for col = row + 1, rows - 1 do value = value - at(r, row, col) * x[col + 1] end
        x[row + 1] = value / at(r, row, row)
    end
    return NdArray.array(x)
end

function Linalg.cholesky(matrix)
    local rows, cols = matrixShape(matrix)
    assert(rows == cols, "cholesky requires a square matrix")
    local l = NdArray.zeros({rows, rows})
    for row = 0, rows - 1 do
        for col = 0, row do
            local value = at(matrix, row, col)
            for k = 0, col - 1 do value = value - at(l, row, k) * at(l, col, k) end
            if row == col then
                assert(value > 0, "cholesky requires a positive-definite matrix")
                put(l, row, col, math.sqrt(value))
            else
                put(l, row, col, value / at(l, col, col))
            end
        end
    end
    return l
end

local function offDiagonalNorm(matrix)
    local n = matrixShape(matrix)
    local result = 0
    for row = 0, n - 1 do for col = 0, n - 1 do if row ~= col then
        local value = at(matrix, row, col); result = result + value * value
    end end end
    return math.sqrt(result)
end

-- Symmetric QR iteration.  Eigenvectors are returned as columns of vectors.
function Linalg.eigQr(matrix, iterLim, tolerance)
    local rows, cols = matrixShape(matrix)
    assert(rows == cols, "eigQr requires a square matrix")
    iterLim, tolerance = iterLim or 100, tolerance or 1e-10
    local current, vectors, converged = NdArray.copy(matrix), identity(rows), false
    for _ = 1, iterLim do
        local q, r = Linalg.qr(current)
        current = NdArray.matMul(r, q)
        vectors = NdArray.matMul(vectors, q)
        if offDiagonalNorm(current) < tolerance then converged = true; break end
    end
    local order = {}
    for i = 0, rows - 1 do order[i + 1] = i end
    table.sort(order, function(a, b) return at(current, a, a) > at(current, b, b) end)
    local values, sortedVectors = {}, NdArray.zeros({rows, rows})
    for newColumn = 0, rows - 1 do
        local oldColumn = order[newColumn + 1]
        values[newColumn + 1] = at(current, oldColumn, oldColumn)
        for row = 0, rows - 1 do put(sortedVectors, row, newColumn, at(vectors, row, oldColumn)) end
    end
    return values, sortedVectors, {converged = converged, iterations = iterLim}
end

function Linalg.eigSymm(matrix, iterLim, tolerance)
    return Linalg.eigQr(matrix, iterLim, tolerance)
end

function Linalg.eigTop(matrix, iterLim)
    local rows, cols = matrixShape(matrix)
    assert(rows == cols, "eigTop requires a square matrix")
    local vector = NdArray.ones({rows})
    local length = Linalg.norm(vector)
    for i = 1, vector.size do vector.data[i] = vector.data[i] / length end
    for _ = 1, (iterLim or 50) do
        local nextVector = NdArray.matMul(matrix, vector)
        local nextLength = Linalg.norm(nextVector)
        assert(nextLength > 1e-14, "power iteration reached a zero vector")
        for i = 1, nextVector.size do nextVector.data[i] = nextVector.data[i] / nextLength end
        vector = nextVector
    end
    return Linalg.dot(vector, NdArray.matMul(matrix, vector)), vector
end

function Linalg.solvePCG()
    error("solvePCG is not implemented")
end

function Linalg.fft()
    error("fft is not implemented")
end

function Linalg.svd()
    error("svd is not implemented")
end

function Linalg.eigTopK()
    error("eigTopK is not implemented")
end

Linalg.eigSymm2 = Linalg.eigSymm
Linalg.eigSymm3 = Linalg.eigSymm

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

function Matrix3.getCol(mat, col)
  local ret = {}
  for row=1,3 do
    ret[row] = mat[(row - 1) * 3 + col]
  end
  return ret
end

function Matrix3.setCol(mat, col, v)
  for row=1,3 do
    mat[(row - 1) * 3 + col] = v[row]
  end
end

function Matrix3.getRow(mat, row)
  local ret = {}
  for col=1,3 do
    ret[row] = mat[(row - 1) * 3 + col]
  end
  return ret
end

function Matrix3.setRow(mat, row, v)
  for col=1,3 do
    mat[(row - 1) * 3 + col] = v[col]
  end
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

-- swaps rows r1 and r2 in place
function Matrix3.rowSwap(mat, r1, r2)
  for col=1,3 do
    local v = mat[(r1 - 1) * 3 + col]
    mat[(r1 - 1) * 3 + col] = mat[(r2 - 1) * 3 + col]
    mat[(r2 - 1) * 3 + col] = v
  end
end

-- scale row r1 by s
function Matrix3.rowScale(mat, r, s)
  for col=1,3 do
    mat[(r - 1) * 3 + col] = s * mat[(r - 1) * 3 + col]
  end
end

-- add s times r1 to r2
function Matrix3.rowAdd(mat, r1, r2, s)
  for col=1,3 do
    mat[(r2 - 1) * 3 + col] = mat[(r2 - 1) * 3 + col] + s * mat[(r1 - 1) * 3 + col]
  end
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

NdArray = {}

local function ndCopyShapeLocal(shape)
    local result = {}
    for axis, size in ipairs(shape) do result[axis] = size end
    return result
end

local function ndSizeLocal(shape)
    local size = 1
    for _, dimension in ipairs(shape) do size = size * dimension end
    return size
end

local function ndViewLocal(arr, shape, stride)
    return {
        shape = ndCopyShapeLocal(shape),
        stride = ndCopyShapeLocal(stride),
        data = arr.data,
        offset = arr.offset or 0,
        size = ndSizeLocal(shape),
    }
end

local function ndAxisLocal(axis, ndim)
    assert(type(axis) == "number" and axis == math.floor(axis), "axis must be an integer")
    if axis < 0 then axis = ndim + axis end
    assert(axis >= 0 and axis < ndim, "axis is out of bounds")
    return axis
end

local function ndFlattenLocal(value, data)
    if type(value) ~= "table" then
        data[#data + 1] = value
        return
    end
    for index = 1, #value do ndFlattenLocal(value[index], data) end
end

local function ndInferShapeLocal(value)
    if type(value) ~= "table" then return {} end
    local childShape = ndInferShapeLocal(value[1])
    for index = 2, #value do
        local shape = ndInferShapeLocal(value[index])
        assert(#shape == #childShape, "array values must be rectangular")
        for axis = 1, #shape do assert(shape[axis] == childShape[axis], "array values must be rectangular") end
    end
    local shape = {#value}
    for axis, size in ipairs(childShape) do shape[#shape + 1] = size end
    return shape
end

function NdArray.array(lst)
    if type(lst) == "table" and lst.shape and lst.data then return NdArray.copy(lst) end
    local shape = ndInferShapeLocal(lst)
    local result = NdArray.empty(shape)
    ndFlattenLocal(lst, result.data)
    assert(#result.data == result.size, "array values must be rectangular")
    return result
end

NdArray.Array = NdArray.array

function NdArray.zeros(shape)
    return NdArray.full(shape, 0)
end

function NdArray.ones(shape)
    return NdArray.full(shape, 1)
end

function NdArray.full(shape, fill)
    local arr = NdArray.empty(shape)
    local data = arr.data
    for i=1, arr.size do
        data[i] = fill
    end
    return arr
end

function NdArray.empty(shape)
    local size = 1
    local stride = {}
    local shape_copy = {}
    for i, dim in ipairs(shape) do
        size = size * dim
        for j, s in ipairs(stride) do
            stride[j] = stride[j] * dim
        end
        stride[i] = 1
        shape_copy[i] = dim
    end
    local data = {}
    local arr = {
        size = size,
        stride = stride,
        shape = shape_copy,
        data = data,
        offset = 0,
    }
    setmetatable(arr, NdArray.meta)
    return arr
end

function NdArray.zerosLike(arr)
    return NdArray.zeros(arr.shape)
end

function NdArray.onesLike(arr)
    return NdArray.ones(arr.shape)
end

function NdArray.fullLike(arr, fill)
    return NdArray.full(arr.shape, fill)
end

function NdArray.emptyLike(arr)
    return NdArray.empty(arr.shape)
end

function NdArray.arange(a, b, c)
    assert(type(a) == "number", "arange requires at least one number")
    local start, stop = 0, a
    if b ~= nil then start, stop = a, b end
    local step = c or (start <= stop and 1 or -1)
    assert(step ~= 0, "arange step must not be zero")
    local data, value = {}, start
    if step > 0 then
        while value < stop do data[#data + 1], value = value, value + step end
    else
        while value > stop do data[#data + 1], value = value, value + step end
    end
    return NdArray.array(data)
end

function NdArray._callbackIter(func, dst, ...)
    assert(type(dst) == "table" and dst.shape and dst.data, "expected an NdArray destination")
    local operands = {dst, ...}
    local coreAxes = {}
    for operand = 1, #operands do coreAxes[operand] = {} end
    local iterator = NdArray.coreIter(operands, coreAxes)
    local indices, views = iterator()
    while indices do
        local args = {}
        for operand = 2, #views do args[operand - 1] = views[operand].data[views[operand].offset + 1] end
        views[1].data[views[1].offset + 1] = func(table.unpack(args, 1, #args))
        indices, views = iterator()
    end
    return dst
end

function NdArray.ndindex(shape)
    local ndim = #shape
    local start_idxs = {}
    for i=1,ndim do
        start_idxs[i] = 0
    end
    start_idxs[ndim] = -1
    return function()
        for d=ndim, 1, -1 do
            if start_idxs[d] == shape[d] then
                if d == 1 then return nil end
                start_idxs[d] = 0
                start_idxs[d - 1] = start_idxs[d - 1] + 1
            else
                start_idxs[d] = start_idxs[d] + 1
                break
            end
        end
        return start_idxs
    end
end

function NdArray._strideIter1(arr)
    local shape = arr.shape
    local stride = arr.stride
    local ndim = #shape
    local idxs = {}
    for i=1,ndim do
        idxs[i] = 1
    end
    local ofs = -stride[ndim]
    local inner_limit = shape[ndim]
    local inner_stride = stride[ndim]
    local inner_idx = 0
    return function()
        if inner_idx < inner_limit then
            inner_idx = inner_idx + 1
            ofs = ofs + inner_stride
            idxs[ndim] = inner_idx
            return idxs, ofs
        end
        inner_idx = 1
        ofs = ofs - inner_stride * (inner_limit - 1)
        idxs[ndim - 1] = idxs[ndim - 1] + 1
        for d=ndim - 1, 1, -1 do
            if idxs[d] < shape[d] then
                idxs[d] = idxs[d] + 1
                ofs = ofs + stride[d]
                break
            else
                if d == 1 then return nil end
                idxs[d] = 0
                idxs[d - 1] = idxs[d - 1] + 1
                ofs = ofs - stride[d] * shape[d]
                ofs = ofs + stride[d - 1]
            end
        end
        idxs[ndim] = 1
        return idxs, ofs
    end
end

function NdArray._strideIter2(arr1, arr2)
    local shape = arr1.shape
    local stride1 = arr1.stride
    local stride2 = arr2.stride
    local ndim = #shape
    local idxs = {}
    for i=1,ndim do
        idxs[i] = 1
    end
    local ofs1 = -stride1[ndim]
    local ofs2 = -stride2[ndim]
    local inner_limit = shape[ndim]
    local inner_stride1 = stride1[ndim]
    local inner_stride2 = stride2[ndim]
    local inner_idx = 0
    return function()
        if inner_idx < inner_limit then
            inner_idx = inner_idx + 1
            ofs1 = ofs1 + inner_stride1
            ofs2 = ofs2 + inner_stride2
            idxs[ndim] = inner_idx
            return idxs, ofs1, ofs2
        end
        inner_idx = 1
        ofs1 = ofs1 - inner_stride1 * (inner_limit - 1)
        ofs2 = ofs2 - inner_stride2 * (inner_limit - 1)
        idxs[ndim - 1] = idxs[ndim - 1] + 1
        for d=ndim - 1, 1, -1 do
            if idxs[d] < shape[d] then
                idxs[d] = idxs[d] + 1
                ofs1 = ofs1 + stride1[d]
                ofs2 = ofs2 + stride2[d]
                break
            else
                if d == 1 then return nil end
                idxs[d] = 0
                idxs[d - 1] = idxs[d - 1] + 1
                ofs1 = ofs1 - stride1[d] * shape[d]
                ofs1 = ofs1 + stride1[d - 1]
                ofs2 = ofs2 - stride2[d] * shape[d]
                ofs2 = ofs2 + stride2[d - 1]
            end
        end
        idxs[ndim] = 1
        return idxs, ofs1, ofs2
    end
end

function NdArray._strideIter3(arr1, arr2, arr3)
    local shape = arr1.shape
    local stride1 = arr1.stride
    local stride2 = arr2.stride
    local stride3 = arr3.stride
    local ndim = #shape
    local idxs = {}
    for i=1,ndim do
        idxs[i] = 1
    end
    local ofs1 = -stride1[ndim]
    local ofs2 = -stride2[ndim]
    local ofs3 = -stride3[ndim]
    local inner_limit = shape[ndim]
    local inner_stride1 = stride1[ndim]
    local inner_stride2 = stride2[ndim]
    local inner_stride3 = stride3[ndim]
    local inner_idx = 0
    return function()
        if inner_idx < inner_limit then
            inner_idx = inner_idx + 1
            ofs1 = ofs1 + inner_stride1
            ofs2 = ofs2 + inner_stride2
            ofs3 = ofs3 + inner_stride3
            idxs[ndim] = inner_idx
            return idxs, ofs1, ofs2, ofs3
        end
        inner_idx = 1
        ofs1 = ofs1 - inner_stride1 * (inner_limit - 1)
        ofs2 = ofs2 - inner_stride2 * (inner_limit - 1)
        ofs3 = ofs3 - inner_stride3 * (inner_limit - 1)
        idxs[ndim - 1] = idxs[ndim - 1] + 1
        for d=ndim - 1, 1, -1 do
            if idxs[d] < shape[d] then
                idxs[d] = idxs[d] + 1
                ofs1 = ofs1 + stride1[d]
                ofs2 = ofs2 + stride2[d]
                ofs3 = ofs3 + stride3[d]
                break
            else
                if d == 1 then return nil end
                idxs[d] = 0
                idxs[d - 1] = idxs[d - 1] + 1
                ofs1 = ofs1 - stride1[d] * shape[d]
                ofs1 = ofs1 + stride1[d - 1]
                ofs2 = ofs2 - stride2[d] * shape[d]
                ofs2 = ofs2 + stride2[d - 1]
                ofs3 = ofs3 - stride3[d] * shape[d]
                ofs3 = ofs3 + stride3[d - 1]
            end
        end
        idxs[ndim] = 1
        return idxs, ofs1, ofs2, ofs3
    end
end

function NdArray._strideIter(shape, strides, baseOffsets)
    local narr = #strides
    local ndim = #shape
    local idxs = {}
    for i=1,ndim - 1 do
        idxs[i] = 1
    end
    local offsets = {}
    for i=1,narr do
        offsets[i] = baseOffsets[i] - strides[i][ndim]
    end
    local inner_limit = shape[ndim]
    local inner_idx = 0
    return function()
        if inner_idx < inner_limit then
            inner_idx = inner_idx + 1
            for i=1,narr do
                offsets[i] = offsets[i] + strides[i][ndim]
            end
            return offsets
        end
        inner_idx = 0
        for i=1,narr do
            offsets[i] = offsets[i] - strides[i][ndim] * (inner_limit - 1)
        end
        for d=ndim - 1, 1, -1 do
            if idxs[d] < shape[d] then
                idxs[d] = idxs[d] + 1
                for i=1,narr do
                    offsets[i] = offsets[i] + strides[i][d]
                end
                break
            else
                if d == 1 then return nil end
                idxs[d] = 0
                idxs[d - 1] = idxs[d - 1] + 1
                for i=1,narr do
                    offsets[i] = offsets[i] - strides[i][d] * (shape[d])
                    offsets[i] = offsets[i] + strides[i][d - 1]
                end
            end
        end
        return offsets
    end
end

function NdArray.copy(arr)
    assert(type(arr) == "table" and arr.shape and arr.stride and arr.data, "expected an NdArray")
    local ret = NdArray.empty(arr.shape)
    local position = 1
    for _, values in NdArray.nditer(arr, nil, "C") do
        ret.data[position], position = values[1], position + 1
    end
    return ret
end

function NdArray.copyTo(dst, src, where)
    if where == false then return end
    if where == true then
        dst, src = table.unpack(NdArray.broadcastArrays({dst, src}))
        for offsets in NdArray._strideIter(dst.shape, {dst.stride, src.stride}, {dst.offset or 0, src.offset or 0}) do
            dst.data[offsets[1]] = src.data[offsets[2]]
        end
    else
        dst, src, where = table.unpack(NdArray.broadcastArrays({dst, src, where}))
        for offsets in NdArray._strideIter(dst.shape, {dst.stride, src.stride, where.stride},
            {dst.offset or 0, src.offset or 0, where.offset or 0}) do
            if where.data[offsets[3]] then
                dst.data[offsets[1]] = src.data[offsets[2]]
            end
        end
    end
end

function NdArray._reshapeIsView(shape1, shape2, stride1)
    local k = 1
    local compressedShape = {}
    local compressedStride = {}
    local ndim1 = #shape1
    local currentShape = shape1[1]
    for i=1,ndim1-1 do
        if stride1[i] == shape1[i + 1] * stride1[i + 1] or shape1[i + 1] == 1 then
            currentShape = currentShape * shape1[i + 1]
        else
            compressedShape[k] = currentShape
            compressedStride[k] = stride1[k]
            k = k + 1
        end
    end

    local p = 1
    local ndim2 = #shape2
    local j = #compressedShape
    local outputStride = {}
    for i=ndim2,2,-1 do
        if p == 1 then
            outputStride[i] = compressedStride[j]
        else
            outputStride[i] = outputStride[i + 1] * shape2[i]
        end
        p = p * shape2[i]
        if p == compressedShape[j] then
            p = 1
            j = j - 1
        elseif p > compressedShape[j] then
            return false, nil
        end
    end
    if p == 1 then
        outputStride[1] = compressedStride[j]
    else
        outputStride[1] = outputStride[2] * shape2[1]
    end
    return true, outputStride
end

function NdArray.reshape(arr, ...)
    local shape = {...}
    if #shape == 1 and type(shape[1]) == "table" then shape = shape[1] end
    local unknown, knownSize = nil, 1
    for axis, dimension in ipairs(shape) do
        if dimension == -1 then
            assert(not unknown, "reshape permits only one inferred dimension")
            unknown = axis
        else
            assert(dimension >= 0 and dimension == math.floor(dimension), "invalid reshape dimension")
            knownSize = knownSize * dimension
        end
    end
    assert(not unknown or knownSize > 0 and arr.size % knownSize == 0, "reshape size is incompatible")
    if unknown then shape[unknown] = arr.size / knownSize else assert(knownSize == arr.size, "reshape size is incompatible") end
    local contiguous, expected = true, 1
    for axis = #arr.shape, 1, -1 do
        if arr.shape[axis] ~= 1 and arr.stride[axis] ~= expected then contiguous = false end
        expected = expected * arr.shape[axis]
    end
    local source = contiguous and arr or NdArray.copy(arr)
    local stride, step = {}, 1
    for axis = #shape, 1, -1 do stride[axis], step = step, step * shape[axis] end
    return ndViewLocal(source, shape, stride)
end

function NdArray.transpose(arr, axis1, axis2)
    local ndim = #arr.shape
    local shape, stride = {}, {}
    if axis1 == nil then
        for axis = 1, ndim do
            shape[axis], stride[axis] = arr.shape[ndim - axis + 1], arr.stride[ndim - axis + 1]
        end
    else
        axis1, axis2 = ndAxisLocal(axis1, ndim), ndAxisLocal(axis2, ndim)
        shape, stride = ndCopyShapeLocal(arr.shape), ndCopyShapeLocal(arr.stride)
        shape[axis1 + 1], shape[axis2 + 1] = shape[axis2 + 1], shape[axis1 + 1]
        stride[axis1 + 1], stride[axis2 + 1] = stride[axis2 + 1], stride[axis1 + 1]
    end
    return ndViewLocal(arr, shape, stride)
end

function NdArray.permuteDims(arr, dims)
    local ndim = #arr.shape
    dims = dims or {}
    if #dims == 0 then for axis = 0, ndim - 1 do dims[axis + 1] = ndim - axis - 1 end end
    assert(#dims == ndim, "permutation rank mismatch")
    local shape, stride, seen = {}, {}, {}
    for position, axis in ipairs(dims) do
        axis = ndAxisLocal(axis, ndim)
        assert(not seen[axis], "permutation axes must be unique")
        seen[axis] = true
        shape[position], stride[position] = arr.shape[axis + 1], arr.stride[axis + 1]
    end
    return ndViewLocal(arr, shape, stride)
end

function NdArray.ravel(arr)
    return NdArray.reshape(arr, -1)
end

function NdArray.flatten(arr)
    local flat = NdArray.copy(arr)
    flat.stride = {1}
    flat.shape = {flat.size}
    return flat
end

function NdArray._elementwise(func, ...)
    local broadcasted = NdArray.broadcastArrays({...})
    local n = #broadcasted
    local ret = NdArray.empty(broadcasted[1].shape)
    local args = {}
    local position = 1
    for _, values in NdArray.nditer(broadcasted, nil, "C") do
        for i = 1, n do args[i] = values[i] end
        ret.data[position], position = func(table.unpack(args, 1, n)), position + 1
    end
    return ret
end

function NdArray.add(arr1, arr2)
    return NdArray._elementwise(function(a, b) return a + b end, arr1, arr2)
end

function NdArray.sub(arr1, arr2)
    return NdArray._elementwise(function(a, b) return a - b end, arr1, arr2)
end

function NdArray.mul(arr1, arr2)
    return NdArray._elementwise(function(a, b) return a * b end, arr1, arr2)
end

function NdArray.matMul(arr1, arr2)
    assert(arr1 and arr2 and arr1.shape and arr2.shape, "matMul expects NdArrays")
    local leftRank, rightRank = #arr1.shape, #arr2.shape
    assert(leftRank > 0 and rightRank > 0, "matMul operands must have at least one dimension")
    local leftVector, rightVector = leftRank == 1, rightRank == 1
    local left = leftVector and ndViewLocal(arr1, {1, arr1.shape[1]}, {0, arr1.stride[1]}) or arr1
    local right = rightVector and ndViewLocal(arr2, {arr2.shape[1], 1}, {arr2.stride[1], 0}) or arr2
    assert(left.shape[#left.shape] == right.shape[#right.shape - 1], "matMul core dimensions do not match")
    local leftAxes, rightAxes = {}, {}
    leftAxes[1], leftAxes[2] = #left.shape - 2, #left.shape - 1
    rightAxes[1], rightAxes[2] = #right.shape - 2, #right.shape - 1
    local iterator = NdArray.coreIter({left, right}, {leftAxes, rightAxes})
    local firstIndices, firstViews = iterator()
    -- coreIter's loop indices are the broadcasted batch shape; obtain its
    -- exact shape from the operands so zero-size batches are preserved.
    local leftBatch, rightBatch = {}, {}
    for axis = 1, #left.shape - 2 do leftBatch[axis] = left.shape[axis] end
    for axis = 1, #right.shape - 2 do rightBatch[axis] = right.shape[axis] end
    local loopShape = NdArray.broadcastShapes({leftBatch, rightBatch})
    local n, k, m = left.shape[#left.shape - 1], left.shape[#left.shape], right.shape[#right.shape]
    local outputShape = ndCopyShapeLocal(loopShape)
    if leftVector and rightVector then
        local sum = 0
        local views = firstViews
        for t = 0, k - 1 do
            sum = sum + views[1].data[views[1].offset + t * views[1].stride[2] + 1]
                * views[2].data[views[2].offset + t * views[2].stride[1] + 1]
        end
        return sum
    end
    if leftVector then outputShape[#outputShape + 1] = m
    elseif rightVector then outputShape[#outputShape + 1] = n
    else outputShape[#outputShape + 1], outputShape[#outputShape + 2] = n, m end
    local result = NdArray.empty(outputShape)
    local indices, views = firstIndices, firstViews
    while indices do
        local outputBase = 0
        for axis = 1, #loopShape do outputBase = outputBase + indices[axis] * result.stride[axis] end
        if leftVector then
            for col = 0, m - 1 do
                local sum = 0
                for t = 0, k - 1 do
                    sum = sum + views[1].data[views[1].offset + t * views[1].stride[2] + 1]
                        * views[2].data[views[2].offset + t * views[2].stride[1] + col * views[2].stride[2] + 1]
                end
                result.data[outputBase + col * result.stride[#result.shape] + 1] = sum
            end
        elseif rightVector then
            for row = 0, n - 1 do
                local sum = 0
                for t = 0, k - 1 do
                    sum = sum + views[1].data[views[1].offset + row * views[1].stride[1] + t * views[1].stride[2] + 1]
                        * views[2].data[views[2].offset + t * views[2].stride[1] + 1]
                end
                result.data[outputBase + row * result.stride[#result.shape] + 1] = sum
            end
        else
            for row = 0, n - 1 do
                for col = 0, m - 1 do
                    local sum = 0
                    for t = 0, k - 1 do
                        sum = sum + views[1].data[views[1].offset + row * views[1].stride[1] + t * views[1].stride[2] + 1]
                            * views[2].data[views[2].offset + t * views[2].stride[1] + col * views[2].stride[2] + 1]
                    end
                    result.data[outputBase + row * result.stride[#result.shape - 1] + col * result.stride[#result.shape] + 1] = sum
                end
            end
        end
        indices, views = iterator()
    end
    return result
end

NdArray.matmul = NdArray.matMul
NdArray.MatMul = NdArray.matMul

function NdArray.div(arr1, arr2)
    return NdArray._elementwise(function(a, b) return a / b end, arr1, arr2)
end

function NdArray.map(func, arr)
    local res, position = NdArray.emptyLike(arr), 1
    for _, values in NdArray.nditer(arr, nil, "C") do
        res.data[position], position = func(values[1]), position + 1
    end
    return res
end

function NdArray.sqrt(arr)
    return NdArray.map(function(a) return math.sqrt(a) end, arr)
end

function NdArray.negative(arr)
    return NdArray.map(function(a) return -a end, arr)
end

function NdArray.vectorize(func, excluded, signature)
    if not signature then
        return function(...)
            local arg = {...}
            local arrs = {}
            for i, v in ipairs(arg) do
                if getmetatable(v) == NdArray.meta and (not excluded or not excluded[i]) then
                    arrs[#arrs + 1] = v
                else
                    arrs[#arrs + 1] = nil
                end
            end
            if #arrs == 0 then
                return func(...)
            else
                local ofs = 0
                arrs = NdArray.broadcastArrays(arrs)
                local ret = NdArray.emptyLike(arrs)
                local args = {}
                for idx, vals in NdArray.nditer(arrs) do
                    for i, v in ipairs(arg) do
                        if getmetatable(v) == NdArray.meta and (not excluded or not excluded[i]) then
                            args[i] = vals[i - ofs]
                        else
                            args[i] = v
                            ofs = ofs + 1
                        end
                    end
                    NdArray._setSingle(ret, idx, func(table.unpack(args)))
                end
            end
        end
    end
    local batchShape = {}
    for i, d in ipairs(arr.shape) do
        if inAxes == nil or i > #inAxes then
            batchShape[i] = d
        end
    end
    for i, ax in ipairs(inAxes) do
        batchShape[i] = 1
    end
    local batchIdx = {}
    local outputShape = {}
    for i, d in ipairs(batchShape) do
        outputShape[i] = d
    end
    local out
    for idx, _ in NdArray.ndindex(batchShape) do
        for i, k in ipairs(idx) do
            batchIdx[i] = k
        end
        for i, ax in ipairs(inAxes) do
            batchIdx[ax] = NdArray.sliceAll
        end
        local input = NdArray.get(arr, batchIdx)
        local output = func(input)
        if out == nil then
            if getmetatable(output) == NdArray.meta then
                for i, d in ipairs(output.shape) do
                    outputShape[#outputShape + 1] = d
                end
            end
            out = NdArray.empty(outputShape)
        end
        NdArray.set(out, idx, output)
    end
    if not keepdim then
        out = NdArray.squeeze(out, inAxes)
    end
    return out
end

function NdArray.reduce(func, arr, axis, keepdim)
    if axis == nil then
        local result, seen = nil, false
        for _, values in NdArray.nditer(arr, nil, "C") do
            result = result == nil and values[1] or func(result, values[1])
            seen = true
        end
        assert(seen, "cannot reduce an empty array")
        return result
    end
    axis = ndAxisLocal(axis, #arr.shape)
    local outputShape = {}
    for sourceAxis = 0, #arr.shape - 1 do
        if keepdim or sourceAxis ~= axis then
            outputShape[#outputShape + 1] = sourceAxis == axis and 1 or arr.shape[sourceAxis + 1]
        end
    end
    local result = NdArray.empty(outputShape)
    local iterator = NdArray.coreIter({arr}, {{axis}})
    local indices, views = iterator()
    while indices do
        local view, value = views[1], nil
        for i = 0, view.shape[1] - 1 do
            local current = view.data[view.offset + i * view.stride[1] + 1]
            value = value == nil and current or func(value, current)
        end
        assert(value ~= nil, "cannot reduce an empty axis")
        local outputOffset, loopPosition = 0, 1
        for sourceAxis = 0, #arr.shape - 1 do
            if sourceAxis ~= axis then
                local targetAxis = keepdim and sourceAxis or sourceAxis - (sourceAxis > axis and 1 or 0)
                outputOffset = outputOffset + indices[loopPosition] * result.stride[targetAxis + 1]
                loopPosition = loopPosition + 1
            end
        end
        result.data[outputOffset + 1] = value
        indices, views = iterator()
    end
    return result
end

function NdArray.sum(arr, axis, keepdim)
    return NdArray.reduce(function(a, b) return a + b end, arr, axis, keepdim)
end

function NdArray.prod(arr, axis, keepdim)
    return NdArray.reduce(function(a, b) return a * b end, arr, axis, keepdim)
end

function NdArray.max(arr, axis, keepdim)
    return NdArray.reduce(math.max, arr, axis, keepdim)
end

function NdArray.min(arr, axis, keepdim)
    return NdArray.reduce(math.min, arr, axis, keepdim)
end

function NdArray.mean(arr, axis, keepdim)
    local res = NdArray.sum(arr, axis, keepdim)
    local count = axis == nil and arr.size or arr.shape[ndAxisLocal(axis, #arr.shape) + 1]
    if type(res) == "number" then return res / count end
    for i = 1, res.size do res.data[i] = res.data[i] / count end
    return res
end

function NdArray.sort(arr, axis)
    if axis then
        if axis < 0 then
            axis = #arr.shape + axis
        end
        local ret = NdArray.emptyLike(arr)
        axis = axis + 1
        local stride = arr.stride[axis]
        local n = arr.shape[axis]
        ret.stride[axis] = 1
        ret.shape[axis] = 1
        for _, offset in NdArray._strideIter1(ret) do
            local values = {}
            for i=1,n do
                values[i] = arr.data[offset + (i - 1) * stride]
            end
            table.sort(values)
            for i=1,n do
                ret.data[offset + (i - 1) * stride] = values[i]
            end
        end
        ret.stride[axis] = stride
        ret.shape[axis] = n
        return ret
    else
        local ret = NdArray.flatten(arr)
        table.sort(ret.data)
        return ret
    end
end

function NdArray.argsort(arr, axis)
    if axis then
        if axis < 0 then
            axis = #arr.shape + axis
        end
        local ret = NdArray.emptyLike(arr)
        axis = axis + 1
        local stride = arr.stride[axis]
        local n = arr.shape[axis]
        ret.stride[axis] = 1
        ret.shape[axis] = 1
        local idxs = {}
        for i=1, arr.size do
            idxs[i] = i - 1
        end
        for _, offset in NdArray._strideIter1(ret) do
            table.sort(idxs, function(a, b) return arr.data[offset + a * stride] < arr.data[offset + b * stride] end)
            for i=1,n do
                ret.data[offset + (i - 1) * stride] = idxs[i]
            end
        end
        ret.stride[axis] = stride
        ret.shape[axis] = n
        return ret
    else
        local ravel = NdArray.ravel(arr)
        local idxs = {}
        for i=1, arr.size do
            idxs[i] = i
        end
        table.sort(idxs, function(a, b) return ravel.data[a] < ravel.data[b] end)
        for i=1, arr.size do
            idxs[i] = idxs[i] - 1
        end
        return NdArray.array(idxs)
    end
end

function NdArray.broadcastStride(arrShape, targetShape, arrStride)
    local stride = {}
    local padding = #targetShape - #arrShape
    assert(padding >= 0, "target shape cannot have lower rank than operand")
    for axis = 1, #targetShape do
        local sourceAxis = axis - padding
        if sourceAxis <= 0 then
            stride[axis] = 0
        else
            local size = arrShape[sourceAxis]
            assert(size == 1 or size == targetShape[axis], "shapes could not be broadcast together")
            stride[axis] = size == 1 and 0 or arrStride[sourceAxis]
        end
    end
    return stride
end

function NdArray.broadcastTo(arr, shape)
    shape = ndCopyShapeLocal(shape)
    local stride = NdArray.broadcastStride(arr.shape, shape, arr.stride)
    return ndViewLocal(arr, shape, stride)
end

function NdArray.broadcastArrays(arrs)
    if arrs and arrs.shape then arrs = {arrs} end
    assert(type(arrs) == "table" and #arrs > 0, "broadcastArrays requires operands")
    local shapes = {}
    for i, arr in ipairs(arrs) do
        shapes[i] = arr.shape
    end
    local new_shape = NdArray.broadcastShapes(shapes)
    local views = {}
    for i, arr in ipairs(arrs) do
        views[i] = NdArray.broadcastTo(arr, new_shape)
    end
    return views
end

function NdArray.broadcastShapes(shapes)
    assert(type(shapes) == "table" and #shapes > 0, "broadcastShapes requires shapes")
    local newShape = {}
    local maxNdim = 0
    for _, shape in ipairs(shapes) do maxNdim = math.max(maxNdim, #shape) end
    for axis = 1, maxNdim do
        local size = 1
        for _, shape in ipairs(shapes) do
            local sourceAxis = axis - (maxNdim - #shape)
            local candidate = sourceAxis > 0 and shape[sourceAxis] or 1
            assert(candidate == 1 or size == 1 or candidate == size,
                "shapes could not be broadcast together")
            if candidate ~= 1 then size = candidate end
        end
        newShape[axis] = size
    end
    return newShape
end

function NdArray.convertAndBroadcast(...)
    local arrs = {}
    for i, arr in ipairs({...}) do
        if type(arr) ~= "table" or not arr.shape or not arr.data then arr = NdArray.array(arr) end
        arrs[i] = arr
    end
    return NdArray.broadcastArrays(arrs)
end

function NdArray.nonzero(arr)
    assert(type(arr) == "table" and arr.shape and arr.data, "expected an NdArray")
    local ndim = #arr.shape
    local coordinates = {}
    for axis = 1, ndim do coordinates[axis] = {} end
    for idxs, values in NdArray.nditer(arr, nil, "C") do
        if values[1] ~= 0 then
            for axis = 1, ndim do coordinates[axis][#coordinates[axis] + 1] = idxs[axis] end
        end
    end
    for axis = 1, ndim do coordinates[axis] = NdArray.array(coordinates[axis]) end
    return table.unpack(coordinates, 1, ndim)
end

function NdArray.countNonzero(arr)
    local count = 0
    for _, values in NdArray.nditer(arr, nil, "C") do
        if values[1] ~= 0 then count = count + 1 end
    end
    return count
end

function NdArray.argwhere(arr)
    assert(type(arr) == "table" and arr.shape and arr.data, "expected an NdArray")
    local rows = {}
    for idx, values in NdArray.nditer(arr, nil, "C") do
        if values[1] ~= 0 then
            local row = {}
            for axis = 1, #arr.shape do row[axis] = idx[axis] end
            rows[#rows + 1] = row
        end
    end
    return NdArray.array(rows)
end

function NdArray.where(cond, ifTrue, ifFalse)
    if ifTrue == nil then
        return NdArray.nonzero(cond)
    end
    cond, ifTrue, ifFalse = table.unpack(NdArray.broadcastArrays({cond, ifTrue, ifFalse}))
    local ret = NdArray.empty(cond.shape)
    for offsets in NdArray._strideIter(
        cond.shape,
        {ret.stride, cond.stride, ifTrue.stride, ifFalse.stride},
        {ret.offset or 0, cond.offset or 0, ifTrue.offset or 0, ifFalse.offset or 0}
    ) do
        local retOfs, condOfs, trueOfs, falseOfs = offsets[1], offsets[2], offsets[3], offsets[4]
        ret.data[retOfs] = cond.data[condOfs] and ifTrue.data[trueOfs] or ifFalse.data[falseOfs]
    end
    
    return ret
end

NdArray._sliceMt = {
    __index=function(table, key)
        return table.start + key * table.step
    end,
    __len=function(table)
        return math.max(0, math.ceil((table.stop - table.start) / table.step))
    end,
}

-- todo: handle negatives
function NdArray.slice(start, stop, step)
    start = start or 0
    stop = stop or nil
    step = step or 1
    local slice = setmetatable({
            start=start,
            stop=stop,
            step=step,
        },
        NdArray._sliceMt)
    return slice
end

NdArray.sliceAll = NdArray.slice()

NdArray.newaxis = "None"

NdArray.ellipsis = "..."

function NdArray.get(arr, idxs)
    -- simple get: return view
    -- fancy get: create new array, use simple get to get values, use simple set to write values to new array
    local fancy, fancyIdxs, fancyDims = NdArray._checkFancy(#arr.shape, idxs)
    if fancy then
        local newShape
        newShape, fancyIdxs = NdArray._prepareFancyIndex(arr, idxs)
        local ret = NdArray.empty(newShape)
        if #fancyIdxs == #arr.shape then
            for setIdxs, getIdxs in NdArray.nditerPacked(fancyIdxs) do
                NdArray._setSingle(ret, setIdxs, NdArray._getSingle(arr, getIdxs))
            end
        else
            for indexed, plain in NdArray._fancyIter(arr, ret, idxs, fancyDims, fancyIdxs) do
                NdArray.copyTo(plain, indexed)
            end
        end
        return ret
    else
        return NdArray._getSimple(arr, idxs)
    end
end

function NdArray.set(arr, idxs, vals)
    -- simple set: get view using simple get, write to view using copyTo
    -- fancy set: similar to fancy get, but write to arr instead of new array
    -- iterate through ndindex over dst shape (make sure src shape broadcasts to dst shape. dst shape does not broadcast)
    -- copy src[idx] to dst[idx]

    -- fancy set: iterate through nditer on fancy indices
    -- simple set each index by corresponding view on vals
    -- local newShape, offsets, steps, srcMap, idxMap
    local fancy, fancyIdxs, fancyDims = NdArray._checkFancy(#arr.shape, idxs)
    if fancy then
        local newShape
        newShape, fancyIdxs = NdArray._prepareFancyIndex(arr, idxs)
        if #fancyIdxs == #arr.shape then
            for getIdxs, setIdxs in NdArray.nditerPacked(fancyIdxs) do
                NdArray._setSingle(arr, setIdxs, NdArray._getSingle(vals, getIdxs))
            end
        else
            for indexed, plain in NdArray._fancyIter(arr, vals, idxs, fancyDims, fancyIdxs) do
                NdArray.copyTo(indexed, plain)
            end
        end
    else
        local dst = NdArray._getSimple(arr, idxs)
        NdArray.copyTo(dst, vals)
    end
end

function NdArray._getOffset(arr, idxs)
    local offset = arr.offset
    for i, idx in ipairs(idxs) do
        if idx < 0 then
            idx = idx + arr.shape[i]
        end
        if idx >= arr.shape[i] or idx < 0 then
            error(string.format("Index %d is out of bounds for dimension %d with size %d", idx, i, arr.shape[i]))
        end
        offset = offset + arr.stride[i] * idx
    end
    return offset
end

function NdArray._getSingle(arr, idxs)
    return arr.data[NdArray._getOffset(arr, idxs) + 1]
end

function NdArray._setSingle(arr, idxs, value)
    arr.data[NdArray._getOffset(arr, idxs) + 1] = value
end

function NdArray._prepareFancyIndex(arr, idxs)
    local fancyIdxs = {}
    local fancyIdx = 1
    for d, idx in ipairs(idxs) do
        if type(idx) == 'table' and getmetatable(idx) ~= NdArray._sliceMt then
            if type(idx[1]) == 'boolean' then
                local nonzero_idxs = NdArray.nonzero(idx)
                for j=fancyIdx,fancyIdx+#nonzero_idxs do
                    fancyIdxs[j] = nonzero_idxs
                end
                fancyIdx = fancyIdx + #nonzero_idxs
            else
                fancyIdxs[fancyIdx] = idx
                fancyIdx = fancyIdx + 1
            end
        end
    end
    if #fancyIdxs > 0 then
        fancyIdxs = NdArray.broadcastArrays(fancyIdxs)
    end
    local fancyShape = fancyIdxs[1].shape
    local newShape = {}
    local newShapeIdx = 1
    local oldShapeIdx = 1
    local found = false

    -- local offsets = {}
    -- local steps = {}
    -- for d, _ in ipairs(arr.shape) do
    --     offsets[d] = 1
    --     steps[d] = 1
    -- end
    -- srcMap maps src dimensions to dst dimensions
    -- local srcMap = {}
    -- idxMap maps src dimensions to idx indices
    -- local idxMap = {}

    for idxIdx, idx in ipairs(idxs) do
        if type(idx) == 'string' then
            if idx == NdArray.sliceAll then
                newShape[newShapeIdx] = arr.shape[oldShapeIdx]
                -- idxMap[oldShapeIdx] = idxIdx
                -- offsets[oldShapeIdx] = 1
                -- steps[oldShapeIdx] = 1
                newShapeIdx = newShapeIdx + 1
                oldShapeIdx = oldShapeIdx + 1
            elseif idx == NdArray.newaxis then
                newShape[newShapeIdx] = 1
                -- newShapeIdx = newShapeIdx + 1
            elseif idx == NdArray.ellipsis then
                local idxsRemaining = #idxs - (idxIdx - 1)
                local missingDims = #arr.shape - (oldShapeIdx - 1) - idxsRemaining
                for d=1,missingDims do
                    newShape[newShapeIdx + d - 1] = arr.shape[oldShapeIdx + d - 1]
                end
                newShapeIdx = newShapeIdx + missingDims
                oldShapeIdx = oldShapeIdx + missingDims
            end
        elseif type(idx) == 'table' then
            if getmetatable(idx) == NdArray._sliceMt then
                local start, stop, step = NdArray._convertSlice(arr.shape[oldShapeIdx], idx.start, idx.stop, idx.step)
                newShape[newShapeIdx] = math.max(0, math.ceil((stop - start) / step))
                -- idxMap[oldShapeIdx] = idxIdx
                -- offsets[oldShapeIdx] = idx.start
                -- steps[oldShapeIdx] = idx.step
                newShapeIdx = newShapeIdx + 1
                oldShapeIdx = oldShapeIdx + 1
            else
                -- local map = {}
                -- for k, d in ipairs(fancyShape) do
                    -- map[k] = newShapeIdx + k - 1
                -- end
                -- srcMap[oldShapeIdx] = map
                -- idxMap[oldShapeIdx] = idxIdx
                if not found then
                    for k, d in ipairs(fancyShape) do
                        newShape[newShapeIdx] = d
                        newShapeIdx = newShapeIdx + 1
                    end
                end
                oldShapeIdx = oldShapeIdx + 1
                found = true
            end
        else
            -- offsets[oldShapeIdx] = idx
            oldShapeIdx = oldShapeIdx + 1
        end
    end
    if oldShapeIdx <= #arr.shape then
        for d=oldShapeIdx,#arr.shape do
            newShape[newShapeIdx] = arr.shape[d]
            newShapeIdx = newShapeIdx + 1
        end
    end
    return newShape, fancyIdxs --, offsets, steps, srcMap, idxMap
end

function NdArray._getSimple(arr, idxs)
    local ret = Util.shallowCopyWithMeta(arr)
    local new_shape = {}
    local new_stride = {}
    local offset = arr.offset
    local newIdx = 1
    local oldIdx = 1
    local size = 1
    for j, idx in ipairs(idxs) do
        -- integer indices remove a dimension
        if type(idx) == 'number' then
            if idx ~= math.floor(idx) then
                error(string.format("All indices must be integers, slices, or newaxis. Got non-integer index with value %f in position %d", idx, j))
            end
            if idx < -arr.shape[oldIdx] or idx >= arr.shape[oldIdx] then
                error(string.format("Index %d is out of bounds for dimension %d with size %d", idx, oldIdx, arr.shape[oldIdx]))
            end
            if idx < 0 then
                idx = idx + arr.shape[oldIdx]
            end
            offset = offset + idx * arr.stride[oldIdx]
            oldIdx = oldIdx + 1
        -- newaxis adds a dimension
        elseif idx == NdArray.newaxis then
            new_shape[newIdx] = 1
            new_stride[newIdx] = 1
            newIdx = newIdx + 1
        elseif idx == NdArray.ellipsis then
            for dim=oldIdx,#arr.shape-#idxs do
                local n = arr.shape[dim]
                size = size * n
                new_shape[newIdx] = n
                new_stride[newIdx] = arr.stride[dim]
                newIdx = newIdx + 1
            end
            oldIdx = oldIdx + #arr.shape - #idxs
        else
            local start
            local stop
            local step
            if idx == NdArray.sliceAll then
                start = 0
                stop = arr.shape[oldIdx]
                step = 1
            else
                start, stop, step = NdArray._convertSlice(arr.shape[oldIdx], idx.start, idx.stop, idx.step)
            end
            if start ~= nil then
                if stop == 0 then
                    stop = arr.shape[oldIdx]
                end
                local n = math.floor((stop - start) / step) + 1
                new_shape[newIdx] = n
                size = size * n
                new_stride[newIdx] = arr.stride[oldIdx] * step
                offset = offset + start * arr.stride[oldIdx]
            end
            newIdx = newIdx + 1
            oldIdx = oldIdx + 1
        end
    end
    for dim=oldIdx,#arr.shape do
        new_shape[newIdx] = arr.shape[dim]
        new_stride[newIdx] = arr.stride[dim]
        newIdx = newIdx + 1
    end

    ret.shape = new_shape
    ret.stride = new_stride
    ret.offset = offset
    ret.size = size
    return ret
end

function NdArray._convertSlice(dim, start, stop, step)
    if start < 0 then
        start = start + dim
    end
    if stop == nil then
        stop = step < 0 and -1 or dim
    elseif stop < 0 then
        stop = stop + dim
    end
    local bound = step < 0 and 1 or 0
    start = math.min(math.max(start, bound), dim + bound)
    stop = math.min(math.max(stop, bound), dim + bound)
    return start, stop, step
end

function NdArray._checkFancy(ndim, idxs)
    local fancy = false
    local fancyIdxs = {}
    local fancyDims = {}
    local ellipsisOffset = 0
    for d, idx in ipairs(idxs) do
        if idx == NdArray.ellipsis then
            ellipsisOffset = ndim - (#idxs - 1)
        elseif type(idx) == 'table' and getmetatable(idx) ~= NdArray._sliceMt then
            fancy = true
            fancyIdxs[#fancyIdxs + 1] = idx
            fancyDims[#fancyDims + 1] = d + ellipsisOffset
        end
    end
    return fancy, fancyIdxs, fancyDims
end

function NdArray._fancyIter(indexed, plain, idxs, fancyDims, fancyIdxs)
    local setIdxs = Utils.shallowCopy(idxs)
    for i, d in fancyDims do
        setIdxs[d] = 1
    end
    local baseIndexedArray = NdArray.get(indexed, setIdxs)
    local baseIndexedOffset = baseIndexedArray.offset

    local plainIdxs = {}
    local firstFancyDim = fancyDims[1]
    for i=1, firstFancyDim do
        plainIdxs[i] = NdArray.sliceAll
    end
    fancyIdxs = NdArray.broadcastArrays(fancyIdxs)
    local fancyNdim = #fancyIdxs[1].shape
    for i=firstFancyDim + 1,firstFancyDim + fancyNdim - 1 do
        plainIdxs[i] = 1
    end
    local basePlainArray = NdArray.get(plain, plainIdxs)
    local basePlainOffset = basePlainArray.offset

    local nditer = NdArray._nditer(fancyIdxs)

    return function()
        local fancyGetIdxs, fancySetIdxs = nditer()
        local indexedOffset = baseIndexedOffset
        local plainOffset = basePlainOffset
        for i, d in fancyDims do
            indexedOffset = indexedOffset + fancySetIdxs[i] * indexed.stride[d]
            plainOffset = plainOffset + fancyGetIdxs[i] * plain.stride[d]
        end
        baseIndexedArray.offset = indexedOffset
        basePlainArray.offset = plainOffset

        return baseIndexedArray, basePlainArray
    end
end

-- Minimal iterator layers for the generic array representation.  These are
-- intentionally kept independent of the unfinished construction/indexing API
-- above: an operand only needs shape, stride, data, and an optional offset.
local function ndCopyShape(shape)
    assert(type(shape) == "table", "shape must be a table")
    local result = {}
    for axis, size in ipairs(shape) do
        assert(type(size) == "number" and size >= 0 and size == math.floor(size),
            "shape dimensions must be non-negative integers")
        result[axis] = size
    end
    return result
end

local function ndSize(shape)
    local size = 1
    for _, dimension in ipairs(shape) do size = size * dimension end
    return size
end

local function ndAxis(axis, ndim)
    assert(type(axis) == "number" and axis == math.floor(axis), "axis must be an integer")
    if axis < 0 then axis = ndim + axis end
    assert(axis >= 0 and axis < ndim, "axis is out of bounds")
    return axis
end

local function ndOrder(order, ndim)
    if order == nil or order == "C" or order == "K" then
        local result = {}
        for axis = 1, ndim do result[axis] = axis end
        return result
    end
    if order == "F" then
        local result = {}
        for axis = 1, ndim do result[axis] = ndim - axis + 1 end
        return result
    end
    assert(type(order) == "table" and #order == ndim,
        "order must be nil, 'C', 'F', or an axis permutation")
    local zeroBased = false
    for _, axis in ipairs(order) do zeroBased = zeroBased or axis == 0 end
    local result, seen = {}, {}
    for position, axis in ipairs(order) do
        axis = zeroBased and axis + 1 or axis
        assert(type(axis) == "number" and axis == math.floor(axis)
            and axis >= 1 and axis <= ndim, "order contains an invalid axis")
        assert(not seen[axis], "order must be a permutation")
        seen[axis] = true
        result[position] = axis
    end
    return result
end

local function ndTraversal(shape, strides, baseOffsets, order)
    local ndim = #shape
    local indices, offsets = {}, {}
    for axis = 1, ndim do indices[axis] = 0 end
    for operand, offset in ipairs(baseOffsets) do offsets[operand] = offset or 0 end
    local axisOrder = ndOrder(order, ndim)
    local first, done = true, ndSize(shape) == 0
    return function()
        if done then return nil end
        if first then
            first = false
            return indices, offsets
        end
        if ndim == 0 then done = true; return nil end
        for position = ndim, 1, -1 do
            local axis = axisOrder[position]
            if indices[axis] + 1 < shape[axis] then
                indices[axis] = indices[axis] + 1
                for operand, stride in ipairs(strides) do
                    offsets[operand] = offsets[operand] + stride[axis]
                end
                return indices, offsets
            end
            indices[axis] = 0
            for operand, stride in ipairs(strides) do
                offsets[operand] = offsets[operand] - stride[axis] * (shape[axis] - 1)
            end
            if position == 1 then done = true; return nil end
        end
    end
end

local function ndBroadcastShape(shapes)
    local rank = 0
    for _, shape in ipairs(shapes) do rank = math.max(rank, #shape) end
    local result = {}
    for axis = 1, rank do result[axis] = 1 end
    for _, shape in ipairs(shapes) do
        local padding = rank - #shape
        for sourceAxis, size in ipairs(shape) do
            local axis = padding + sourceAxis
            assert(size == 1 or result[axis] == 1 or result[axis] == size,
                "operands could not be broadcast together")
            if size ~= 1 then result[axis] = size end
        end
    end
    return result
end

local function ndAlignedStride(arr, targetShape)
    local padding = #targetShape - #arr.shape
    assert(padding >= 0, "iteration shape cannot have lower rank than an operand")
    local result = {}
    for axis = 1, #targetShape do
        local sourceAxis = axis - padding
        if sourceAxis <= 0 then
            result[axis] = 0
        else
            local size = arr.shape[sourceAxis]
            assert(size == 1 or size == targetShape[axis],
                "operand cannot be broadcast to the iteration shape")
            result[axis] = size == 1 and 0 or arr.stride[sourceAxis]
        end
    end
    return result
end

local function ndCheckArray(arr)
    assert(type(arr) == "table" and type(arr.shape) == "table"
        and type(arr.stride) == "table" and type(arr.data) == "table",
        "expected an NdArray-like operand")
end

local function ndOpAxes(arrs, opAxes, iterShape)
    local ndim = iterShape and #iterShape or 0
    if opAxes then
        for operand = 1, #arrs do
            local mapping = opAxes[operand]
            if mapping then ndim = math.max(ndim, #mapping) end
        end
    else
        for _, arr in ipairs(arrs) do ndim = math.max(ndim, #arr.shape) end
    end
    if iterShape then assert(#iterShape == ndim, "itershape rank does not match op_axes") end

    local shape
    if iterShape then
        shape = ndCopyShape(iterShape)
    elseif opAxes then
        shape = {}
    else
        local shapes = {}
        for operand, arr in ipairs(arrs) do shapes[operand] = arr.shape end
        shape = ndBroadcastShape(shapes)
    end
    for axis = 1, ndim do shape[axis] = shape[axis] or 1 end
    local strides = {}
    for operand, arr in ipairs(arrs) do
        local mapping = opAxes and opAxes[operand]
        if mapping then
            assert(#mapping == ndim, "each op_axes mapping must match iterator rank")
            local used = {}
            strides[operand] = {}
            for iteratorAxis = 1, ndim do
                local sourceAxis = mapping[iteratorAxis]
                if sourceAxis ~= -1 then
                    sourceAxis = ndAxis(sourceAxis, #arr.shape)
                    assert(not used[sourceAxis], "an operand axis may appear only once in op_axes")
                    used[sourceAxis] = true
                    local size = arr.shape[sourceAxis + 1]
                    if not iterShape then
                        if size ~= 1 then
                            assert(shape[iteratorAxis] == 1 or shape[iteratorAxis] == size,
                                "op_axes dimensions could not be broadcast")
                            shape[iteratorAxis] = size
                        end
                    else
                        assert(size == 1 or size == shape[iteratorAxis],
                            "operand cannot be broadcast to itershape")
                    end
                    strides[operand][iteratorAxis] = size == 1 and 0 or arr.stride[sourceAxis + 1]
                else
                    strides[operand][iteratorAxis] = 0
                end
            end
        else
            strides[operand] = ndAlignedStride(arr, shape)
        end
    end
    return shape, strides
end

-- Scalar iterator.  `opAxes[operand][iteratorAxis]` is a complete mapping
-- (zero-based operand axes, or -1 for newaxis), matching numpy.nditer.
function NdArray.nditer(arrs, opFlags, order, opAxes, iterShape)
    if arrs and arrs.shape then arrs = {arrs} end
    assert(type(arrs) == "table" and #arrs > 0, "nditer requires operands")
    for _, arr in ipairs(arrs) do ndCheckArray(arr) end
    local shape, strides = ndOpAxes(arrs, opAxes, iterShape)
    local offsets = {}
    for operand, arr in ipairs(arrs) do offsets[operand] = arr.offset or 0 end
    local iterator = ndTraversal(shape, strides, offsets, order)
    local values = {}
    return function()
        local indices, currentOffsets = iterator()
        if not indices then return nil end
        for operand, arr in ipairs(arrs) do
            values[operand] = arr.data[currentOffsets[operand] + 1]
        end
        return indices, values
    end
end

local function ndNormalizeCore(arrs, coreAxes, iterShape)
    local loopShapes, loopAxes = {}, {}
    local coreShapes, coreStrides = {}, {}
    for operand, arr in ipairs(arrs) do
        ndCheckArray(arr)
        local axes = coreAxes[operand] or {}
        local selected, normalized = {}, {}
        for position, axis in ipairs(axes) do
            axis = ndAxis(axis, #arr.shape)
            assert(not selected[axis], "an operand core axis may appear only once")
            selected[axis] = true
            normalized[position] = axis
            coreShapes[operand] = coreShapes[operand] or {}
            coreStrides[operand] = coreStrides[operand] or {}
            coreShapes[operand][position] = arr.shape[axis + 1]
            coreStrides[operand][position] = arr.stride[axis + 1]
        end
        loopAxes[operand] = {}
        loopShapes[operand] = {}
        for axis = 0, #arr.shape - 1 do
            if not selected[axis] then
                loopAxes[operand][#loopAxes[operand] + 1] = axis
                loopShapes[operand][#loopShapes[operand] + 1] = arr.shape[axis + 1]
            end
        end
        coreShapes[operand] = coreShapes[operand] or {}
        coreStrides[operand] = coreStrides[operand] or {}
    end
    local loopShape = iterShape and ndCopyShape(iterShape) or ndBroadcastShape(loopShapes)
    local loopStrides, offsets = {}, {}
    for operand, arr in ipairs(arrs) do
        local axes, shape = loopAxes[operand], loopShapes[operand]
        local padding = #loopShape - #shape
        assert(padding >= 0, "loop shape cannot have lower rank than an operand")
        loopStrides[operand] = {}
        for globalAxis = 1, #loopShape do
            local localAxis = globalAxis - padding
            if localAxis <= 0 then
                loopStrides[operand][globalAxis] = 0
            else
                local size = shape[localAxis]
                assert(size == 1 or size == loopShape[globalAxis],
                    "loop dimensions could not be broadcast")
                loopStrides[operand][globalAxis] = size == 1 and 0
                    or arr.stride[axes[localAxis] + 1]
            end
        end
        offsets[operand] = arr.offset or 0
    end
    return loopShape, loopStrides, offsets, coreShapes, coreStrides
end

local function ndView(arr, shape, stride, offset)
    return {shape = shape, stride = stride, data = arr.data, offset = offset, size = ndSize(shape)}
end

-- Direct-axis core iterator.  Unlike nditer, this preserves the selected
-- axes as reusable N-D views and broadcasts only the remaining loop axes.
function NdArray.coreIter(arrs, coreAxes, order, iterShape)
    if arrs and arrs.shape then arrs = {arrs} end
    assert(type(arrs) == "table" and #arrs > 0, "coreIter requires operands")
    coreAxes = coreAxes or {}
    local loopShape, loopStrides, offsets, coreShapes, coreStrides =
        ndNormalizeCore(arrs, coreAxes, iterShape)
    local iterator = ndTraversal(loopShape, loopStrides, offsets, order)
    local views = {}
    for operand, arr in ipairs(arrs) do
        views[operand] = ndView(arr, coreShapes[operand], coreStrides[operand], offsets[operand])
    end
    return function()
        local indices, currentOffsets = iterator()
        if not indices then return nil end
        for operand, arr in ipairs(arrs) do
            views[operand].offset = currentOffsets[operand]
        end
        return indices, views
    end
end

local function ndParseSignature(signature)
    assert(type(signature) == "string", "gufunc signature must be a string")
    local inputText, outputText = signature:match("^%s*(.-)%s*%-%>%s*(.-)%s*$")
    assert(inputText and outputText, "gufunc signature must contain '->'")
    local function parsePart(text)
        local result, consumed = {}, 0
        for group in text:gmatch("%b()") do
            local start = text:find(group, consumed + 1, true)
            assert(start == consumed + 1 or text:sub(consumed + 1, start - 1):match("^%s*,%s*$"),
                "invalid gufunc signature")
            consumed = start + #group
            local dimensions = {}
            local body = group:sub(2, -2)
            for label in body:gmatch("[^,%s]+") do
                assert(label:match("^[%a_][%w_]*$"), "invalid core dimension name")
                dimensions[#dimensions + 1] = label
            end
            result[#result + 1] = dimensions
        end
        assert(text:sub(consumed + 1):match("^%s*$"), "invalid gufunc signature")
        return result
    end
    local inputs, outputs = parsePart(inputText), parsePart(outputText)
    assert(#outputs == 1, "this minimal gufunc wrapper supports one output")
    return inputs, outputs[1]
end

local function ndDefaultCoreAxes(arr, rank)
    assert(rank <= #arr.shape, "operand rank is smaller than its core rank")
    local result = {}
    for position = 0, rank - 1 do result[position + 1] = #arr.shape - rank + position end
    return result
end

local function ndContiguousArray(shape)
    local result, stride = {}, {}
    local step = 1
    for axis = #shape, 1, -1 do
        stride[axis] = step
        step = step * shape[axis]
    end
    result.shape, result.stride, result.data, result.offset, result.size = ndCopyShape(shape), stride, {}, 0, step
    return result
end

local function ndOutputLayout(loopShape, coreShape, coreAxes)
    local rank = #loopShape + #coreShape
    local axes = coreAxes and {} or nil
    if not axes then
        axes = {}
        for position = 0, #coreShape - 1 do axes[position + 1] = #loopShape + position end
    else
        for position, axis in ipairs(coreAxes) do axes[position] = ndAxis(axis, rank) end
        assert(#axes == #coreShape, "output axes must match output core rank")
        local used = {}
        for _, axis in ipairs(axes) do assert(not used[axis], "output axes must be unique"); used[axis] = true end
    end
    local shape, coreAt = {}, {}
    for position, axis in ipairs(axes) do coreAt[axis] = position end
    local loopPosition = 1
    for axis = 0, rank - 1 do
        local corePosition = coreAt[axis]
        if corePosition then shape[axis + 1] = coreShape[corePosition]
        else shape[axis + 1] = loopShape[loopPosition]; loopPosition = loopPosition + 1 end
    end
    return shape, axes
end

local function ndCopyCore(result, output, outputBase, outputAxes, coreShape)
    local rank = #coreShape
    if rank == 0 then
        assert(type(result) == "number", "scalar gufunc output must be a number")
        output.data[outputBase + 1] = result
        return
    end
    ndCheckArray(result)
    assert(#result.shape == rank, "gufunc result rank does not match its signature")
    for axis = 1, rank do assert(result.shape[axis] == coreShape[axis], "gufunc result shape does not match its signature") end
    local iterator = ndTraversal(coreShape, {result.stride}, {result.offset or 0}, "C")
    while true do
        local indices, offsets = iterator()
        if not indices then break end
        local outputOffset = outputBase
        for axis = 1, rank do outputOffset = outputOffset + indices[axis] * output.stride[outputAxes[axis] + 1] end
        output.data[outputOffset + 1] = result.data[offsets[1] + 1]
    end
end

-- Signature layer.  `axes` is optional and, when supplied, contains one
-- zero-based core-axis list per input followed by the output core-axis list.
function NdArray.gufunc(func, signature, axes)
    assert(type(func) == "function", "gufunc requires a function")
    local inputDims, outputDims = ndParseSignature(signature)
    return function(...)
        local args = {...}
        assert(#args == #inputDims, "gufunc argument count does not match signature")
        local inputAxes, outputAxes = {}, nil
        for operand, dimensions in ipairs(inputDims) do
            inputAxes[operand] = axes and axes[operand] or ndDefaultCoreAxes(args[operand], #dimensions)
            assert(#inputAxes[operand] == #dimensions, "input axes do not match signature")
        end
        if axes and axes[#inputDims + 1] then outputAxes = axes[#inputDims + 1] end

        local labelSizes = {}
        for operand, dimensions in ipairs(inputDims) do
            for position, label in ipairs(dimensions) do
                local axis = ndAxis(inputAxes[operand][position], #args[operand].shape)
                local size = args[operand].shape[axis + 1]
                assert(not labelSizes[label] or labelSizes[label] == size,
                    "gufunc core dimensions must match exactly")
                labelSizes[label] = size
            end
        end
        local outputCoreShape = {}
        for position, label in ipairs(outputDims) do
            assert(labelSizes[label], "output core dimension is not inferable from inputs")
            outputCoreShape[position] = labelSizes[label]
        end

        local loopShape, loopStrides, offsets, coreShapes, coreStrides =
            ndNormalizeCore(args, inputAxes)
        for operand, dimensions in ipairs(inputDims) do
            assert(#coreShapes[operand] == #dimensions)
        end
        local outputShape, normalizedOutputAxes = ndOutputLayout(loopShape, outputCoreShape, outputAxes)
        local output = ndContiguousArray(outputShape)
        local iterator = ndTraversal(loopShape, loopStrides, offsets, "C")
        local views = {}
        for operand, arr in ipairs(args) do
            views[operand] = ndView(arr, coreShapes[operand], coreStrides[operand], offsets[operand])
        end
        while true do
            local indices, currentOffsets = iterator()
            if not indices then break end
            for operand = 1, #args do views[operand].offset = currentOffsets[operand] end
            local result = func(table.unpack(views, 1, #views))
            local outputBase = output.offset or 0
            local outputCoreSet = {}
            for _, axis in ipairs(normalizedOutputAxes) do outputCoreSet[axis] = true end
            local loopPosition = 1
            for axis = 0, #outputShape - 1 do
                if not outputCoreSet[axis] then
                    outputBase = outputBase + indices[loopPosition] * output.stride[axis + 1]
                    loopPosition = loopPosition + 1
                end
            end
            ndCopyCore(result, output, outputBase, normalizedOutputAxes, outputCoreShape)
        end
        return output
    end
end

function NdArray.eye(size)
    local result = NdArray.zeros({size, size})
    for i = 0, size - 1 do result.data[i * size + i + 1] = 1 end
    return result
end

NdArray.identity = NdArray.eye

-- These remain named functions intentionally.  Lua comparison metamethods
-- (__eq/__lt/__le, where supported) must produce one boolean for the
-- comparison expression; they cannot return an elementwise NdArray.
local function ndComparison(func, a, b) return NdArray._elementwise(func, a, b) end
NdArray.equal = function(a, b) return ndComparison(function(x, y) return x == y end, a, b) end
NdArray.notEqual = function(a, b) return ndComparison(function(x, y) return x ~= y end, a, b) end
NdArray.less = function(a, b) return ndComparison(function(x, y) return x < y end, a, b) end
NdArray.lessEqual = function(a, b) return ndComparison(function(x, y) return x <= y end, a, b) end
NdArray.greater = function(a, b) return ndComparison(function(x, y) return x > y end, a, b) end
NdArray.greaterEqual = function(a, b) return ndComparison(function(x, y) return x >= y end, a, b) end

local function ndWriteTraversal(source, output, axisOffset, axisLength, axis)
    local iterator = ndTraversal(source.shape, {source.stride}, {source.offset or 0}, "C")
    while true do
        local indices, offsets = iterator()
        if not indices then break end
        local outputOffset = output.offset or 0
        for dimension = 0, #source.shape - 1 do
            local targetDimension = dimension == axis and axisOffset + indices[dimension + 1] or indices[dimension + 1]
            outputOffset = outputOffset + targetDimension * output.stride[dimension + 1]
        end
        output.data[outputOffset + 1] = source.data[offsets[1] + 1]
    end
end

function NdArray.concatenate(arrays, axis)
    assert(type(arrays) == "table" and #arrays > 0, "concatenate requires arrays")
    ndCheckArray(arrays[1])
    axis = ndAxis(axis or 0, #arrays[1].shape)
    local shape = ndCopyShape(arrays[1].shape)
    shape[axis + 1] = 0
    for _, arr in ipairs(arrays) do
        ndCheckArray(arr); assert(#arr.shape == #shape, "concatenate ranks must match")
        for dimension = 1, #shape do
            if dimension ~= axis + 1 then assert(arr.shape[dimension] == arrays[1].shape[dimension], "concatenate shapes must match") end
        end
        shape[axis + 1] = shape[axis + 1] + arr.shape[axis + 1]
    end
    local result, offset = ndContiguousArray(shape), 0
    for _, arr in ipairs(arrays) do
        ndWriteTraversal(arr, result, offset, arr.shape[axis + 1], axis)
        offset = offset + arr.shape[axis + 1]
    end
    return result
end

function NdArray.stack(arrays, axis)
    assert(type(arrays) == "table" and #arrays > 0, "stack requires arrays")
    local expanded = {}
    for i, arr in ipairs(arrays) do expanded[i] = NdArray.expandDims(arr, axis or 0) end
    return NdArray.concatenate(expanded, axis or 0)
end

function NdArray.expandDims(arr, axis)
    ndCheckArray(arr)
    axis = axis or 0
    if axis < 0 then axis = #arr.shape + axis + 1 end
    assert(axis >= 0 and axis <= #arr.shape, "axis is out of bounds")
    local shape, stride = {}, {}
    for dimension = 0, #arr.shape do
        if dimension == axis then shape[dimension + 1], stride[dimension + 1] = 1, 0
        else
            local source = dimension > axis and dimension or dimension + 1
            shape[dimension + 1], stride[dimension + 1] = arr.shape[source], arr.stride[source]
        end
    end
    return ndView(arr, shape, stride, arr.offset or 0)
end

function NdArray.squeeze(arr, axis)
    ndCheckArray(arr)
    local remove = {}
    if axis == nil then
        for dimension, size in ipairs(arr.shape) do if size == 1 then remove[dimension] = true end end
    else
        axis = ndAxis(axis, #arr.shape); assert(arr.shape[axis + 1] == 1, "squeeze axis must have size one"); remove[axis + 1] = true
    end
    local shape, stride = {}, {}
    for dimension = 1, #arr.shape do
        if not remove[dimension] then shape[#shape + 1], stride[#stride + 1] = arr.shape[dimension], arr.stride[dimension] end
    end
    return ndView(arr, shape, stride, arr.offset or 0)
end

function NdArray.diag(arr, offset)
    ndCheckArray(arr); offset = offset or 0
    if #arr.shape == 1 then
        local size = arr.shape[1] + math.abs(offset)
        local result = NdArray.zeros({size, size})
        for i = 0, arr.shape[1] - 1 do
            local row, col = offset >= 0 and i or i - offset, offset >= 0 and i + offset or i
            result.data[row * size + col + 1] = arr.data[(arr.offset or 0) + i * arr.stride[1] + 1]
        end
        return result
    end
    assert(#arr.shape == 2, "diag requires a vector or matrix")
    local startRow, startCol = offset < 0 and -offset or 0, offset > 0 and offset or 0
    local length = math.max(0, math.min(arr.shape[1] - startRow, arr.shape[2] - startCol))
    local result = NdArray.empty({length})
    for i = 0, length - 1 do
        result.data[i + 1] = arr.data[(arr.offset or 0) + (startRow + i) * arr.stride[1]
            + (startCol + i) * arr.stride[2] + 1]
    end
    return result
end

local function ndArgReduce(comparison, arr, axis, keepdim)
    ndCheckArray(arr)
    if axis == nil then
        local best, index, position = nil, nil, 0
        for _, values in NdArray.nditer(arr, nil, "C") do
            if best == nil or comparison(values[1], best) then best, index = values[1], position end
            position = position + 1
        end
        assert(index ~= nil, "cannot reduce an empty array")
        return index
    end
    axis = ndAxis(axis, #arr.shape)
    local outputShape = {}
    for dimension = 0, #arr.shape - 1 do
        if keepdim or dimension ~= axis then outputShape[#outputShape + 1] = dimension == axis and 1 or arr.shape[dimension + 1] end
    end
    local result = ndContiguousArray(outputShape)
    local iterator = NdArray.coreIter({arr}, {{axis}})
    local indices, views = iterator()
    while indices do
        local view, best, bestIndex = views[1], nil, nil
        for i = 0, view.shape[1] - 1 do
            local value = view.data[view.offset + i * view.stride[1] + 1]
            if best == nil or comparison(value, best) then best, bestIndex = value, i end
        end
        assert(bestIndex ~= nil, "cannot reduce an empty axis")
        local outputOffset, loopPosition = 0, 1
        for dimension = 0, #arr.shape - 1 do if dimension ~= axis then
            local target = keepdim and dimension or dimension - (dimension > axis and 1 or 0)
            outputOffset = outputOffset + indices[loopPosition] * result.stride[target + 1]; loopPosition = loopPosition + 1
        end end
        result.data[outputOffset + 1] = bestIndex
        indices, views = iterator()
    end
    return result
end

NdArray.argmax = function(arr, axis, keepdim) return ndArgReduce(function(x, y) return x > y end, arr, axis, keepdim) end
NdArray.argmin = function(arr, axis, keepdim) return ndArgReduce(function(x, y) return x < y end, arr, axis, keepdim) end

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

function Util.shallowCopyWithMeta(tab)
    local new = Util.ShallowCopy(tab)
    new.metatable = tab.metatable
    return new
end

function Util.shallowCopy(tab)
    return {tab.unpack()}
end

function Util.defaultdict(factory, nested)
    local meta = {}
    meta.__index = function(t, k)
        local v = factory()
        if nested then
            setmetatable(v, meta)
        end
        rawset(t, k, v)
        return v
    end
    return setmetatable(factory(), meta)
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
  Augmented Pure Proportional Navigation
  guidance law. FtD APN is actually pure
  PN, not APN.

  Pure proportional guidance commands
  acceleration perpendicular to missile
  velocity instead of line of sight
  between missile and target like true
  proportional guidance does.
--]]
function Targeting.APPN(gain, relPos, missileVel, targetVel, targetAccel)
  local relVel = targetVel - missileVel
  local tgo = relPos.magnitude / missileVel.magnitude

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
  guidance law. FtD APN uses pure PN,
  not true PN.

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
    direction = Quaternion.Inverse(rot) * direction
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

function StringUtil.Join(tab, delimiter, start, stop)
  start = start or 1
  stop = stop or #tab
  delimiter = delimiter or ','
  if stop > 8 then
    local l = StringUtil.Join(tab, delimiter, start, math.floor(stop * 0.5))
    local r = StringUtil.Join(tab, delimiter, math.floor(stop * 0.5) + 1, stop)
    return l..r
  end
  local str = ''
  for i, s in ipairs(tab) do
    str = str..(s..',')
  end
  return str
end
