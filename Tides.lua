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

local function vectorLength(vector)
    assert(vector and vector.shape and #vector.shape == 1, "expected a vector")
    return vector.shape[1]
end

local function vectorValue(vector, index)
    return vector.data[(vector.offset or 0) + index * vector.stride[1] + 1]
end

local function rhsInfo(rhs, rows)
    assert(rhs and rhs.shape and rhs.data, "expected an NdArray right hand side")
    if #rhs.shape == 1 then
        assert(rhs.shape[1] == rows, "right hand side dimensions do not match")
        return 1, true
    end
    assert(#rhs.shape == 2 and rhs.shape[1] == rows, "right hand side dimensions do not match")
    return rhs.shape[2], false
end

local function rhsValue(rhs, row, column, vector)
    if vector then return rhs.data[(rhs.offset or 0) + row * rhs.stride[1] + 1] end
    return at(rhs, row, column)
end

local function solveUpper(u, rhs, columns, vector)
    local rows = matrixShape(u)
    local result = vector and {} or NdArray.zeros({rows, columns})
    for column = 0, columns - 1 do
        local solution = {}
        for row = rows - 1, 0, -1 do
            local value = vector and rhs[row + 1] or rhs[row + 1][column + 1]
            for nextColumn = row + 1, rows - 1 do value = value - at(u, row, nextColumn) * solution[nextColumn + 1] end
            solution[row + 1] = value / at(u, row, row)
            if not vector then put(result, row, column, solution[row + 1]) end
        end
        if vector then result = solution end
    end
    return vector and NdArray.array(result) or result
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
    local columns, vector = rhsInfo(b, n)
    local permutation = permutationRows(p, n)
    local y = {}
    for column = 0, columns - 1 do
        y[column + 1] = {}
        for row = 0, n - 1 do
            local value = rhsValue(b, permutation[row + 1], column, vector)
            for previous = 0, row - 1 do value = value - at(l, row, previous) * y[column + 1][previous + 1] end
            y[column + 1][row + 1] = value
        end
    end
    if vector then return solveUpper(u, y[1], 1, true) end
    local result = NdArray.zeros({n, columns})
    for column = 0, columns - 1 do
        local solution = solveUpper(u, y[column + 1], 1, true)
        for row = 0, n - 1 do put(result, row, column, solution.data[row + 1]) end
    end
    return result
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
    local columns, vector = rhsInfo(b, rows)
    local transformed = {}
    for column = 0, columns - 1 do
        transformed[column + 1] = {}
        for row = 0, rows - 1 do
            local value = 0
            for source = 0, rows - 1 do value = value + at(q, source, row) * rhsValue(b, source, column, vector) end
            transformed[column + 1][row + 1] = value
        end
    end
    if vector then return solveUpper(r, transformed[1], 1, true) end
    local result = NdArray.zeros({rows, columns})
    for column = 0, columns - 1 do
        local solution = solveUpper(r, transformed[column + 1], 1, true)
        for row = 0, rows - 1 do put(result, row, column, solution.data[row + 1]) end
    end
    return result
end

-- Full-column-rank least-squares solve using the Householder QR factorization.
-- The vector/matrix shape of b is preserved in the result.
function Linalg.solveLeastSquares(matrix, b, tolerance)
    local rows, cols = matrixShape(matrix)
    assert(rows >= cols, "least-squares QR requires at least as many rows as columns")
    local rhsColumns, vector = rhsInfo(b, rows)
    local q, r = Linalg.qr(matrix)
    local transformed = {}
    for column = 0, rhsColumns - 1 do
        transformed[column + 1] = {}
        for row = 0, cols - 1 do
            local value = 0
            for source = 0, rows - 1 do value = value + at(q, source, row) * rhsValue(b, source, column, vector) end
            transformed[column + 1][row + 1] = value
        end
    end
    tolerance = tolerance or 1e-14
    for row = 0, cols - 1 do assert(math.abs(at(r, row, row)) > tolerance, "least-squares matrix is rank deficient") end
    if vector then return solveUpper(NdArray.get(r, {NdArray.slice(0, cols), NdArray.slice(0, cols)}), transformed[1], 1, true) end
    local leading = NdArray.get(r, {NdArray.slice(0, cols), NdArray.slice(0, cols)})
    local result = NdArray.zeros({cols, rhsColumns})
    for column = 0, rhsColumns - 1 do
        local solution = solveUpper(leading, transformed[column + 1], 1, true)
        for row = 0, cols - 1 do put(result, row, column, solution.data[row + 1]) end
    end
    return result
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

function Linalg.solveCholesky(l, b)
    local n, cols = matrixShape(l)
    assert(cols == n, "Cholesky factor must be square")
    local columns, vector = rhsInfo(b, n)
    local forward = {}
    for column = 0, columns - 1 do
        forward[column + 1] = {}
        for row = 0, n - 1 do
            local value = rhsValue(b, row, column, vector)
            for previous = 0, row - 1 do value = value - at(l, row, previous) * forward[column + 1][previous + 1] end
            forward[column + 1][row + 1] = value / at(l, row, row)
        end
    end
    if vector then
        local x = {}
        for row = n - 1, 0, -1 do
            local value = forward[1][row + 1]
            for nextRow = row + 1, n - 1 do value = value - at(l, nextRow, row) * x[nextRow + 1] end
            x[row + 1] = value / at(l, row, row)
        end
        return NdArray.array(x)
    end
    local result = NdArray.zeros({n, columns})
    for column = 0, columns - 1 do
        local x = {}
        for row = n - 1, 0, -1 do
            local value = forward[column + 1][row + 1]
            for nextRow = row + 1, n - 1 do value = value - at(l, nextRow, row) * x[nextRow + 1] end
            x[row + 1] = value / at(l, row, row)
        end
        for row = 0, n - 1 do put(result, row, column, x[row + 1]) end
    end
    return result
end

function Linalg.normalize(vector, tolerance)
    local length = Linalg.norm(vector)
    assert(length > (tolerance or 1e-14), "cannot normalize a zero vector")
    local result = NdArray.copy(vector)
    for i = 1, result.size do result.data[i] = result.data[i] / length end
    return result, length
end

function Linalg.cross(a, b)
    assert(vectorLength(a) == 3 and vectorLength(b) == 3, "cross requires three-dimensional vectors")
    return NdArray.array({
        vectorValue(a, 1) * vectorValue(b, 2) - vectorValue(a, 2) * vectorValue(b, 1),
        vectorValue(a, 2) * vectorValue(b, 0) - vectorValue(a, 0) * vectorValue(b, 2),
        vectorValue(a, 0) * vectorValue(b, 1) - vectorValue(a, 1) * vectorValue(b, 0),
    })
end

function Linalg.outer(a, b)
    local rows, cols = vectorLength(a), vectorLength(b)
    local result = NdArray.zeros({rows, cols})
    for row = 0, rows - 1 do
        local left = a.data[(a.offset or 0) + row * a.stride[1] + 1]
        for col = 0, cols - 1 do put(result, row, col, left * b.data[(b.offset or 0) + col * b.stride[1] + 1]) end
    end
    return result
end

function Linalg.skew(vector)
    assert(vectorLength(vector) == 3, "skew requires a three-dimensional vector")
    local x, y, z = vectorValue(vector, 0), vectorValue(vector, 1), vectorValue(vector, 2)
    return NdArray.array({{0, -z, y}, {z, 0, -x}, {-y, x, 0}})
end

function Linalg.det(matrix)
    local rows, cols = matrixShape(matrix)
    assert(rows == cols, "det requires a square matrix")
    local p, _, u = Linalg.lu(matrix)
    local sign = 1
    local permutation, invalid = {}, false
    for row = 0, rows - 1 do
        for col = 0, rows - 1 do
            if at(p, row, col) ~= 0 then permutation[row + 1] = col; break end
        end
        invalid = invalid or permutation[row + 1] == nil
    end
    assert(not invalid, "invalid permutation matrix")
    local visited = {}
    for start = 0, rows - 1 do
        if not visited[start] then
            local length, current = 0, start
            while not visited[current] do visited[current], current, length = true, permutation[current + 1], length + 1 end
            if length % 2 == 0 then sign = -sign end
        end
    end
    local result = sign
    for row = 0, rows - 1 do result = result * at(u, row, row) end
    return result
end

function Linalg.inverse(matrix)
    local rows, cols = matrixShape(matrix)
    assert(rows == cols, "inverse requires a square matrix")
    if rows == 3 then
        -- Preserve Matrix3's inexpensive adjugate path, but return a generic
        -- NdArray and use the same zero-based strided access as the rest of
        -- this module.
        local a, b, c = at(matrix, 0, 0), at(matrix, 0, 1), at(matrix, 0, 2)
        local d, e, f = at(matrix, 1, 0), at(matrix, 1, 1), at(matrix, 1, 2)
        local g, h, i = at(matrix, 2, 0), at(matrix, 2, 1), at(matrix, 2, 2)
        local c00, c01, c02 = e * i - f * h, f * g - d * i, d * h - e * g
        local c10, c11, c12 = c * h - b * i, a * i - c * g, b * g - a * h
        local c20, c21, c22 = b * f - c * e, c * d - a * f, a * e - b * d
        local determinant = a * c00 + b * c01 + c * c02
        assert(math.abs(determinant) > 1e-14, "inverse encountered a singular matrix")
        return NdArray.array({
            {c00 / determinant, c10 / determinant, c20 / determinant},
            {c01 / determinant, c11 / determinant, c21 / determinant},
            {c02 / determinant, c12 / determinant, c22 / determinant},
        })
    end
    local p, l, u = Linalg.lu(matrix)
    return Linalg.solveLU(p, l, u, NdArray.eye(rows))
end

function Linalg.dampedLeastSquares(jacobian, damping)
    local _, cols = matrixShape(jacobian)
    damping = damping or 1e-6
    assert(damping > 0, "damping must be positive")
    local transpose = NdArray.transpose(jacobian)
    local normal = NdArray.add(NdArray.matMul(transpose, jacobian),
        NdArray.mul(NdArray.eye(cols), NdArray.full({}, damping * damping)))
    return Linalg.solveCholesky(Linalg.cholesky(normal), transpose)
end

function Linalg.composeTransform(first, second)
    local rows, cols = matrixShape(first)
    local otherRows, otherCols = matrixShape(second)
    assert(rows == 4 and cols == 4 and otherRows == 4 and otherCols == 4, "transforms must be 4 by 4")
    return NdArray.matMul(first, second)
end

function Linalg.rigidInverse(transform)
    local rows, cols = matrixShape(transform)
    assert(rows == 4 and cols == 4, "rigid transforms must be 4 by 4")
    local result = NdArray.eye(4)
    for row = 0, 2 do
        for col = 0, 2 do put(result, row, col, at(transform, col, row)) end
    end
    for row = 0, 2 do
        local value = 0
        for col = 0, 2 do value = value - at(result, row, col) * at(transform, col, 3) end
        put(result, row, 3, value)
    end
    return result
end

function Linalg.transformPoints(transform, points)
    local rows, cols = matrixShape(transform)
    assert(rows == 4 and cols == 4, "transform must be 4 by 4")
    if #points.shape == 1 then
        assert(points.shape[1] == 3, "a point must have three coordinates")
        local homogeneous = NdArray.array({vectorValue(points, 0), vectorValue(points, 1), vectorValue(points, 2), 1})
        local transformed = NdArray.matMul(transform, homogeneous)
        return NdArray.array({transformed.data[1], transformed.data[2], transformed.data[3]})
    end
    assert(#points.shape == 2 and points.shape[2] == 3, "points must be a vector or an N by 3 matrix")
    local result = NdArray.zeros(points.shape)
    for row = 0, points.shape[1] - 1 do
        local homogeneous = NdArray.array({at(points, row, 0), at(points, row, 1), at(points, row, 2), 1})
        local transformed = NdArray.matMul(transform, homogeneous)
        for col = 0, 2 do put(result, row, col, transformed.data[col + 1]) end
    end
    return result
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
    local rows, cols = matrixShape(matrix)
    if rows == 3 and cols == 3 then return Linalg.eigSymm3(matrix, iterLim, tolerance) end
    return Linalg.eigQr(matrix, iterLim, tolerance)
end

local function vectorNorm(x, y, z)
    return math.sqrt(x * x + y * y + z * z)
end

local function normalize3(x, y, z)
    local length = vectorNorm(x, y, z)
    assert(length > 1e-30, "eigenvector is numerically zero")
    return x / length, y / length, z / length
end

local function dot3(ax, ay, az, bx, by, bz)
    return ax * bx + ay * by + az * bz
end

local function cross3(ax, ay, az, bx, by, bz)
    return ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
end

local function eigenvector3(a, b, c, d, e, f, g, h, i, eigenvalue)
    local r1x, r1y, r1z = a - eigenvalue, b, c
    local r2x, r2y, r2z = d, e - eigenvalue, f
    local r3x, r3y, r3z = g, h, i - eigenvalue
    local x1, y1, z1 = cross3(r1x, r1y, r1z, r2x, r2y, r2z)
    local n1 = vectorNorm(x1, y1, z1)
    local x2, y2, z2 = cross3(r1x, r1y, r1z, r3x, r3y, r3z)
    local n2 = vectorNorm(x2, y2, z2)
    local x3, y3, z3 = cross3(r2x, r2y, r2z, r3x, r3y, r3z)
    local n3 = vectorNorm(x3, y3, z3)
    if n2 > n1 then x1, y1, z1, n1 = x2, y2, z2, n2 end
    if n3 > n1 then x1, y1, z1, n1 = x3, y3, z3, n3 end
    if n1 <= 1e-30 then
        -- A repeated eigenvalue can leave A - lambda I at rank one, so no
        -- pair of rows has a useful cross product.  Any vector perpendicular
        -- to its nonzero row is then a valid member of the eigenspace.
        local x, y, z = r1x, r1y, r1z
        local rowLength = vectorNorm(x, y, z)
        local candidate = vectorNorm(r2x, r2y, r2z)
        if candidate > rowLength then x, y, z, rowLength = r2x, r2y, r2z, candidate end
        candidate = vectorNorm(r3x, r3y, r3z)
        if candidate > rowLength then x, y, z, rowLength = r3x, r3y, r3z, candidate end
        if rowLength <= 1e-30 then return nil end
        local ax, ay, az
        if math.abs(x) <= math.abs(y) and math.abs(x) <= math.abs(z) then
            ax, ay, az = 1, 0, 0
        elseif math.abs(y) <= math.abs(z) then
            ax, ay, az = 0, 1, 0
        else
            ax, ay, az = 0, 0, 1
        end
        x, y, z = cross3(x, y, z, ax, ay, az)
        return normalize3(x, y, z)
    end
    return normalize3(x1, y1, z1)
end

-- Closed-form eigendecomposition for a real symmetric 3 by 3 matrix.  The
-- eigenvalues are descending and eigenvectors are returned as columns.
function Linalg.eigSymm3(matrix, _, tolerance)
    local rows, cols = matrixShape(matrix)
    assert(rows == 3 and cols == 3, "eigSymm3 requires a 3 by 3 matrix")
    tolerance = tolerance or 1e-12
    local a, b, c = at(matrix, 0, 0), at(matrix, 0, 1), at(matrix, 0, 2)
    local d, e, f = at(matrix, 1, 0), at(matrix, 1, 1), at(matrix, 1, 2)
    local g, h, i = at(matrix, 2, 0), at(matrix, 2, 1), at(matrix, 2, 2)
    local scale = math.max(1, math.abs(a), math.abs(b), math.abs(c), math.abs(d),
        math.abs(e), math.abs(f), math.abs(g), math.abs(h), math.abs(i))
    assert(math.abs(b - d) <= tolerance * scale and math.abs(c - g) <= tolerance * scale
        and math.abs(f - h) <= tolerance * scale, "eigSymm3 requires a symmetric matrix")

    local mean = (a + e + i) / 3
    local p1 = b * b + c * c + f * f
    local p2 = (a - mean) * (a - mean) + (e - mean) * (e - mean)
        + (i - mean) * (i - mean) + 2 * p1
    local p = math.sqrt(p2 / 6)
    if p <= 1e-30 then
        return {mean, mean, mean}, NdArray.eye(3), {converged = true, iterations = 0}
    end

    local ba, be, bi = (a - mean) / p, (e - mean) / p, (i - mean) / p
    local determinant = ba * (be * bi - (f / p) * (f / p))
        - (b / p) * ((b / p) * bi - (f / p) * (c / p))
        + (c / p) * ((b / p) * (f / p) - be * (c / p))
    local r = math.max(-1, math.min(1, determinant / 2))
    local phi = math.acos(r) / 3
    local first = mean + 2 * p * math.cos(phi)
    local third = mean + 2 * p * math.cos(phi + 2 * math.pi / 3)
    local second = 3 * mean - first - third

    local v1x, v1y, v1z = eigenvector3(a, b, c, d, e, f, g, h, i, first)
    local v3x, v3y, v3z = eigenvector3(a, b, c, d, e, f, g, h, i, third)
    assert(v1x and v3x, "could not construct eigSymm3 eigenvectors")
    local projection = dot3(v3x, v3y, v3z, v1x, v1y, v1z)
    v3x, v3y, v3z = normalize3(v3x - projection * v1x, v3y - projection * v1y,
        v3z - projection * v1z)
    local v2x, v2y, v2z = cross3(v3x, v3y, v3z, v1x, v1y, v1z)
    v2x, v2y, v2z = normalize3(v2x, v2y, v2z)
    local vectors = NdArray.array({
        {v1x, v2x, v3x},
        {v1y, v2y, v3y},
        {v1z, v2z, v3z},
    })
    return {first, second, third}, vectors, {converged = true, iterations = 0}
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
  assert(NdArray and Linalg, "Matrix3.inverse requires the NdArray Linalg backend")
  local array = NdArray.array({
    {mat[1], mat[2], mat[3]},
    {mat[4], mat[5], mat[6]},
    {mat[7], mat[8], mat[9]},
  })
  return Linalg.inverse(array)
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

local function ndShapeString(shape)
    local dimensions = {}
    for axis, size in ipairs(shape) do dimensions[axis] = tostring(size) end
    return "(" .. table.concat(dimensions, ", ") .. ")"
end

local function ndTruthy(value)
    return value ~= nil and value ~= false and value ~= 0
end

local function ndAsArray(value)
    if type(value) == "table" and value.shape and value.data then return value end
    return NdArray.array(value)
end

local function ndNormalizeDType(dtype)
    if dtype == nil then return "any" end
    assert(dtype == "any" or dtype == "bool" or dtype == "int"
        or dtype == "float" or dtype == "object", "unsupported dtype")
    return dtype
end

local function ndValueDType(value)
    if type(value) == "boolean" then return "bool" end
    if type(value) ~= "number" then return "object" end
    return value == math.floor(value) and "int" or "float"
end

local function ndInferDType(data)
    if #data == 0 then return "any" end
    local dtype = ndValueDType(data[1])
    for index = 2, #data do
        local valueType = ndValueDType(data[index])
        if valueType == "object" or dtype == "object" then
            dtype = "object"
        elseif valueType == "bool" and dtype ~= "bool" then
            dtype = "object"
        elseif dtype == "bool" and valueType ~= "bool" then
            dtype = "object"
        elseif valueType == "float" or dtype == "float" then
            dtype = "float"
        end
    end
    return dtype
end

local function ndRefreshDType(arr)
    if arr.size > 0 then arr.dtype = ndInferDType(arr.data) end
    return arr
end

local function ndWarning(message)
    local text = "NdArray warning: " .. message .. "\n"
    if io and io.stderr then io.stderr:write(text) else print(text) end
end

local function ndBroadcastError(shapes)
    local formatted = {}
    for index, shape in ipairs(shapes) do formatted[index] = ndShapeString(shape) end
    return string.format("operands could not be broadcast together with shapes %s",
        table.concat(formatted, " and "))
end

local function ndViewLocal(arr, shape, stride)
    local result = {
        shape = ndCopyShapeLocal(shape),
        stride = ndCopyShapeLocal(stride),
        data = arr.data,
        offset = arr.offset or 0,
        size = ndSizeLocal(shape),
        dtype = arr.dtype or "any",
    }
    setmetatable(result, NdArray.meta)
    return result
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

function NdArray.array(lst, dtype)
    if type(lst) == "table" and lst.shape and lst.data then
        local result = NdArray.copy(lst)
        if dtype ~= nil then result.dtype = ndNormalizeDType(dtype) end
        return result
    end
    local shape = ndInferShapeLocal(lst)
    local result = NdArray.empty(shape, dtype)
    ndFlattenLocal(lst, result.data)
    assert(#result.data == result.size, "array values must be rectangular")
    if dtype == nil then result.dtype = ndInferDType(result.data) end
    return result
end

NdArray.Array = NdArray.array

function NdArray.zeros(shape, dtype)
    return NdArray.full(shape, 0, dtype or "int")
end

function NdArray.ones(shape, dtype)
    return NdArray.full(shape, 1, dtype or "int")
end

function NdArray.full(shape, fill, dtype)
    local arr = NdArray.empty(shape, dtype or ndValueDType(fill))
    local data = arr.data
    for i=1, arr.size do
        data[i] = fill
    end
    return arr
end

function NdArray.empty(shape, dtype)
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
        dtype = ndNormalizeDType(dtype),
    }
    setmetatable(arr, NdArray.meta)
    return arr
end

function NdArray.zerosLike(arr)
    return NdArray.zeros(arr.shape, arr.dtype)
end

function NdArray.onesLike(arr)
    return NdArray.ones(arr.shape, arr.dtype)
end

function NdArray.fullLike(arr, fill)
    return NdArray.full(arr.shape, fill, arr.dtype)
end

function NdArray.emptyLike(arr)
    return NdArray.empty(arr.shape, arr.dtype)
end

-- Lua cannot implicitly coerce a table to a number.  This is the explicit
-- scalar boundary for rank-zero arrays and is also used by scalar indexing.
function NdArray.item(arr)
    assert(type(arr) == "table" and arr.shape and arr.data, "expected an NdArray")
    assert(#arr.shape == 0, "item requires a zero-dimensional NdArray")
    return arr.data[(arr.offset or 0) + 1]
end

NdArray.scalar = NdArray.item

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

-- Specialized C-order walkers retain the same zero-based offset contract as
-- _strideIter, but keep operand offsets in locals for the 1/2/3-operand paths.
function NdArray._strideIter1(arr)
    local shape = arr.shape
    local stride = arr.stride
    local ndim = #shape
    local idxs, offset = {}, arr.offset or 0
    for axis = 1, ndim do idxs[axis] = 0 end
    if ndim == 0 then
        local first = true
        return function()
            if not first then return nil end
            first = false
            return idxs, offset
        end
    end
    local inner_limit, inner_stride = shape[ndim], stride[ndim]
    local inner_idx, done = 0, ndSizeLocal(shape) == 0
    offset = offset - inner_stride
    return function()
        if done then return nil end
        if inner_idx < inner_limit then
            inner_idx = inner_idx + 1
            offset = offset + inner_stride
            idxs[ndim] = inner_idx
            return idxs, offset
        end
        inner_idx = 0
        idxs[ndim] = 0
        offset = offset - inner_stride * (inner_limit - 1)
        if ndim == 1 then
            done = true
            return nil
        end
        for d = ndim - 1, 1, -1 do
            if idxs[d] + 1 < shape[d] then
                idxs[d] = idxs[d] + 1
                offset = offset + stride[d]
                break
            else
                idxs[d] = 0
                offset = offset - stride[d] * (shape[d] - 1)
                if d == 1 then
                    done = true
                    return nil
                end
            end
        end
        inner_idx = 1
        idxs[ndim] = 1
        return idxs, offset
    end
end

function NdArray._strideIter2(arr1, arr2)
    local shape = arr1.shape
    local stride1 = arr1.stride
    local stride2 = arr2.stride
    local ndim = #shape
    local idxs, offset1, offset2 = {}, arr1.offset or 0, arr2.offset or 0
    for axis = 1, ndim do idxs[axis] = 0 end
    if ndim == 0 then
        local first = true
        return function()
            if not first then return nil end
            first = false
            return idxs, offset1, offset2
        end
    end
    local inner_limit = shape[ndim]
    local inner_stride1, inner_stride2 = stride1[ndim], stride2[ndim]
    local inner_idx, done = 0, ndSizeLocal(shape) == 0
    offset1, offset2 = offset1 - inner_stride1, offset2 - inner_stride2
    return function()
        if done then return nil end
        if inner_idx < inner_limit then
            inner_idx = inner_idx + 1
            offset1, offset2 = offset1 + inner_stride1, offset2 + inner_stride2
            idxs[ndim] = inner_idx
            return idxs, offset1, offset2
        end
        inner_idx = 0
        idxs[ndim] = 0
        offset1 = offset1 - inner_stride1 * (inner_limit - 1)
        offset2 = offset2 - inner_stride2 * (inner_limit - 1)
        if ndim == 1 then
            done = true
            return nil
        end
        for d = ndim - 1, 1, -1 do
            if idxs[d] + 1 < shape[d] then
                idxs[d] = idxs[d] + 1
                offset1, offset2 = offset1 + stride1[d], offset2 + stride2[d]
                break
            else
                idxs[d] = 0
                offset1 = offset1 - stride1[d] * (shape[d] - 1)
                offset2 = offset2 - stride2[d] * (shape[d] - 1)
                if d == 1 then
                    done = true
                    return nil
                end
            end
        end
        inner_idx = 1
        idxs[ndim] = 1
        return idxs, offset1, offset2
    end
end

function NdArray._strideIter3(arr1, arr2, arr3)
    local shape = arr1.shape
    local stride1 = arr1.stride
    local stride2 = arr2.stride
    local stride3 = arr3.stride
    local ndim = #shape
    local idxs, offset1, offset2, offset3 = {}, arr1.offset or 0, arr2.offset or 0, arr3.offset or 0
    for axis = 1, ndim do idxs[axis] = 0 end
    if ndim == 0 then
        local first = true
        return function()
            if not first then return nil end
            first = false
            return idxs, offset1, offset2, offset3
        end
    end
    local inner_limit = shape[ndim]
    local inner_stride1, inner_stride2, inner_stride3 = stride1[ndim], stride2[ndim], stride3[ndim]
    local inner_idx, done = 0, ndSizeLocal(shape) == 0
    offset1, offset2, offset3 = offset1 - inner_stride1, offset2 - inner_stride2, offset3 - inner_stride3
    return function()
        if done then return nil end
        if inner_idx < inner_limit then
            inner_idx = inner_idx + 1
            offset1, offset2, offset3 = offset1 + inner_stride1, offset2 + inner_stride2, offset3 + inner_stride3
            idxs[ndim] = inner_idx
            return idxs, offset1, offset2, offset3
        end
        inner_idx = 0
        idxs[ndim] = 0
        offset1 = offset1 - inner_stride1 * (inner_limit - 1)
        offset2 = offset2 - inner_stride2 * (inner_limit - 1)
        offset3 = offset3 - inner_stride3 * (inner_limit - 1)
        if ndim == 1 then
            done = true
            return nil
        end
        for d = ndim - 1, 1, -1 do
            if idxs[d] + 1 < shape[d] then
                idxs[d] = idxs[d] + 1
                offset1, offset2, offset3 = offset1 + stride1[d], offset2 + stride2[d], offset3 + stride3[d]
                break
            else
                idxs[d] = 0
                offset1 = offset1 - stride1[d] * (shape[d] - 1)
                offset2 = offset2 - stride2[d] * (shape[d] - 1)
                offset3 = offset3 - stride3[d] * (shape[d] - 1)
                if d == 1 then
                    done = true
                    return nil
                end
            end
        end
        inner_idx = 1
        idxs[ndim] = 1
        return idxs, offset1, offset2, offset3
    end
end

function NdArray.copy(arr)
    assert(type(arr) == "table" and arr.shape and arr.stride and arr.data, "expected an NdArray")
    local ret = NdArray.empty(arr.shape, arr.dtype)
    local position = 1
    for _, values in NdArray.nditer(arr, nil, "C") do
        ret.data[position], position = values[1], position + 1
    end
    return ndRefreshDType(ret)
end

function NdArray.copyTo(dst, src, where)
    if where == false then return end
    if where == nil or where == true then
        dst, src = table.unpack(NdArray.broadcastArrays({dst, src}))
        for offsets in NdArray._strideIter(dst.shape, {dst.stride, src.stride}, {dst.offset or 0, src.offset or 0}) do
            dst.data[offsets[1] + 1] = src.data[offsets[2] + 1]
        end
    else
        dst, src, where = table.unpack(NdArray.broadcastArrays({dst, src, where}))
        for offsets in NdArray._strideIter(dst.shape, {dst.stride, src.stride, where.stride},
            {dst.offset or 0, src.offset or 0, where.offset or 0}) do
            if ndTruthy(where.data[offsets[3] + 1]) then
                dst.data[offsets[1] + 1] = src.data[offsets[2] + 1]
            end
        end
    end
end

function NdArray._reshapeIsView(shape1, shape2, stride1)
    if ndSizeLocal(shape1) ~= ndSizeLocal(shape2) then return false, nil end

    -- A zero-sized view has no addresses to preserve.  Give it ordinary
    -- C-order strides so a later reshape can continue to reason about it.
    if ndSizeLocal(shape1) == 0 then
        local stride, step = {}, 1
        for axis = #shape2, 1, -1 do stride[axis], step = step, step * shape2[axis] end
        return true, stride
    end

    -- Remove singleton source dimensions and group adjacent dimensions whose
    -- addresses are C-contiguous.  A reshape view may split or join one of
    -- these groups, but it cannot cross a non-contiguous boundary.
    local blocks = {}
    local blockSize, blockStride, previousStride
    for axis, size in ipairs(shape1) do
        if size > 1 then
            local stride = stride1[axis]
            if not blockSize then
                blockSize, blockStride = size, stride
            elseif previousStride == size * stride then
                blockSize, blockStride = blockSize * size, stride
            else
                blocks[#blocks + 1] = {size = blockSize, stride = blockStride}
                blockSize, blockStride = size, stride
            end
            previousStride = stride
        end
    end
    if blockSize then blocks[#blocks + 1] = {size = blockSize, stride = blockStride} end

    -- Match target dimensions against source blocks from the inner axis out.
    -- Singleton target dimensions consume no source elements and may be given
    -- any stride; zero is both cheap and unambiguous.
    local outputStride = {}
    local block = #blocks
    local remaining = block > 0 and blocks[block].size or 1
    local innerSize = 1
    for axis = #shape2, 1, -1 do
        local size = shape2[axis]
        if size == 1 then
            outputStride[axis] = 0
        else
            if block == 0 then return false, nil end
            outputStride[axis] = blocks[block].stride * innerSize
            innerSize = innerSize * size
            if innerSize > remaining then return false, nil end
            if innerSize == remaining then
                block = block - 1
                remaining = block > 0 and blocks[block].size or 1
                innerSize = 1
            end
        end
    end
    if block ~= 0 or innerSize ~= 1 then return false, nil end
    return true, outputStride
end

function NdArray.reshape(arr, ...)
    local requested = {...}
    local shape = #requested == 1 and type(requested[1]) == "table"
        and ndCopyShapeLocal(requested[1]) or requested
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
    local valid, stride = NdArray._reshapeIsView(arr.shape, shape, arr.stride)
    local source = valid and arr or NdArray.copy(arr)
    if not valid then
        stride = {}
        local step = 1
        for axis = #shape, 1, -1 do stride[axis], step = step, step * shape[axis] end
    end
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
    local operands = {...}
    for i = 1, #operands do operands[i] = ndAsArray(operands[i]) end
    local broadcasted = NdArray.broadcastArrays(operands)
    local n = #broadcasted
    local ret = NdArray.empty(broadcasted[1].shape)
    local args = {}
    local position = 1
    for _, values in NdArray.nditer(broadcasted, nil, "C") do
        for i = 1, n do args[i] = values[i] end
        ret.data[position], position = func(table.unpack(args, 1, n)), position + 1
    end
    return ndRefreshDType(ret)
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
                values[i] = arr.data[offset + (i - 1) * stride + 1]
            end
            table.sort(values)
            for i=1,n do
                ret.data[offset + (i - 1) * stride + 1] = values[i]
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
        for i=1, n do
            idxs[i] = i - 1
        end
        for _, offset in NdArray._strideIter1(ret) do
            table.sort(idxs, function(a, b) return arr.data[offset + a * stride + 1] < arr.data[offset + b * stride + 1] end)
            for i=1,n do
                ret.data[offset + (i - 1) * stride + 1] = idxs[i]
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
    assert(padding >= 0, ndBroadcastError({arrShape, targetShape}))
    for axis = 1, #targetShape do
        local sourceAxis = axis - padding
        if sourceAxis <= 0 then
            stride[axis] = 0
        else
            local size = arrShape[sourceAxis]
            assert(size == 1 or size == targetShape[axis], ndBroadcastError({arrShape, targetShape}))
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
                ndBroadcastError(shapes))
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
    local count = 0
    for idxs, values in NdArray.nditer(arr, nil, "C") do
        if ndTruthy(values[1]) then
            count = count + 1
            for axis = 1, ndim do coordinates[axis][count] = idxs[axis] end
        end
    end
    for axis = 1, ndim do
        coordinates[axis] = NdArray.array(coordinates[axis], "int")
    end
    return table.unpack(coordinates, 1, ndim)
end

function NdArray.countNonzero(arr)
    local count = 0
    for _, values in NdArray.nditer(arr, nil, "C") do
        if ndTruthy(values[1]) then count = count + 1 end
    end
    return count
end

function NdArray.any(arr)
    return NdArray.countNonzero(arr) > 0
end

function NdArray.all(arr)
    assert(type(arr) == "table" and arr.shape and arr.data, "expected an NdArray")
    return NdArray.countNonzero(arr) == arr.size
end

function NdArray.argwhere(arr)
    assert(type(arr) == "table" and arr.shape and arr.data, "expected an NdArray")
    local ndim, count, position = #arr.shape, 0, 0
    local data = {}
    for idx, values in NdArray.nditer(arr, nil, "C") do
        if ndTruthy(values[1]) then
            count = count + 1
            for axis = 1, ndim do
                position = position + 1
                data[position] = idx[axis]
            end
        end
    end
    return NdArray.reshape(NdArray.array(data, "int"), {count, ndim})
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
        if ndTruthy(cond.data[condOfs + 1]) then
            ret.data[retOfs + 1] = ifTrue.data[trueOfs + 1]
        else
            ret.data[retOfs + 1] = ifFalse.data[falseOfs + 1]
        end
    end
    
    return ndRefreshDType(ret)
end

NdArray._sliceMt = {
    __index=function(table, key)
        if type(key) ~= "number" then return nil end
        return table.start + key * table.step
    end,
    __len=function(table)
        if table.start == nil or table.stop == nil then return 0 end
        return math.max(0, math.ceil((table.stop - table.start) / table.step))
    end,
}

function NdArray.slice(start, stop, step)
    step = step or 1
    assert(type(step) == "number" and step == math.floor(step) and step ~= 0,
        "slice step must be a non-zero integer")
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

local function ndSliceBounds(dim, start, stop, step)
    if step > 0 then
        start = start == nil and 0 or start
        stop = stop == nil and dim or stop
        if start < 0 then start = start + dim end
        if stop < 0 then stop = stop + dim end
        start, stop = math.max(0, math.min(dim, start)), math.max(0, math.min(dim, stop))
        return start, stop, step, math.max(0, math.ceil((stop - start) / step))
    end
    local omittedStop = stop == nil
    start = start == nil and dim - 1 or start
    stop = stop == nil and -1 or stop
    if start < 0 then start = start + dim end
    if stop < 0 and not omittedStop then stop = stop + dim end
    start = math.max(-1, math.min(dim - 1, start))
    stop = math.max(-1, math.min(dim - 1, stop))
    return start, stop, step, math.max(0, math.ceil((start - stop) / -step))
end

local function ndIsBooleanArray(arr)
    if arr.dtype == "bool" then return true end
    if arr.size == 0 then return false end
    for _, values in NdArray.nditer(arr, nil, "C") do
        if type(values[1]) ~= "boolean" then return false end
    end
    return true
end

local function ndIndexArray(index)
    if type(index) == "table" and index.shape and index.data then return index end
    assert(type(index) == "table", "advanced indices must be arrays or Lua tables")
    return NdArray.array(index)
end

local function ndIndexBooleanRank(index)
    if type(index) ~= "table" or getmetatable(index) == NdArray._sliceMt then return nil end
    local advanced = ndIndexArray(index)
    if ndIsBooleanArray(advanced) then return #advanced.shape end
    return nil
end

local function ndScalarIndexToken(token)
    if type(token) == "table" and token.shape and token.data and #token.shape == 0 then
        return NdArray.item(token)
    end
    return token
end

local function ndNormalizeIndex(index, dimension, axis)
    assert(type(index) == "number" and index == math.floor(index),
        string.format("All indices must be integers; got %s on dimension %d", tostring(index), axis))
    if index < 0 then index = index + dimension end
    assert(index >= 0 and index < dimension,
        string.format("Index %d is out of bounds for dimension %d with size %d", index, axis, dimension))
    return index
end

-- Build one coordinate map for basic, fancy, and boolean indexing. Basic
-- entries remain views; only advanced entries use the materialized map.
local function ndIndexPlan(arr, idxs)
    idxs = idxs or {}
    assert(type(idxs) == "table", "indices must be supplied as a table")
    local raw = {}
    for position, token in ipairs(idxs) do raw[position] = ndScalarIndexToken(token) end
    local consumed, ellipsis = 0, nil
    for position, token in ipairs(raw) do
        if token == NdArray.ellipsis then
            assert(not ellipsis, "an index may contain only one ellipsis")
            ellipsis = position
        elseif token ~= NdArray.newaxis and type(token) ~= "boolean" then
            consumed = consumed + (ndIndexBooleanRank(token) or 1)
        end
    end
    assert(consumed <= #arr.shape, "too many indices for array")
    local expanded, inserted = {}, false
    for position = 1, #raw + 1 do
        if position == ellipsis then
            for _ = 1, #arr.shape - consumed do expanded[#expanded + 1] = NdArray.sliceAll end
            inserted = true
        elseif position <= #raw then expanded[#expanded + 1] = raw[position] end
    end
    if not inserted then for _ = consumed + 1, #arr.shape do expanded[#expanded + 1] = NdArray.sliceAll end end

    local sourceAxis, entries, fancyArrays = 1, {}, {}
    for position, token in ipairs(expanded) do
        if token == NdArray.newaxis then
            entries[#entries + 1] = {kind = "newaxis"}
        elseif type(token) == "boolean" then
            entries[#entries + 1] = {kind = "boolScalar", value = token}
        else
            assert(sourceAxis <= #arr.shape, "too many indices for array")
            if type(token) == "number" then
                entries[#entries + 1] = {kind = "index", axis = sourceAxis,
                    value = ndNormalizeIndex(token, arr.shape[sourceAxis], sourceAxis)}
                sourceAxis = sourceAxis + 1
            elseif type(token) == "table" and getmetatable(token) == NdArray._sliceMt then
                local start, stop, step, size = ndSliceBounds(arr.shape[sourceAxis], token.start, token.stop, token.step)
                entries[#entries + 1] = {kind = "slice", axis = sourceAxis, start = start, step = step, size = size}
                sourceAxis = sourceAxis + 1
            elseif type(token) == "table" then
                local advanced = ndIndexArray(token)
                if ndIsBooleanArray(advanced) then
                    local rank = #advanced.shape
                    assert(rank > 0, "a zero-dimensional boolean index must be a scalar boolean")
                    assert(sourceAxis + rank - 1 <= #arr.shape,
                        "boolean index has more dimensions than the indexed array")
                    for axis = 1, rank do
                        assert(advanced.shape[axis] == arr.shape[sourceAxis + axis - 1],
                            ndBroadcastError({advanced.shape, {table.unpack(arr.shape, sourceAxis, sourceAxis + rank - 1)}}))
                    end
                    local coordinates = {NdArray.nonzero(advanced)}
                    for axis = 1, rank do
                        local coordinate = coordinates[axis]
                        entries[#entries + 1] = {kind = "fancy", axis = sourceAxis + axis - 1,
                            array = coordinate}
                        fancyArrays[#fancyArrays + 1] = coordinate
                    end
                    sourceAxis = sourceAxis + rank
                else
                    entries[#entries + 1] = {kind = "fancy", axis = sourceAxis, array = advanced}
                    fancyArrays[#fancyArrays + 1] = advanced
                    sourceAxis = sourceAxis + 1
                end
            else
                error(string.format("unsupported index type %s at position %d", type(token), position))
            end
        end
    end
    local fancyShape
    if #fancyArrays > 0 then
        for _, entry in ipairs(entries) do
            if entry.kind == "boolScalar" then
                error("boolean scalar indices cannot be combined with array indices")
            end
        end
        local broadcasted = NdArray.broadcastArrays(fancyArrays)
        fancyShape = broadcasted[1].shape
        local fancyIndex = 0
        for _, entry in ipairs(entries) do
            if entry.kind == "fancy" then fancyIndex = fancyIndex + 1; entry.array = broadcasted[fancyIndex] end
        end
    else
        for _, entry in ipairs(entries) do
            if entry.kind == "boolScalar" then
                fancyShape = {entry.value and 1 or 0}
                break
            end
        end
        fancyShape = fancyShape or {}
    end
    local firstFancy, advancedSeparated, sawBasicAfterFancy = nil, false, false
    for position, entry in ipairs(entries) do
        local isFancy = entry.kind == "fancy" or entry.kind == "boolScalar"
        if isFancy then
            if not firstFancy then firstFancy = position end
            if sawBasicAfterFancy then advancedSeparated = true end
        elseif firstFancy then
            sawBasicAfterFancy = true
        end
    end
    local shape, fancyStart = {}, nil
    if firstFancy and advancedSeparated then
        fancyStart = 1
        for _, size in ipairs(fancyShape) do shape[#shape + 1] = size end
        for _, entry in ipairs(entries) do
            if entry.kind == "slice" then entry.outputAxis = #shape + 1; shape[#shape + 1] = entry.size
            elseif entry.kind == "newaxis" then entry.outputAxis = #shape + 1; shape[#shape + 1] = 1 end
        end
    else
        for _, entry in ipairs(entries) do
            if entry.kind == "fancy" or entry.kind == "boolScalar" then
                if not fancyStart then
                    fancyStart = #shape + 1
                    for _, size in ipairs(fancyShape) do shape[#shape + 1] = size end
                end
            elseif entry.kind == "slice" then
                entry.outputAxis = #shape + 1; shape[#shape + 1] = entry.size
            elseif entry.kind == "newaxis" then
                entry.outputAxis = #shape + 1; shape[#shape + 1] = 1
            end
        end
    end
    return {entries = entries, shape = shape, fancyShape = fancyShape, fancyStart = fancyStart}
end

local function ndIndexOffset(arr, plan, outputIndices)
    local offset = arr.offset or 0
    local fancyCoordinate = {}
    if plan.fancyStart then
        for axis = 1, #plan.fancyShape do fancyCoordinate[axis] = outputIndices[plan.fancyStart + axis - 1] end
    end
    for _, entry in ipairs(plan.entries) do
        if entry.kind == "index" then
            offset = offset + entry.value * arr.stride[entry.axis]
        elseif entry.kind == "slice" then
            offset = offset + (entry.start + outputIndices[entry.outputAxis] * entry.step) * arr.stride[entry.axis]
        elseif entry.kind == "newaxis" then
        elseif entry.kind == "fancy" then
            local coordinateOffset = entry.array.offset or 0
            for axis = 1, #plan.fancyShape do coordinateOffset = coordinateOffset + fancyCoordinate[axis] * entry.array.stride[axis] end
            local value = entry.array.data[coordinateOffset + 1]
            offset = offset + ndNormalizeIndex(value, arr.shape[entry.axis], entry.axis) * arr.stride[entry.axis]
        end
    end
    return offset
end

function NdArray._getOffset(arr, idxs)
    local offset = arr.offset or 0
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

-- Final indexing entry points use one normalized plan for basic, fancy, and
-- boolean indexing. The scalar helpers above remain for internal callers.
function NdArray.get(arr, idxs)
    assert(type(arr) == "table" and arr.shape and arr.stride and arr.data, "expected an NdArray")
    local plan = ndIndexPlan(arr, idxs)
    if not plan.fancyStart then
        local ret = {}
        for key, value in pairs(arr) do ret[key] = value end
        local metatable = getmetatable(arr); if metatable then setmetatable(ret, metatable) end
        local offset, shape, stride = arr.offset or 0, {}, {}
        for _, entry in ipairs(plan.entries) do
            if entry.kind == "index" then offset = offset + entry.value * arr.stride[entry.axis]
            elseif entry.kind == "slice" then
                shape[entry.outputAxis], stride[entry.outputAxis] = entry.size, arr.stride[entry.axis] * entry.step
                offset = offset + entry.start * arr.stride[entry.axis]
            else shape[entry.outputAxis], stride[entry.outputAxis] = 1, 0 end
        end
        ret.shape, ret.stride, ret.offset, ret.size = shape, stride, offset, ndSizeLocal(shape)
        return ret
    end
    local ret, position = NdArray.empty(plan.shape, arr.dtype), 1
    local zeroStride = {}; for axis = 1, #plan.shape do zeroStride[axis] = 0 end
    local iterator = NdArray._strideIter(plan.shape, {zeroStride}, {0}, "C", true)
    while true do
        local indices = iterator(); if not indices then break end
        ret.data[position] = arr.data[ndIndexOffset(arr, plan, indices) + 1]
        position = position + 1
    end
    return ret
end

function NdArray.set(arr, idxs, vals)
    assert(type(arr) == "table" and arr.shape and arr.stride and arr.data, "expected an NdArray")
    local plan = ndIndexPlan(arr, idxs)
    if not plan.fancyStart then
        if type(vals) ~= "table" or not vals.shape or not vals.data then vals = NdArray.array(vals) end
        if vals.data == arr.data then
            ndWarning("assignment source shares storage with destination; use an explicit copy to avoid overlap ambiguity")
        end
        NdArray.copyTo(NdArray.get(arr, idxs), vals); return
    end
    if type(vals) ~= "table" or not vals.shape or not vals.data then vals = NdArray.array(vals) end
    if vals.data == arr.data then
        ndWarning("assignment source shares storage with destination; use an explicit copy to avoid overlap ambiguity")
    end
    vals = NdArray.broadcastTo(NdArray.copy(vals), plan.shape)
    local zeroStride = {}; for axis = 1, #plan.shape do zeroStride[axis] = 0 end
    local iterator = NdArray._strideIter(plan.shape, {zeroStride}, {0}, "C", true)
    while true do
        local indices = iterator(); if not indices then break end
        local valueOffset = vals.offset or 0
        for axis = 1, #plan.shape do valueOffset = valueOffset + indices[axis] * vals.stride[axis] end
        arr.data[ndIndexOffset(arr, plan, indices) + 1] = vals.data[valueOffset + 1]
    end
end

-- Minimal iterator layers for the generic array representation. An operand
-- only needs shape, stride, data, and an optional offset.
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

local function ndOrder(order, ndim, strides)
    if order == nil or order == "C" then
        local result = {}
        for axis = 1, ndim do result[axis] = axis end
        return result
    end
    if order == "K" then
        assert(strides, "K order requires operand strides")
        local result, costs = {}, {}
        for axis = 1, ndim do
            local cost = 0
            for _, operandStrides in ipairs(strides) do
                local stride = math.abs(operandStrides[axis] or 0)
                if stride > cost then cost = stride end
            end
            costs[axis] = cost
            result[axis] = axis
        end

        -- `result` is ordered from outermost to innermost. Keep the
        -- largest-stride axes outermost so the smallest stride is advanced
        -- most frequently. Insertion sort keeps ties in logical-axis order
        -- and avoids a comparator closure in this one-time setup path.
        for position = 2, ndim do
            local axis, cost = result[position], costs[result[position]]
            local insert = position - 1
            while insert >= 1 and costs[result[insert]] < cost do
                result[insert + 1] = result[insert]
                insert = insert - 1
            end
            result[insert + 1] = axis
        end
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

-- Lowest-level generic traversal.  It advances logical offsets only; it never
-- reads an operand and it reuses its offset table on every yield.  The
-- optional `withIndices` result is for layers that also need coordinates.
function NdArray._strideIter(shape, strides, baseOffsets, order, withIndices)
    local ndim = #shape
    local positions, offsets = {}, {}
    for axis = 1, ndim do positions[axis] = 0 end
    local indices = withIndices and positions or nil
    for operand, offset in ipairs(baseOffsets) do offsets[operand] = offset or 0 end
    local axisOrder = ndOrder(order, ndim, strides)
    local first, done = true, ndSize(shape) == 0
    return function()
        if done then return nil end
        if first then
            first = false
            if indices then return indices, offsets end
            return offsets
        end
        if ndim == 0 then done = true; return nil end
        for position = ndim, 1, -1 do
            local axis = axisOrder[position]
            local index = positions[axis]
            if index + 1 < shape[axis] then
                positions[axis] = index + 1
                for operand, stride in ipairs(strides) do
                    offsets[operand] = offsets[operand] + stride[axis]
                end
                if indices then return indices, offsets end
                return offsets
            end
            positions[axis] = 0
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
                ndBroadcastError(shapes))
            if size ~= 1 then result[axis] = size end
        end
    end
    return result
end

local function ndAlignedStride(arr, targetShape)
    local padding = #targetShape - #arr.shape
    assert(padding >= 0, ndBroadcastError({arr.shape, targetShape}))
    local result = {}
    for axis = 1, #targetShape do
        local sourceAxis = axis - padding
        if sourceAxis <= 0 then
            result[axis] = 0
        else
            local size = arr.shape[sourceAxis]
            assert(size == 1 or size == targetShape[axis],
                ndBroadcastError({arr.shape, targetShape}))
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
                                ndBroadcastError({arr.shape, shape}))
                            shape[iteratorAxis] = size
                        end
                    else
                        assert(size == 1 or size == shape[iteratorAxis],
                            ndBroadcastError({arr.shape, shape}))
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
    local iterator = NdArray._strideIter(shape, strides, offsets, order, true)
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
                    ndBroadcastError({shape, loopShape}))
                loopStrides[operand][globalAxis] = size == 1 and 0
                    or arr.stride[axes[localAxis] + 1]
            end
        end
        offsets[operand] = arr.offset or 0
    end
    return loopShape, loopStrides, offsets, coreShapes, coreStrides
end

local function ndView(arr, shape, stride, offset)
    local result = {
        shape = shape, stride = stride, data = arr.data, offset = offset,
        size = ndSize(shape), dtype = arr.dtype or "any",
    }
    setmetatable(result, NdArray.meta)
    return result
end

-- Direct-axis core iterator.  Unlike nditer, this preserves the selected
-- axes as reusable N-D views and broadcasts only the remaining loop axes.
function NdArray.coreIter(arrs, coreAxes, order, iterShape)
    if arrs and arrs.shape then arrs = {arrs} end
    assert(type(arrs) == "table" and #arrs > 0, "coreIter requires operands")
    coreAxes = coreAxes or {}
    local loopShape, loopStrides, offsets, coreShapes, coreStrides =
        ndNormalizeCore(arrs, coreAxes, iterShape)
    local iterator = NdArray._strideIter(loopShape, loopStrides, offsets, order, true)
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

local function ndContiguousArray(shape, dtype)
    local result, stride = {}, {}
    local step = 1
    for axis = #shape, 1, -1 do
        stride[axis] = step
        step = step * shape[axis]
    end
    result.shape, result.stride, result.data, result.offset, result.size, result.dtype =
        ndCopyShape(shape), stride, {}, 0, step, ndNormalizeDType(dtype)
    setmetatable(result, NdArray.meta)
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
    local iterator = NdArray._strideIter(coreShape, {result.stride}, {result.offset or 0}, "C", true)
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
        local iterator = NdArray._strideIter(loopShape, loopStrides, offsets, "C", true)
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
NdArray.equal = function(a, b) return NdArray._elementwise(function(x, y) return x == y end, a, b) end
NdArray.notEqual = function(a, b) return NdArray._elementwise(function(x, y) return x ~= y end, a, b) end
NdArray.less = function(a, b) return NdArray._elementwise(function(x, y) return x < y end, a, b) end
NdArray.lessEqual = function(a, b) return NdArray._elementwise(function(x, y) return x <= y end, a, b) end
NdArray.greater = function(a, b) return NdArray._elementwise(function(x, y) return x > y end, a, b) end
NdArray.greaterEqual = function(a, b) return NdArray._elementwise(function(x, y) return x >= y end, a, b) end

local function ndWriteTraversal(source, output, axisOffset, axis)
    local iterator = NdArray._strideIter(source.shape, {source.stride}, {source.offset or 0}, "C", true)
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
        ndWriteTraversal(arr, result, offset, axis)
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

local function ndComparisonOperatorError()
    error("NdArray comparison operators are disabled; use NdArray.equal, NdArray.notEqual, "
        .. "NdArray.less, NdArray.lessEqual, NdArray.greater, or NdArray.greaterEqual", 2)
end

-- Lua has no __bool metamethod, but its arithmetic and comparison operators
-- can be redirected. Elementwise comparisons remain named functions because
-- Lua comparison expressions must produce one boolean rather than an array.
NdArray.meta = {
    __add = NdArray.add,
    __sub = NdArray.sub,
    __mul = NdArray.mul,
    __div = NdArray.div,
    __unm = NdArray.negative,
    __eq = ndComparisonOperatorError,
    __lt = ndComparisonOperatorError,
    __le = ndComparisonOperatorError,
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

Stats = Stats or {}

function Stats.Distribution(vars)
  local distr = { n = 0, vars = vars }
  if vars then
    local mean = {}
    local nVars = #vars
    local cov = NdArray.zeros({nVars, nVars})
    for i, v in ipairs(vars) do
      mean[v] = 0
    end
    distr.mean = mean
    distr.cov = cov
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
        local index = (i - 1) * nVars + j
        local symmetricIndex = (j - 1) * nVars + i
        distr.cov.data[index] = (distr.cov.data[index] * oldN + upd) / distr.n
        distr.cov.data[symmetricIndex] = distr.cov.data[index]
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
        local index = (i - 1) * nVars + j
        local symmetricIndex = (j - 1) * nVars + i
        distr.cov.data[index] = (distr.cov.data[index] * oldN + sum) / distr.n
        distr.cov.data[symmetricIndex] = distr.cov.data[index]
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
          return distr.cov.data[(i - 1) * nVars + j]
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
