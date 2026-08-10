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
