function Linalg.fft()

function Linalg.dot(a, b)
  local sum = 0
  for i, x in ipairs(a) do
    sum = sum + x * b[i]
  end
  return sum
end

function Linalg.norm(a)
  return math.sqrt(Linalg.dot(a, a))
end

-- Matrix solving
-- direct solve using the LU decomposition
function Linalg.solveLU(P, L, U, b)

end

-- direct solve using the QR decomposition
function Linalg.solveQR(Q, R, b)

end

-- preconditioned conjugate gradient using a basic (Jacobi) preconditioner
function Linalg.solvePCG(A, b, P, x0)

end


-- Matrix Decompositions
-- LU decomposition with partial pivoting. Returns P, L, U
function Linalg.lu(mat, pivoting)
  if pivoting == nil then
    pivoting = true
  end
  local r = 3
  local c = 3
  local permutation = {}
  local U = Matrix3.Matrix3(mat)
  local L = Matrix3.Identity()
  for col=1,c do
    -- find pivot
    local k = col
    local pivot = Matrix3.get(U, col, col)
    if pivoting then
      for row=col,r do
        local candidate = Matrix3.get(U, row, col)
        if math.abs(candidate) > math.abs(pivot) then
          pivot = candidate
          k = row
        end
      end
      Matrix3.rowSwap(U, k, col)
    end
    permutation[col] = k

    if not MathUtil.isZero(pivot) then
      -- eliminate lower entries
      for row=col+1,r do
        local multiplier = Matrix3.get(U, row, col) / pivot
        Matrix3.rowAdd(U, col, row, -multiplier)
        Matrix3.set(L, row, col, multiplier)
      end
    end
  end

  return permutation, L, U
end

function Linalg.qr(mat)
  local r = 3
  local c = 3
  local R = NdArray.Array(mat)
  for col=1, c do
    local x = NdArray.getCol(R, col)
    local alpha = Linalg.norm(x) * (x[col] >= 0 and 1 or -1)
    local u = Vector3.Vector3(x)
    u[col] = u[col] - alpha
    local v = u / Linalg.norm(u)
    for row=col, r do
      local dot = Linalg.dot(x, v)
      Matrix3.
end

function Linalg.svd(mat)

end

-- finds the eigendecomposition of a symmetric matrix
function Linalg.eigSymm(mat)

end

-- finds the eigendecomposition of a symmetrix 2x2 matrix
function Linalg.eigSymm2(mat)
  local a = Matrix3.get(mat, 1, 1)
  local b = Matrix3.get(mat, 2, 2)
  local c = Matrix3.get(mat, 2, 1)
  local delta = math.sqrt(4 * c * c + (a - b) * (a - b))
  local lambda1 = 0.5 * (a + b - delta)
  local lambda2 = 0.5 * (a + b + delta)
  local v1 = {(lambda2 - b) / c, 1}
  local v2 = {(lambda1 - b) / c, 1}
  return {lambda1, lambda2}, {v1, v2}
end

-- finds the eigendecomposition of a symmetrix 3x3 matrix
function Linalg.eigSymm3(mat)
  local a = Matrix3.get(mat, 1, 1)
  local b = Matrix3.get(mat, 2, 2)
  local c = Matrix3.get(mat, 3, 3)
  local d = Matrix3.get(mat, 2, 1)
  local e = Matrix3.get(mat, 3, 2)
  local f = Matrix3.get(mat, 3, 1)
    local x1 = a * a + b * b + c * c - a * b - a * c - b * c + 3 * (d * d + f * f + e * e)
  local abc = 2 * a - b - c
  local bac = 2 * b - a - c
  local cab = 2 * c - a - b
  local x2 = -abc * bac * cab + 9 * (cab * d * d + bac * f * f + abc * e * e) - 54 * d * e * f
  local phi = math.pi * 0.5
  if x2 > 0 then
    phi = math.atan(math.sqrt(4 * x1 * x1 * x1 - x2 * x2) / x2)
  elseif x2 < 0 then
    phi = math.atan(math.sqrt(4 * x1 * x1 * x1 - x2 * x2) / x2) + math.pi
  end
  local base = a + b + c
  local lambdas = {
    (base - 2 * math.sqrt(x1) * math.cos(phi / 3)) / 3,
    (base + 2 * math.sqrt(x1) * math.cos((phi - math.pi) / 3)) / 3,
    (base + 2 * math.sqrt(x1) * math.cos((phi + math.pi) / 3)) / 3,
  }
  local eigvecs = {}
  for i=1,3 do
    local m = (d * (c - lambdas[i]) - e * f) / (f * (b - lambdas[i]) - d * e)
    eigvecs[i] = {(lambdas[i] - c - e * m) / f, m, 1}
  end
  return lambdas, eigvecs
end

-- Partial algorithms for large matrices
-- finds top eigenpair using the power method
-- only accepts a single matrix
function Linalg.eigTop(mat, iterLim)
  iterLim = iterLim or 50
  local ndim = #mat.shape
  assert(ndim >= 2 and mat.shape[ndim] == mat.shape[ndim-1])
  local n = mat.shape[ndim]
  local v = {}
  for i=1,n do
    v[i] = math.random()
  end
  v = NdArray.Array(v)
  for i=1, iterLim do
    v = v / Linalg.norm(v)
    v = NdArray.MatMul(mat, v)
  end
  return Linalg.norm(v), v
end

-- finds top k eigenspace using subspace iteration
function Linalg.eigTopK(mat, k)
  -- todo
  return nil
end
