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
  return ret
end

function Matrix3.vecmul(mat, vec)
  local ret = Vector3.zero
  for row=0,2 do
    local val = 0
    for col=1,3 do
      val = val + vec[Matrix3.vecIdx[col]] * mat[row * 3 + col]
    end
    ret[Matrix3.vecIdx[row + 1]] = val
  end
end

function Matrix3.matmul(mat1, mat2)
  local ret = {}
  for i=0,2 do
    for j=0,2 do
      local val = 0
      for k=0,2 do
        val = val + mat1[i * 3 + k + 1] * mat2[k * 3 + j + 1]
      end
      ret[i * 3 + j + 1] = val
    end
  end
end

function Matrix3.mul(a, b)
  if getmetatable(a) ~= Matrix3.meta then
    if a.x then return Matrix3.vecmul(Matrix3.transpose(b), a) end
    return Matrix3.mul(b, a)
  end
  if getmetatable(b) == Matrix3.meta then
    return Matrix3.matmul(a, b)
  end
  if b.x then
    return Matrix3.vecmul(a, b)
  end
  return Matrix3.scalarmul(a, b)
end

function Matrix3.Identity()
  return {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1,
  }
end

function Matrix3.Zero()
  return {
    0, 0, 0,
    0, 0, 0,
    0, 0, 0,
  }
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
  return ret
end

function Matrix3.hadamard(mat1, mat2)
  local ret = {}
  for i=1,9 do
    ret[i] = mat1[i] * mat2[i]
  end
  return ret
end

function Matrix3.transpose(mat)
  local transposed = {}
  for row=0,2 do
    for col=0,2 do
      transposed[col * 3 + row + 1] = mat[row * 3 + col + 1]
    end
  end
  return transposed
end

function Matrix3.determinant(mat)
  local diag1 = 0
  local diag2 = 0
  for d=0,2 do
    for s=0,2 do
      diag1 = diag1 + mat[s * 3 + (s + d) % 3 + 1]
      diag2 = diag2 + mat[s * 3 + (-s + d) % 3 + 1]
    end
  end
  return diag1 - diag2
end

function Matrix3.adjugate(mat)
  local adj = {}
  for row=0,2 do
    for col=0,2 do
      local cofac = 0
      for i=0,1 do
        local diag = 1
        for j=1,2 do
          -- this indexing is painful; I understand why most languages start from 0 now
          diag = diag * mat[(row + j) % 3 * 3 + (col + i + j) % 3 + 1]
        end
        cofac = cofac + diag
      end
      adj[col * 3 + row + 1] = cofac
    end
  end
  return adj
end

function Matrix3.inverse(mat)
  local det = Matrix3.determinant(mat)
  if Stats.isZero(det) then return end
  local adj = Matrix3.cofactors(mat)
  return adj / det
end

Matrix3.vecIdx = {
  'x', 'y', 'z'
}

Matrix3.meta = {
  __add = Matrix3.add,
  __mul = Matrix3.mul,
  __unm = function(m) return Matrix3.scalarmul(m, -1) end,
  __pow = Matrix3.pow,
}
