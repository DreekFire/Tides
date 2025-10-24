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
