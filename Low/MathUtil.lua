local MathUtil = {}

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
    set - an iterator or table
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
  local A = Mathf.Acos((b * b + c * c - a * a) / (2 * b * c)) * Mathf.Rad2Deg
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
  local csq = a * a + b * b - 2 * a * b * Mathf.Cos(C * Mathf.Deg2Rad)
  return Mathf.Sqrt(csq)
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
              one or both may be nil
--]]
function MathUtil.sideSSA(a, b, A)
  local q0 = b * b - a * a
  local q1 = -2 * b * Mathf.Cos(A * Mathf.Deg2Rad)
  local c1, c2 = MathUtil.solveQuadratic(q0, q1, 1)
  if c1 < c2 then return c1, c2 end
  return c2, c1
end

--[[
  Arguments:
    a - the length of a side
    b - the length of another side
    A - the angle opposite side a
  Returns:
    B1, C1 - the first possibility for the second and third side
    B2, C2 - the second possibility for the second and third side
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
  return a * Mathf.Sin(B * Mathf.Deg2Rad) / Mathf.Sin(A * Mathf.Deg2Rad)
end

--[[
  Arguments:
    a - one side in the triangle
    b - another side in the triangle
    A - the angle opposite side a
  Returns:
    B - the angle opposite side b
--]]
function MathUtil.angleLoSin(a, b, A)
  return Mathf.Asin(b * Mathf.Sin(A * Mathf.Deg2Rad) / a) * Mathf.Rad2Deg
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
  local offAngle = Mathf.Min(angle, Vector3.Angle(v1, v2))
  local axis = Vector3.Cross(v1, v2)
  return Quaternion.AngleAxis(offAngle, axis) * v1
end

-- code from lua-polynomials by piqey (John Kushmer) on github
-- https://github.com/piqey/lua-polynomials
-- GNU General Public License, version 3

local eps = 1e-9

-- checks if d is close enough to 0 to be considered 0 (for our purposes)
function MathUtil.isZero(d)
  return (d > -eps and d < eps)
end

-- fixes an issue with math.pow that returns nan when the result should be a real number
function MathUtil.cuberoot(x)
  return (x > 0) and (x ^ (1 / 3)) or -(math.abs(x) ^ (1 / 3))
end

function MathUtil.solveQuadratic(c0, c1, c2)
  local s0, s1

  local p, q, D

  -- x^2 + px + q = 0
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

  -- substitute x = y - A/3 to eliminate quadric term: x^3 + px + q = 0
  sq_A = A * A
  p = (1 / 3) * (-(1 / 3) * sq_A + B)
  q = 0.5 * ((2 / 27) * A * sq_A - (1 / 3) * A * B + C)

  -- use Cardano's formula
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
      s1 = -u
      num = 2
      --return s0, s1
    end
  elseif (D < 0) then -- Casus irreducibilis: three real solutions
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
  local num

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