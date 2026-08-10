local function close(actual, expected, message)
    assert(math.abs(actual - expected) < 1e-8, message or (actual .. " ~= " .. expected))
end

local function assertArrayClose(actual, expected, message)
    assert(#actual.shape == #expected.shape, message or "rank mismatch")
    for axis = 1, #actual.shape do assert(actual.shape[axis] == expected.shape[axis], message or "shape mismatch") end
    for i = 1, actual.size do close(actual.data[i], expected.data[i], message) end
end

local matrix = NdArray.array({{12, -51, 4}, {6, 167, -68}, {-4, 24, -41}})
local q, r = Linalg.qr(matrix)
assertArrayClose(NdArray.matMul(q, r), matrix, "QR reconstruction")
assertArrayClose(NdArray.matMul(NdArray.transpose(q), q), NdArray.eye(3), "Q orthogonality")

local luInput = NdArray.array({{0, 2, 1}, {1, 1, 0}, {2, 0, 1}})
local p, l, u = Linalg.lu(luInput)
assertArrayClose(NdArray.matMul(p, luInput), NdArray.matMul(l, u), "LU reconstruction")
local rhs = NdArray.array({3, 2, 3})
assertArrayClose(Linalg.solveLU(p, l, u, rhs), NdArray.array({1, 1, 1}), "LU solve")

local cholInput = NdArray.array({{4, 12, -16}, {12, 37, -43}, {-16, -43, 98}})
local chol = Linalg.cholesky(cholInput)
assertArrayClose(NdArray.matMul(chol, NdArray.transpose(chol)), cholInput, "Cholesky reconstruction")

local qrQ, qrR = Linalg.qr(NdArray.array({{3, 1}, {1, 3}}))
assertArrayClose(Linalg.solveQR(qrQ, qrR, NdArray.array({4, 4})), NdArray.array({1, 1}), "QR solve")
assertArrayClose(Linalg.solveQR(qrQ, qrR, NdArray.array({{4, 8}, {4, 8}})),
    NdArray.array({{1, 2}, {1, 2}}), "QR multiple right hand sides")

local values, vectors, info = Linalg.eigQr(NdArray.array({{2, 1}, {1, 2}}), 100)
assert(info.converged, "symmetric QR did not converge")
close(values[1], 3, "largest eigenvalue")
close(values[2], 1, "smallest eigenvalue")
local diagonal = NdArray.zeros({2, 2})
diagonal.data[1], diagonal.data[4] = values[1], values[2]
assertArrayClose(NdArray.matMul(NdArray.matMul(vectors, diagonal), NdArray.transpose(vectors)),
    NdArray.array({{2, 1}, {1, 2}}), "eigendecomposition reconstruction")

local topValue, topVector = Linalg.eigTop(NdArray.array({{2, 1}, {1, 2}}), 50)
close(topValue, 3, "power iteration")
close(math.abs(topVector.data[1]), math.abs(topVector.data[2]), "power eigenvector")

local rectangular = NdArray.array({{1, 0}, {0, 1}, {1, 1}})
local rectangularQ, rectangularR = Linalg.qr(rectangular)
assertArrayClose(NdArray.matMul(rectangularQ, rectangularR), rectangular, "rectangular QR reconstruction")
local leastSquares = Linalg.solveLeastSquares(rectangular, NdArray.array({1, 2, 3}))
assertArrayClose(leastSquares, NdArray.array({1, 2}), "rectangular least-squares solve")
assertArrayClose(Linalg.solveLeastSquares(rectangular, NdArray.array({{1, 2}, {2, 4}, {3, 6}})),
    NdArray.array({{1, 2}, {2, 4}}), "rectangular multiple right hand sides")

local rhsMatrix = NdArray.array({{3, 1}, {2, 0}, {3, 1}})
local luSolutions = Linalg.solveLU(p, l, u, rhsMatrix)
assertArrayClose(NdArray.matMul(luInput, luSolutions), rhsMatrix, "LU multiple right hand sides")
local cholRhs = NdArray.array({{4, 8}, {12, 24}, {-16, -32}})
local cholSolutions = Linalg.solveCholesky(chol, cholRhs)
assertArrayClose(NdArray.matMul(cholInput, cholSolutions), cholRhs, "Cholesky solve")

close(Linalg.det(NdArray.array({{1, 2}, {3, 4}})), -2, "determinant")
assertArrayClose(NdArray.matMul(NdArray.array({{1, 2}, {3, 4}}), Linalg.inverse(NdArray.array({{1, 2}, {3, 4}}))),
    NdArray.eye(2), "inverse")
local normalized, length = Linalg.normalize(NdArray.array({3, 4}))
close(length, 5, "vector norm")
assertArrayClose(normalized, NdArray.array({0.6, 0.8}), "vector normalization")
assertArrayClose(Linalg.cross(NdArray.array({1, 0, 0}), NdArray.array({0, 1, 0})), NdArray.array({0, 0, 1}), "cross product")
assertArrayClose(Linalg.outer(NdArray.array({1, 2}), NdArray.array({3, 4, 5})),
    NdArray.array({{3, 4, 5}, {6, 8, 10}}), "outer product")
assertArrayClose(Linalg.skew(NdArray.array({1, 2, 3})),
    NdArray.array({{0, -3, 2}, {3, 0, -1}, {-2, 1, 0}}), "skew matrix")

local transform = NdArray.array({{0, -1, 0, 2}, {1, 0, 0, 3}, {0, 0, 1, 4}, {0, 0, 0, 1}})
assertArrayClose(Linalg.transformPoints(transform, NdArray.array({1, 0, 2})), NdArray.array({2, 4, 6}), "point transform")
assertArrayClose(Linalg.composeTransform(transform, Linalg.rigidInverse(transform)), NdArray.eye(4), "rigid transform inverse")
local points = Linalg.transformPoints(transform, NdArray.array({{1, 0, 2}, {0, 1, 0}}))
assertArrayClose(points, NdArray.array({{2, 4, 6}, {1, 3, 4}}), "batched point transform")
local damped = Linalg.dampedLeastSquares(NdArray.array({{1, 0}, {0, 1}, {1, 1}}), 1e-8)
assert(damped.shape[1] == 2 and damped.shape[2] == 3, "damped least-squares shape")

print("linalg ndarray tests passed")
