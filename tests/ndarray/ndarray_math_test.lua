local function close(actual, expected, message)
    assert(math.abs(actual - expected) < 1e-10, message or (actual .. " ~= " .. expected))
end

local a = NdArray.array({{1, 2, 3}, {4, 5, 6}})
local b = NdArray.array({{7, 8}, {9, 10}, {11, 12}})
local product = NdArray.matMul(a, b)
assert(product.shape[1] == 2 and product.shape[2] == 2)
close(product.data[1], 58)
close(product.data[4], 154)

local vector = NdArray.array({1, 2, 3})
assert(NdArray.matMul(vector, vector) == 14)
local row = NdArray.matMul(vector, b)
assert(row.shape[1] == 2 and row.data[1] == 58 and row.data[2] == 64)
local column = NdArray.matMul(a, vector)
assert(column.shape[1] == 2 and column.data[1] == 14 and column.data[2] == 32)

local batchLeft = NdArray.array({{{1, 0}, {0, 1}}, {{2, 0}, {0, 2}}})
local batchRight = NdArray.array({{3, 4}, {5, 6}})
local batchProduct = NdArray.matMul(batchLeft, batchRight)
assert(batchProduct.shape[1] == 2 and batchProduct.shape[2] == 2 and batchProduct.shape[3] == 2)
close(batchProduct.data[1], 3)
close(batchProduct.data[4], 6)
close(batchProduct.data[5], 6)
close(batchProduct.data[8], 12)

local transposed = NdArray.transpose(a)
local transposedProduct = NdArray.matMul(transposed, a)
assert(transposedProduct.shape[1] == 3 and transposedProduct.shape[2] == 3)
close(transposedProduct.data[1], 17)
close(transposedProduct.data[9], 45)

local broadcast = NdArray.add(NdArray.array({{1}, {2}}), NdArray.array({10, 20, 30}))
assert(broadcast.shape[1] == 2 and broadcast.shape[2] == 3)
close(broadcast.data[1], 11)
close(broadcast.data[6], 32)
local reduced = NdArray.sum(NdArray.array({{1, 2, 3}, {4, 5, 6}}), 1)
assert(reduced.shape[1] == 2 and reduced.shape[2] == nil)
close(reduced.data[1], 6)
close(reduced.data[2], 15)
close(NdArray.mean(vector), 2)
local comparison = NdArray.greater(vector, NdArray.array({0, 3, 2}))
assert(comparison.data[1] == true and comparison.data[2] == false and comparison.data[3] == true)

local destination = NdArray.zeros({2, 3})
NdArray._callbackIter(function(x, y) return x + y end, destination,
    NdArray.array({{1}, {2}}), NdArray.array({10, 20, 30}))
close(destination.data[1], 11)
close(destination.data[6], 32)
local copied = NdArray.zeros({2, 3})
NdArray.copyTo(copied, destination)
assert(table.concat(copied.data, ",") == "11,21,31,12,22,32", "unmasked copy traversal")
NdArray.copyTo(copied, NdArray.full({2, 3}, 99), NdArray.array({{true, false, true}, {false, true, false}}))
assert(table.concat(copied.data, ",") == "99,21,99,12,99,32", "masked copy traversal")
local selected = NdArray.where(NdArray.array({{true, false}, {false, true}}),
    NdArray.array({{1, 2}, {3, 4}}), NdArray.zeros({2, 2}))
assert(table.concat(selected.data, ",") == "1,0,0,4", "where traversal")
local sorted = NdArray.sort(NdArray.array({{3, 1, 2}, {6, 4, 5}}), 1)
assert(table.concat(sorted.data, ",") == "1,2,3,4,5,6", "axis sort traversal")
local sortedIndices = NdArray.argsort(NdArray.array({{3, 1, 2}, {6, 4, 5}}), 1)
assert(table.concat(sortedIndices.data, ",") == "1,2,0,1,2,0", "axis argsort traversal")
local stacked = NdArray.stack({NdArray.array({1, 2}), NdArray.array({3, 4})}, 0)
assert(stacked.shape[1] == 2 and stacked.shape[2] == 2 and stacked.data[4] == 4)
local diagonal = NdArray.diag(NdArray.array({1, 2, 3}))
assert(diagonal.shape[1] == 3 and diagonal.data[1] == 1 and diagonal.data[5] == 2 and diagonal.data[9] == 3)
local coordinates = NdArray.argwhere(NdArray.array({{0, 2}, {3, 0}}))
assert(coordinates.shape[1] == 2 and coordinates.shape[2] == 2 and coordinates.data[1] == 0 and coordinates.data[4] == 0)
assert(table.concat(coordinates.data, ",") == "0,1,1,0", "argwhere should preserve C-order coordinates")
local noCoordinates = NdArray.argwhere(NdArray.zeros({2, 2}))
assert(noCoordinates.shape[1] == 0 and noCoordinates.shape[2] == 2 and #noCoordinates.data == 0,
    "argwhere should preserve the coordinate width for empty results")

local indexed = NdArray.array({{10, 20, 30}, {40, 50, 60}})
local rowIndices = NdArray.array({0, 1})
local columnIndices = NdArray.array({2, 0})
local selected = NdArray.get(indexed, {rowIndices, columnIndices})
assert(selected.shape[1] == 2 and table.concat(selected.data, ",") == "30,40",
    "fancy get should use scalar nditer values as source coordinates")
NdArray.set(indexed, {rowIndices, columnIndices}, NdArray.array({300, 400}))
assert(table.concat(indexed.data, ",") == "10,20,300,400,50,60",
    "fancy set should use scalar nditer values as destination coordinates")
local partial = NdArray.array({{1, 2, 3}, {4, 5, 6}})
local selectedRows = NdArray.get(partial, {rowIndices, NdArray.sliceAll})
assert(table.concat(selectedRows.data, ",") == "1,2,3,4,5,6",
    "partial fancy get should share the scalar iterator")
NdArray.set(partial, {rowIndices, NdArray.sliceAll}, NdArray.array({{10, 20, 30}, {40, 50, 60}}))
assert(table.concat(partial.data, ",") == "10,20,30,40,50,60",
    "partial fancy set should share the scalar iterator")

-- Basic indexing is a zero-copy view and follows Python/NumPy slice bounds.
local slicedSource = NdArray.array({{1, 2, 3, 4}, {5, 6, 7, 8}, {9, 10, 11, 12}})
local reversedRow = NdArray.get(slicedSource, {1, NdArray.slice(nil, nil, -1)})
assert(reversedRow.shape[1] == 4 and table.concat(NdArray.copy(reversedRow).data, ",") == "8,7,6,5",
    "negative-step slices should reverse a dimension")
local stepped = NdArray.get(slicedSource, {NdArray.slice(0, nil, 2), NdArray.slice(1, 4, 2)})
assert(stepped.shape[1] == 2 and stepped.shape[2] == 2 and table.concat(NdArray.copy(stepped).data, ",") == "2,4,10,12",
    "positive slices should preserve view strides")
local scalar = NdArray.get(slicedSource, {-1, -2})
assert(#scalar.shape == 0 and NdArray.copy(scalar).data[1] == 11, "negative integer indexing")
local expanded = NdArray.get(slicedSource, {NdArray.ellipsis, NdArray.newaxis})
assert(expanded.shape[1] == 3 and expanded.shape[2] == 4 and expanded.shape[3] == 1,
    "ellipsis and newaxis should preserve positional axes")
local viewSource = NdArray.array({{1, 2, 3}, {4, 5, 6}})
local view = NdArray.get(viewSource, {NdArray.slice(0, 2), NdArray.slice(1, 3)})
NdArray.set(view, {0, 1}, 99)
assert(viewSource.data[3] == 99, "basic indexing should return a writable view")
NdArray.set(slicedSource, {NdArray.slice(0, 3, 2), NdArray.slice(0, 4, 2)}, -1)
assert(table.concat(slicedSource.data, ",") == "-1,2,-1,4,5,6,7,8,-1,10,-1,12",
    "basic set should accept scalar broadcast values")

-- Advanced indices broadcast against one another and always return a copy.
local advancedSource = NdArray.array({{0, 1, 2, 3}, {4, 5, 6, 7}, {8, 9, 10, 11}})
local rows = NdArray.array({{0}, {2}})
local columns = NdArray.array({{1, 3}})
local advanced = NdArray.get(advancedSource, {rows, columns})
assert(advanced.shape[1] == 2 and advanced.shape[2] == 2 and table.concat(advanced.data, ",") == "1,3,9,11",
    "fancy indices should broadcast by shape")
NdArray.set(advancedSource, {rows, columns}, NdArray.array({{20}, {40}}))
assert(table.concat(advancedSource.data, ",") == "0,20,2,20,4,5,6,7,8,40,10,40",
    "fancy assignment should broadcast values")
local mixed = NdArray.get(advancedSource, {NdArray.sliceAll, NdArray.array({3, 1})})
assert(mixed.shape[1] == 3 and mixed.shape[2] == 2
    and table.concat(mixed.data, ",") == "20,20,7,5,40,40",
    "mixed basic and fancy indexing")
local rawFancy = NdArray.get(advancedSource, {{2, 0}, NdArray.sliceAll})
assert(rawFancy.shape[1] == 2 and rawFancy.shape[2] == 4
    and table.concat(rawFancy.data, ",") == "8,40,10,40,0,20,2,20",
    "Lua-table fancy indices should follow the NdArray path")

-- Boolean masks support full-array selection and one-dimensional axis masks.
local booleanSource = NdArray.array({{1, 2, 3}, {4, 5, 6}})
local fullMask = NdArray.array({{true, false, true}, {false, true, false}})
local selectedBoolean = NdArray.get(booleanSource, {fullMask})
assert(selectedBoolean.shape[1] == 3 and table.concat(selectedBoolean.data, ",") == "1,3,5",
    "full boolean masks should return C-order selected values")
local transposedBooleanSource = NdArray.transpose(booleanSource)
local transposedMask = NdArray.transpose(fullMask)
local selectedTransposedBoolean = NdArray.get(transposedBooleanSource, {transposedMask})
assert(table.concat(selectedTransposedBoolean.data, ",") == "1,5,3",
    "boolean masks should work for non-contiguous views")
local rowMask = NdArray.array({true, false})
local selectedRowsBoolean = NdArray.get(booleanSource, {rowMask, NdArray.sliceAll})
assert(selectedRowsBoolean.shape[1] == 1 and selectedRowsBoolean.shape[2] == 3
    and table.concat(selectedRowsBoolean.data, ",") == "1,2,3", "axis boolean masks")
NdArray.set(booleanSource, {fullMask}, 9)
assert(table.concat(booleanSource.data, ",") == "9,2,9,4,9,6", "boolean assignment")

-- Other high-value array paths used by robotics code.
local concat = NdArray.concatenate({NdArray.array({{1, 2}, {3, 4}}), NdArray.array({{5, 6}})}, 0)
assert(concat.shape[1] == 3 and concat.shape[2] == 2 and table.concat(concat.data, ",") == "1,2,3,4,5,6",
    "concatenate along rows")
local concatColumns = NdArray.concatenate({NdArray.array({{1}, {2}}), NdArray.array({{3}, {4}})}, 1)
assert(concatColumns.shape[1] == 2 and concatColumns.shape[2] == 2 and table.concat(concatColumns.data, ",") == "1,3,2,4",
    "concatenate along columns")
local expandedView = NdArray.expandDims(NdArray.array({1, 2, 3}), -1)
assert(expandedView.shape[1] == 3 and expandedView.shape[2] == 1, "expandDims negative axis")
assert(NdArray.squeeze(expandedView).shape[1] == 3, "squeeze singleton axis")
local diagonalOffset = NdArray.diag(NdArray.array({1, 2}), 1)
assert(diagonalOffset.shape[1] == 3 and diagonalOffset.data[2] == 1 and diagonalOffset.data[6] == 2,
    "positive diagonal offset")
local matrixDiagonal = NdArray.diag(NdArray.array({{1, 2, 3}, {4, 5, 6}}), -1)
assert(matrixDiagonal.shape[1] == 1 and matrixDiagonal.data[1] == 4, "matrix diagonal offset")
local nonzeroMask = NdArray.nonzero(fullMask)
assert(nonzeroMask.shape[1] == 3 and NdArray.countNonzero(fullMask) == 3, "boolean nonzero and count")
local keepdimSum = NdArray.sum(NdArray.array({{1, 2, 3}, {4, 5, 6}}), -1, true)
assert(keepdimSum.shape[1] == 2 and keepdimSum.shape[2] == 1 and keepdimSum.data[1] == 6 and keepdimSum.data[2] == 15,
    "negative-axis keepdim reduction")
local argmax = NdArray.argmax(NdArray.array({{1, 7, 3}, {9, 2, 4}}), 0)
assert(argmax.shape[1] == 3 and table.concat(argmax.data, ",") == "1,0,1", "axis argmax")
local axisZeroSort = NdArray.sort(NdArray.array({{3, 2}, {1, 4}}), 0)
assert(table.concat(NdArray.copy(axisZeroSort).data, ",") == "1,2,3,4", "axis-zero sort")
local transposedCopy = NdArray.ravel(NdArray.transpose(NdArray.array({{1, 2}, {3, 4}})))
assert(table.concat(NdArray.copy(transposedCopy).data, ",") == "1,3,2,4", "ravel non-contiguous view")

print("ndarray math tests passed")
