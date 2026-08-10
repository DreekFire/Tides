local function array(shape, data)
    local stride, step = {}, 1
    for axis = #shape, 1, -1 do
        stride[axis] = step
        step = step * shape[axis]
    end
    return {shape = shape, stride = stride, data = data, offset = 0, size = step}
end

local function assertEqual(actual, expected, message)
    assert(actual == expected, (message or "values differ") .. ": " .. tostring(actual) .. " ~= " .. tostring(expected))
end

local rows = array({2, 1}, {10, 20})
local columns = array({3}, {1, 2, 3})
local broadcasted = {}
for _, values in NdArray.nditer({rows, columns}) do
    broadcasted[#broadcasted + 1] = values[1] + values[2]
end
assertEqual(table.concat(broadcasted, ","), "11,12,13,21,22,23", "scalar broadcasting")

local scalarIterator = NdArray.nditer(rows)
local firstIndices, firstValues = scalarIterator()
local secondIndices, secondValues = scalarIterator()
assert(firstIndices == secondIndices and firstValues == secondValues,
    "scalar iteration should reuse index and value tables")

local left = array({2}, {1, 2})
local right = array({3}, {10, 20, 30})
local outer = {}
for indices, values in NdArray.nditer({left, right}, nil, "C", {{0, -1}, {-1, 0}}) do
    outer[#outer + 1] = indices[1] .. ":" .. indices[2] .. "=" .. (values[1] * values[2])
end
assertEqual(table.concat(outer, ","), "0:0=10,0:1=20,0:2=30,1:0=20,1:1=40,1:2=60", "op_axes outer product")

local matrix = array({2, 3}, {1, 2, 3, 4, 5, 6})
local coreOffsets, coreViews, firstCoreView = {}, nil, nil
for indices, views in NdArray.coreIter(matrix, {{1}}) do
    coreOffsets[#coreOffsets + 1] = views[1].offset
    coreViews = views
    if firstCoreView then assertEqual(firstCoreView, views[1], "core view table is reused") else firstCoreView = views[1] end
    assertEqual(#views[1].shape, 1, "core rank")
    assertEqual(views[1].shape[1], 3, "core size")
    assertEqual(views[1].data[views[1].offset + 1], indices[1] * 3 + 1, "core view offset")
end
assertEqual(table.concat(coreOffsets, ","), "0,3", "core view traversal")

local sumRows = NdArray.gufunc(function(view)
    local total = 0
    for i = 0, view.shape[1] - 1 do total = total + view.data[view.offset + i * view.stride[1] + 1] end
    return total
end, "(n)->()")
local sums = sumRows(matrix)
assertEqual(table.concat(sums.data, ","), "6,15", "gufunc scalar output")
assertEqual(table.concat(sums.shape, ","), "2", "gufunc loop output shape")

local columnSums = NdArray.gufunc(function(view)
    local total = 0
    for i = 0, view.shape[1] - 1 do total = total + view.data[view.offset + i * view.stride[1] + 1] end
    return total
end, "(n)->()", {{0}})
local sumsByColumn = columnSums(matrix)
assertEqual(table.concat(sumsByColumn.data, ","), "5,7,9", "gufunc positional core axes")

local copyRows = NdArray.gufunc(function(view) return view end, "(n)->(n)")
local copied = copyRows(matrix)
assertEqual(table.concat(copied.data, ","), "1,2,3,4,5,6", "gufunc core output")

local reshapeSource = array({2, 3, 4}, {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
    13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
})
local reshapedView = NdArray.reshape(NdArray.transpose(reshapeSource, 0, 1), {3, 1, 2, 1, 4})
assert(reshapedView.data == reshapeSource.data, "reshape should retain compatible non-contiguous views")
assertEqual(table.concat(reshapedView.stride, ","), "4,0,12,0,1", "non-contiguous reshape strides")

local reshapedCopy = NdArray.reshape(NdArray.transpose(matrix), {6})
assert(reshapedCopy.data ~= matrix.data, "incompatible reshape should copy")
assertEqual(table.concat(reshapedCopy.data, ","), "1,4,2,5,3,6", "incompatible reshape copy")

local broadcastOk, broadcastError = pcall(function()
    NdArray.broadcastArrays({array({2}), array({3})})
end)
assert(not broadcastOk, "incompatible broadcast should fail")
assert(broadcastError:match("%(2%)") and broadcastError:match("%(3%)"),
    "broadcast error should include incompatible shapes: " .. tostring(broadcastError))

local transposedCopy = NdArray.gufunc(function(view) return view end, "(n)->(n)", {{0}, {0}})
local transposed = transposedCopy(array({3, 2}, {1, 2, 3, 4, 5, 6}))
assertEqual(table.concat(transposed.shape, ","), "3,2", "custom output core axis")
assertEqual(table.concat(transposed.data, ","), "1,2,3,4,5,6", "custom output placement")

local ok = pcall(function()
    NdArray.gufunc(function(a, b) return a.data[a.offset + 1] + b.data[b.offset + 1] end, "(n),(n)->()")(
        array({2, 3}, {1, 2, 3, 4, 5, 6}), array({2, 1}, {10, 20}))
end)
assert(not ok, "gufunc core dimensions must not broadcast")
