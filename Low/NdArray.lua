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

local function ndBroadcastError(shapes)
    local formatted = {}
    for index, shape in ipairs(shapes) do formatted[index] = ndShapeString(shape) end
    return string.format("operands could not be broadcast together with shapes %s",
        table.concat(formatted, " and "))
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
        if values[1] ~= 0 then
            count = count + 1
            for axis = 1, ndim do coordinates[axis][count] = idxs[axis] end
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
    local ndim, count, position = #arr.shape, 0, 0
    local data = {}
    for idx, values in NdArray.nditer(arr, nil, "C") do
        if values[1] ~= 0 then
            count = count + 1
            for axis = 1, ndim do
                position = position + 1
                data[position] = idx[axis]
            end
        end
    end
    return NdArray.reshape(NdArray.array(data), {count, ndim})
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
NdArray.equal = function(a, b) return NdArray._elementwise(function(x, y) return x == y end, a, b) end
NdArray.notEqual = function(a, b) return NdArray._elementwise(function(x, y) return x ~= y end, a, b) end
NdArray.less = function(a, b) return NdArray._elementwise(function(x, y) return x < y end, a, b) end
NdArray.lessEqual = function(a, b) return NdArray._elementwise(function(x, y) return x <= y end, a, b) end
NdArray.greater = function(a, b) return NdArray._elementwise(function(x, y) return x > y end, a, b) end
NdArray.greaterEqual = function(a, b) return NdArray._elementwise(function(x, y) return x >= y end, a, b) end

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
