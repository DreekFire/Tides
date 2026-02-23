NdArray = {}

function NdArray.array(lst)
    local stack = {}

end

function NdArray.zeros(shape)
    return NdArray.full(shape, 0)
end

function NdArray.ones(shape)
    return NdArray.full(shape, 1)
end

function NdArray.full(shape, fill)
    local arr = NdArray.Empty(shape)
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
    local n = math.floor((stop - start) / step)
    local ret = NdArray.empty({n})
    ret.data = {}
    for i=0,n-1 do
        ret.data[i+1] = start + i * step
    end
    return ret
end

function NdArray._callbackIter(func, dst, ...)

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

function NdArray.nditer(arrs, opFlags, order, opAxes, iterShape)
    local shape
    local strides = {}
    local baseOffsets = {}
    local iter
    if not opAxes then
        if not iterShape then
            arrs = NdArray.broadcastArrays(arrs)
            shape = arrs[1].shape
        else
            local broadcasted = {}
            for i, arr in ipairs(arrs) do
                broadcasted[i] = NdArray.broadcastTo(arr, iterShape)
            end
            arrs = broadcasted
            shape = iterShape
        end
        iter = NdArray._strideIter(shape, strides, baseOffsets)
    else
        local coreShapes = {}
        local coreStrides = {}
        local arrShapes = {}
        for i, arr in ipairs(arrs) do
            if not opAxes[i] then
                strides[i] = arr.stride
                arrShapes[i] = arr.shape
            else
                local stride = {}
                local arrShape = {}
                local coreShape = {}
                local coreStride = {}
                local coreNdim = 0
                local isIter = {}
                for j, ax in ipairs(opAxes[i]) do
                    stride[j] = arr.stride[ax]
                    arrShape[j] = arr.shape[ax]
                    isIter[ax] = true
                end
                for ax, n in ipairs(arrs[i].shape) do
                    if not isIter[ax] then
                        coreNdim = coreNdim + 1
                        coreShape[coreNdim] = n
                        coreStride[coreNdim] = arr.stride[ax]
                    end
                end
                strides[i] = stride
                arrShapes[i] = arrShape
                coreShapes[i] = coreShape
            end
            baseOffsets[i] = arr.offset or 1
        end
        if not iterShape then
            shape = NdArray.broadcastShapes(arrShapes)
        else
            shape = iterShape
        end
        local broadcastedStrides = {}
        for i, arr in ipairs(arrs) do
            broadcastedStrides[i] = NdArray.broadcastStride(arrShapes[i], shape, strides[i])
        end
        iter = NdArray._strideIter(shape, broadcastedStrides, baseOffsets)
    end

    local vals = {}
    return function()
        local idxs, offsets = iter()
        if idxs and offsets then
            for i, arr in ipairs(arrs) do
                vals[i] = arr.data[offsets[i]]
            end
            return idxs, vals
        end
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
    local shape = arr.shape
    local ndim = #shape
    local data = {}
    local k = 1
    for _, x in NdArray.nditer(arr) do
        data[k] = x
        k = k + 1
    end
    local stride = {}
    stride[ndim] = 1
    for d=ndim-1,1,-1 do
        stride[d] = stride[d+1] * shape[d+1]
    end
    local ret = Util.shallowCopyWithMeta(arr)
    ret.data = data
    ret.stride = stride
    return ret
end

function NdArray.copyTo(dst, src, where)
    if where == false then return end
    if where == true then
        dst, src = NdArray.broadcastArrays(dst, src)
        for _, dstOfs, srcOfs in NdArray._strideIter(dst.shape, dst.stride, src.stride) do
            dst.data[dstOfs] = src.data[srcOfs]
        end
    else
        dst, src, where = NdArray.broadcastArrays(dst, src, where)
        for _, dstOfs, srcOfs, whereOfs in NdArray._strideIter(dst.shape, dst.stride, src.stride, where.stride) do
            if where.data[whereOfs] then
                dst.data[dstOfs] = src.data[srcOfs]
            end
        end
    end
end

function NdArray._reshapeIsView(shape1, shape2, stride1)
    local k = 1
    local compressedShape = {}
    local compressedStride = {}
    local ndim1 = #shape1
    local currentShape = shape1[1]
    for i=1,ndim1-1 do
        if stride1[i] == shape1[i + 1] * stride1[i + 1] or shape1[i + 1] == 1 then
            currentShape = currentShape * shape1[i + 1]
        else
            compressedShape[k] = currentShape
            compressedStride[k] = stride1[k]
            k = k + 1
        end
    end

    local p = 1
    local ndim2 = #shape2
    local j = #compressedShape
    local outputStride = {}
    for i=ndim2,2,-1 do
        if p == 1 then
            outputStride[i] = compressedStride[j]
        else
            outputStride[i] = outputStride[i + 1] * shape2[i]
        end
        p = p * shape2[i]
        if p == compressedShape[j] then
            p = 1
            j = j - 1
        elseif p > compressedShape[j] then
            return false, nil
        end
    end
    if p == 1 then
        outputStride[1] = compressedStride[j]
    else
        outputStride[1] = outputStride[2] * shape2[1]
    end
    return true, outputStride
end

function NdArray.reshape(arr, ...)
    local shape = {...}
    local p1 = NdArray.prod(arr.shape)
    local p2 = NdArray.prod(shape)
    if p1 ~= p2 then
        local flex = false
        for i, d in ipairs(shape) do
            assert(not flex, string.format("Target shape can only have one unspecified (-1) dimension"))
            if d == -1 then
                flex = true
                shape[i] = math.floor(p1 / -p2)
                assert(p2 * shape[i] == p1, string.format("Shapes are not compatible: (%s) and (%s)", StringUtil.Join(arr.shape), StringUtil.Join(shape)))
            end
        end
        assert(flex, string.format("Reshape is not possible: array 1 has %s elements while array 2 has %s", p1, p2))
    end
    local valid, stride = NdArray._reshapeIsView(arr.shape, shape, arr.stride)
    local ret
    if valid then
        ret = Util.shallowCopyWithMeta(arr)
    else
        ret = NdArray.copy(arr)
    end
    ret.stride = stride
    ret.shape = shape
    return ret
end

function NdArray.transpose(arr, axis1, axis2)
    local res = Util.shallowCopyWithMeta(arr)
    local ndim = #arr.shape
    if axis1 == nil then
        for d, s in ipairs(arr.shape) do
            local m = ndim - d + 1
            res.shape[d] = arr.shape[m]
            res.stride[d] = arr.stride[m]
        end
    else
        if axis1 < 0 then
            axis1 = ndim + axis1 + 1
        end
        if axis2 < 0 then
            axis2 = ndim + axis2 + 1
        end
        res.shape[axis1] = arr.shape[axis2]
        res.shape[axis2] = arr.shape[axis1]
        res.stride[axis1] = arr.stride[axis2]
        res.stride[axis2] = arr.stride[axis1]
    end
    return res
end

function NdArray.permuteDims(arr, dims)
    local res = Util.shallowCopyWithMeta(arr)
    local ndim = #arr.shape
    if dims == nil then
        for i=1,ndim do
            dims[i] = ndim - i + 1
        end
    end
    for i=1,ndim do
        res.shape[i] = arr.shape[dims[i]]
        res.stride[i] = arr.stride[dims[i]]
    end
    return res
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
    local broadcasted = {NdArray.broadcastArrays(...)}
    local n = #broadcasted
    local ret = NdArray.empty(broadcasted[1].shape)
    local args = {}
    for idx, v in NdArray.nditer(broadcasted[1]) do
        for i=1,n do
            args[i] = NdArray._getSingle(broadcasted[i], idx)
        end
        NdArray._setSingle(ret, idx, func(table.unpack(args)))
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
    if #arr1.shape == 1 and #arr2.shape == 1 then
        return NdArray.sum(NdArray.mul(arr1, arr2))
    end
    if #arr2.shape == 1 then
        assert(
            arr2.shape[1] == arr1.shape[#arr1.shape],
            string.format("Last dimension not compatible for matrix-vector multiplication: (%s) and (%s)",
                arr1.shape[#arr1.shape],
                arr2.shape[1]
            )
        )
        arr2 = NdArray.reshape(arr2, arr2.size, 1)
    end
    local n = arr1.shape[#arr1.shape - 1]
    local k = arr1.shape[#arr1.shape]
    local m = arr2.shape[#arr2.shape]
    assert(
        arr2.shape[#arr2.shape - 1] == k,
        string.format("Last 2 dimensions not compatible for matrix multiplication: (%s, %s) and (%s, %s)",
            n, k,
            arr2.shape[#arr2.shape - 1], m
        )
    )
    local batchShape1 = {}
    local batchShape2 = {}
    for i=1,#arr1.shape - 2 do
        batchShape1[i] = arr1.shape[i]
    end
    for i=1,#arr2.shape - 2 do
        batchShape2[i] = arr2.shape[i]
    end
    local batchShapeBroadcasted = NdArray.broadcastShapes(batchShape1, batchShape2)
    local shape1Broadcasted = Utils.shallowCopy(batchShape1)
    local shape2Broadcasted = Utils.shallowCopy(batchShape2)
    local ndim = #batchShapeBroadcasted + 2
    shape1Broadcasted[ndim - 1] = n
    shape1Broadcasted[ndim] = k
    shape2Broadcasted[ndim - 1] = k
    shape2Broadcasted[ndim] = m
    local broad1 = NdArray.broadcastTo(arr1, shape1Broadcasted)
    local broad2 = NdArray.broadcastTo(arr2, shape2Broadcasted)
    
end

function NdArray.div(arr1, arr2)
    return NdArray._elementwise(function(a, b) return a / b end, arr1, arr2)
end

function NdArray.map(func, arr)
    local res = NdArray.emptyLike(arr)
    for _, arrOfs, resOfs in NdArray._strideIter2(arr, res) do
        res.data[resOfs] = func(arr.data[arrOfs])
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
    local res
    if axis == nil then
        for _, v in NdArray.nditer(arr) do
            res = func(res, v)
        end
        return res
    else
        if axis < 0 then
            axis = #arr.shape + axis
        end
        local n = arr.shape[axis + 1]
        local outShape = {}
        for d, s in ipairs(arr.shape) do
            outShape[d] = s
        end
        outShape[axis + 1] = 1
        res = NdArray.empty(outShape)
        res.shape[axis + 1] = n
        res.stride[axis + 1] = 0
        for _, arrOfs, resOfs in NdArray._strideIter2(arr, res) do
            res.data[resOfs] = func(res.data[resOfs], arr.data[arrOfs])
        end
        res.shape[axis + 1] = 1
        res.stride[axis + 1] = 1
        if not keepdim then
            for d=axis, #res.shape do
                -- intentionally allow overrun to set last element to nil
                res.shape[d] = res.shape[d + 1]
                res.stride[d] = res.stride[d + 1]
            end
        end
    end
    return res
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
    if axis then
        res = res / arr.shape[axis]
    else
        res = res / arr.size
    end
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
    local ndim = #arrShape
    local stride = {}
    local padDims = #targetShape - ndim
    for j=1, padDims do
        stride[j] = 1
    end
    for j=1, ndim do
        -- check shape
        assert(
            targetShape[j + padDims] == arrShape[j] or arrShape[j] == 1,
            string.format("shapes could not be broadcast together: (%s) and (%s)",
                StringUtil.Join(targetShape),
                StringUtil.Join(arrShape))
        )
        stride[j + padDims] = arrShape[j] > 1 and arrStride[j] or 0
    end
    return stride
end

function NdArray.broadcastTo(arr, shape)
    local view = Util.shallowCopyWithMeta(arr)
    local stride = NdArray.broadcastStride(arr.shape, shape, arr.stride)
    view.shape = shape
    view.stride = stride
    return view
end

function NdArray.broadcastArrays(arrs)
    local views = {}
    local new_shape = {}
    local maxNdim = 0
    for i, arr in ipairs(arrs) do
        views[i] = Util.shallowCopyWithMeta(arr)
        local ndim = #arr.shape
        maxNdim = math.max(maxNdim, ndim)
    end
    local shapes = {}
    for i, arr in ipairs(arrs) do
        shapes[i] = arr.shape
    end
    new_shape = NdArray.broadcastShapes(shapes)
    for i, arr in ipairs(arrs) do
        views[i] = NdArray.broadcastTo(arr, new_shape)
    end
    return views
end

function NdArray.broadcastShapes(shapes)
    local ndims = {}
    local newShape = {}
    local maxNdim = 0
    for i, shape in ipairs(shapes) do
        local ndim = #shape
        ndims[i] = ndim
        maxNdim = math.max(maxNdim, ndim)
    end
    for i, shape in ipairs(shapes) do
        local ndim = ndims[i]
        local padDims = maxNdim - ndim
        for j=1, ndim do
            if newShape[j] == 1 then
                newShape[j] = shape[j + padDims]
            end
            -- check shape
            assert(
                newShape[j + padDims] == shape[j] or shape[j] == 1,
                string.format("shapes could not be broadcast together: (%s) and (%s)",
                    StringUtil.Join(newShape),
                    StringUtil.Join(shape))
            )
        end
    end
    return newShape
end

function NdArray.convertAndBroadcast(...)
    local arrs = {}
    for i, arr in ipairs(arg) do
        if type(arr) ~= "table" or getmetatable(arr) ~= NdArray.meta then
            arrs[i] = NdArray.Array(arr)
        end
    end
    return NdArray.broadcastArrays(arrs.unpack())
end

function NdArray.nonzero(arr)
    local ndim = #arr.shape
    local nonzeroData = {}
    local k = 0
    for idxs, x in NdArray.nditer(arr) do
        if x and x ~= 0 then
            for d=1,ndim do
                nonzeroData[k * ndim + d] = idxs[d]
            end
            k = k + 1
        end
    end
    local nonzero = {}
    for d=1,ndim do
        nonzero[d] = NdArray.empty({k})
        nonzero[d].data = nonzeroData
        nonzero[d].offset = k * (d - 1) + 1
    end
    return nonzero
end

function NdArray.countNonzero(arr)
    local k = 1
    for _, x in NdArray.nditer(arr) do
        if x and x ~= 0 then
            k = k + 1
        end
    end
    return k
end

function NdArray.argwhere(arr)
    local ndim = #arr.shape
    local argwhere = {}
    local k = 0
    for idx, x in NdArray.nditer(arr) do
        if x and x ~= 0 then
            for d=1,ndim do
                argwhere[k * ndim + d] = idx[d]
            end
            k = k + 1
        end
    end
    argwhere = NdArray.array(argwhere)
    return NdArray.swapaxes(NdArray.reshape(argwhere, k, ndim), 0, 1)
end

function NdArray.where(cond, ifTrue, ifFalse)
    if ifTrue == nil then
        return NdArray.nonzero(cond)
    end
    cond, ifTrue, ifFalse = NdArray.broadcastArrays(cond, ifTrue, ifFalse)
    local ret = NdArray.Empty(cond.shape)
    for _, offsets in NdArray._strideIter(
        cond.shape,
        {ret.stride, cond.stride, ifTrue.stride, ifFalse.stride},
        {ret.offset, cond.offset, ifTrue.offset, ifFalse.offset}
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

--todo:
-- add, sub, mul, div, matmul, comparisons, split, concatenate, stack
-- sum, prod, mean, max, min, argmax, argmin, count_nonzero, nonzero, where, argwhere
-- arange, linspace, meshgrid, mgrid, ogrid, diag, eye, expand_dims, squeeze, repeat, tile,