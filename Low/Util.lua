function Util.shallowCopyWithMeta(tab)
    local new = Util.ShallowCopy(tab)
    new.metatable = tab.metatable
    return new
end

function Util.shallowCopy(tab)
    return {tab.unpack()}
end

function Util.defaultdict(factory, nested)
    local meta = {}
    meta.__index = function(t, k)
        local v = factory()
        if nested then
            setmetatable(v, meta)
        end
        rawset(t, k, v)
        return v
    end
    return setmetatable(factory(), meta)
end