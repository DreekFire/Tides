function StringUtil.LogVector(I, vec, label)
  I:Log(label.."("..vec.x..", "..vec.y..", "..vec.z..")")
end

function StringUtil.Join(tab, delimiter, start, stop)
  start = start or 1
  stop = stop or #tab
  delimiter = delimiter or ','
  if stop > 8 then
    local l = StringUtil.Join(tab, delimiter, start, math.floor(stop * 0.5))
    local r = StringUtil.Join(tab, delimiter, math.floor(stop * 0.5) + 1, stop)
    return l..r
  end
  local str = ''
  for i, s in ipairs(tab) do
    str = str..(s..',')
  end
  return str
end