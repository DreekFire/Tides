function StringUtil.LogVector(I, vec, label)
  I:Log(label.."("..vec.x..", "..vec.y..", "..vec.z..")")
end

--[[
does not work because the Lua interpreter used by From The Depths
does not support table.concat

function StringUtil.LogTable(I, tab, label, depth, indent)
  depth = depth or 0
  indent = indent or string.rep("  ", depth)
  local lines = {}
  for k, v in pairs(tab) do
    if type(v) == "table" then
      lines.insert(k..": "..StringUtil.LogTable(I, v, "", depth + 1, indent))
    else
      lines.insert(k..": "..v)
    end
  end
  return label.."{\n"..indent..table.concat(lines, "\n"..indent).."\n}"
end
]]