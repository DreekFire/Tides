--[[
  Arguments:
    I - the variable passed to Update
    name - the name of the weapons to search for
           can be set by pressing shift-N in build mode
    count - the number of weapons to search for
    mode - whether to search for weapons on the main hull
           or on subconstructs:
           0 = hull, 1 = subconstructs, 2 = both
  Returns:
    weapons - a list of tables with elements subIdx and wpnIdx
              subIdx is the ID of the subconstruct the weapon
              is mounted on
              wpnIdx is the index of the weapon

  NOTE: returns a list even if you specify count = 1
]]--
function BlockUtil.getWeaponsByName(I, name, count, mode)
  if DEBUG then I:Log("searching for "..name) end
  local weapons = {}
  count = count or -1
  local c = count
  if not mode or mode == 0 or mode == 2 then
    for i=0, I:GetWeaponCount() - 1 do
      if c == 0 then break end
      if I:GetWeaponBlockInfo(i).CustomName == name then
        table.insert(weapons, {subIdx = nil, wpnIdx = i})
        if DEBUG then I:Log("found weapon "..name.." on hull, type "..I:GetWeaponInfo(i).WeaponType) end
        c = c - 1
      end
    end
  end
  if not mode or mode == 1 or mode == 2 then
    local subcs = I:GetAllSubConstructs()
    for idx=1, #subcs do -- for some reason not an actual table so can't use pairs
      local sub = subcs[idx]
      for i=0, I:GetWeaponCountOnSubConstruct(sub) - 1 do
        if c == 0 then break end
        if I:GetWeaponBlockInfoOnSubConstruct(sub, i).CustomName == name then
          table.insert(weapons, {subIdx = sub, wpnIdx = i})
          if DEBUG then I:Log("found weapon "..name.." on subobj "..sub..", type "..I:GetWeaponInfoOnSubConstruct(sub, i).WeaponType) end
          c = c - 1
        end
      end
    end
  end
  if DEBUG then I:Log("weapon count: "..#weapons) end
  return weapons
end

--[[
  Arguments:
    I - the variable passed to Update
    name - the name of the subconstructs to search for
           can be set by pressing shift-N in build mode
    count - the number of subconstructs to search for
  Returns:
    subobjs - a list of subconstruct IDs where each ID
              corresponds to a subconstruct with a matching name
]]--
function BlockUtil.getSubConstructsByName(I, name, count)
	if DEBUG then I:Log("searching for "..name) end
  local subcs = I:GetAllSubConstructs()
  local subobjs = {}
  count = count or -1
  local c = count
  for idx=1, #subcs do -- for some reason not an actual table so can't use pairs
    local sub = subcs[idx]
    if c == 0 then break end
    if I:GetSubConstructInfo(sub).CustomName == name then
      table.insert(subobjs, sub)
      if DEBUG then I:Log("found subobj "..name) end
      c = c - 1
    end
  end
  if DEBUG then I:Log("subobj count: "..#subobjs) end
  return subobjs
end

--[[
  Arguments:
    I - the variable passed to Update
    type - the type of block to search for
           see help menu in lua block for list
    name - the name of the components to search for
           can be set by pressing shift-N in build mode
    count - the number of blocks to search for
  Returns:
    comps - a list of block indices where each index
              corresponds to a block with a matching name
]]--
function BlockUtil.getBlocksByName(I, name, id, count)
	if DEBUG then I:Log("searching for "..name) end
  local comps = {}
  count = count or -1
  local c = count
  for idx=0, I:Component_GetCount(id) - 1 do
    if c == 0 then break end
    if I:Component_GetBlockInfo(id, idx).CustomName == name then
      table.insert(comps, idx)
      if DEBUG then I:Log("found component "..name) end
      c = c - 1
    end
  end
  if DEBUG then I:Log("component count: "..#comps) end
  return comps
end

--[[
  If you have many names to search for, instead of
  searching for each name individually, you can populate
  a table containing all blocks grouped by their name.

  Unnamed blocks will be placed in the array proprtion of
  the table (i.e. t[1], t[2], ...)

  For example:
  {
    3, 14, 15
    "main": {2, 7, 18}
    "secondary": {1, 4}
  }

  Arguments:
    I - the variable passed to Update
    mode - the same as getWeaponsByName
  Returns:
    weapons - a table with weapons grouped by name. Each weapon
              is in the format {subIdx, wpnIdx}.
              See getWeaponsByName for more
]]--
function BlockUtil.populateWeaponsByName(I, mode)
	if DEBUG then I:Log("populating all weapons, mode "..mode) end
  local weapons = {}
  for idx=0, I:GetWeaponCount() - 1 do
    local name = I:Component_GetBlockInfo(type, idx).CustomName
    if name and name ~= '' then
      weapons[name] = weapons[name] or {}
      table.insert(weapons[name], {subIdx = nil, wpnIdx = idx})
      if DEBUG then I:Log("found weapon "..name.." on hull, type "..I:GetWeaponInfo(idx).WeaponType) end
    else
      table.insert(weapons, {subIdx = nil, wpnIdx = idx})
      if DEBUG then I:Log("found unnamed weapon on hull, type "..I:GetWeaponInfo(idx).WeaponType) end
    end
  end
  if not mode or mode == 1 or mode == 2 then
    local subcs = I:GetAllSubConstructs()
    for idx=1, #subcs do
      local sub = subcs[idx]
      for i=0, I:GetWeaponCountOnSubConstruct(sub) - 1 do
        local name = I:Component_GetBlockInfo(type, i).CustomName
        if name and name ~= '' then
          weapons[name] = weapons[name] or {}
          table.insert(weapons[name], {subIdx = sub, wpnIdx = i})
          if DEBUG then I:Log("found weapon "..name.." on subobj "..sub..", type "..I:GetWeaponInfoOnSubConstruct(sub, i).WeaponType) end
        else
          table.insert(weapons, {subIdx = sub, wpnIdx = i})
          if DEBUG then I:Log("found unnamed weapon on subobj "..sub..", type "..I:GetWeaponInfoOnSubConstruct(sub, i).WeaponType) end
        end
      end
    end
  end
  if DEBUG then
    local count = 0
    for k, v in pairs(weapons) do
      if type(v) == "table" then
        count = count + #v
      else
        count = count + 1 end
    end
    I:Log("weapon count: "..count)
  end
  return weapons
end

--[[
  Arguments:
    I - the variable passed to Update
  Returns:
    subobjs - a table with names as keys and a list of subconstructs
              with that name as the value. See populateWeaponsByName
]]--
function BlockUtil.populateSubConstructsByName(I)
	if DEBUG then I:Log("populating all subconstructs") end
  local subcs = I:GetAllSubConstructs()
  local subobjs = {}
  for idx=1, #subcs do -- for some reason not an actual table so can't use pairs
    local sub = subcs[idx]
    local name = I:GetSubConstructInfo(sub).CustomName
    if name and name ~= '' then
      subobjs[name] = subobjs[name] or {}
      table.insert(subobjs[name], sub)
      if DEBUG then I:Log("found subobj "..name) end
    else
      table.insert(subobjs, sub)
      if DEBUG then I:Log("found unnamed subobj") end
    end
  end
  if DEBUG then
    local count = 0
    for k, v in pairs(subobjs) do
      if type(v) == "table" then
        count = count + #v
      else
        count = count + 1 end
    end
    I:Log("subobject count: "..count)
  end
  return subobjs
end

--[[
  Arguments:
    I - the variable passed to Update
    id - the type of block to search for
         see help menu in lua block for list
  Returns:
    comps - a table with names as keys and a list of indices
            corresponding to blocks with that name as the value
            See populateWeaponsByName
]]--
function BlockUtil.populateBlocksByName(I, id)
	if DEBUG then I:Log("populating all blocks of type "..id) end
  local comps = {}
  for idx=0, I:Component_GetCount(id) - 1 do
    local name = I:Component_GetBlockInfo(id, idx).CustomName
    if name and name ~= '' then
      comps[name] = comps[name] or {}
      table.insert(comps[name], idx)
      if DEBUG then I:Log("found component "..name) end
    else
      table.insert(comps, idx)
      if DEBUG then I:Log("found unnamed component of type "..id) end
    end
  end
  if DEBUG then
    local count = 0
    for k, v in pairs(comps) do
      if type(v) == "table" then
        count = count + #v
      else
        count = count + 1 end
    end
    I:Log("component count: "..count)
  end
  return comps
end

--[[
  FtD's lua API has different functions depending on whether the weapon is on
  a subconstruct or not, resulting in duplicated code. This condenses them
  into one function.

  Arguments:
    I - the variable passed to Update
    weapon - a table containing subIdx and wpnIdx fields
             subIdx is the subconstruct identifier of the subconstruct the
               weapon is on, or nil if it is on the main hull
             wpnIdx is the weapon index
  Returns:
    info - the weapon info for the specified weapon
]]--
function BlockUtil.getWeaponInfo(I, weapon)
  if weapon.subIdx then
    return I:GetWeaponInfoOnSubConstruct(weapon.subIdx, weapon.wpnIdx)
  end
  return I:GetWeaponInfo(weapon.wpnIdx)
end

--[[
  Arguments:
    see getWeaponInfo
  Returns:
    weaponBlockInfo - the BlockInfo corresponding to the weapon
]]--
function BlockUtil.getWeaponBlockInfo(I, weapon)
  if weapon.subIdx then
    return I:GetWeaponBlockInfoOnSubConstruct(weapon.subIdx, weapon.wpnIdx)
  end
  return I:GetWeaponBlockInfo(weapon.wpnIdx)
end

--[[
  Arguments:
    I, weapon - see getWeaponInfo
    dir - the direction to aim in as a Vector3 in world space
    slot - the weapon slot to control
]]--
function BlockUtil.aimWeapon(I, weapon, dir, slot)
  if weapon.subIdx then
    I:AimWeaponInDirectionOnSubConstruct(weapon.subIdx, weapon.wpnIdx, dir.x, dir.y, dir.z, slot)
  else
    I:AimWeaponInDirection(weapon.wpnIdx, dir.x, dir.y, dir.z, slot)
  end
end

--[[
  Arguments:
    I, weapon, slot - see aimWeapon
  Returns:
    fired - whether the weapon fired or not
]]--
function BlockUtil.fireWeapon(I, weapon, slot)
  if weapon.subIdx then
    return I:FireWeaponOnSubConstruct(weapon.subIdx, weapon.wpnIdx, slot)
  end
  return I:FireWeapon(weapon.wpnIdx, slot)
end
