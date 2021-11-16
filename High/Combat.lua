local Combat = {}

--[[
  Arguments:
    I - the variable passed to Update
    mainframe - the mainframe index to use
    priorityFunc (optional) - a function that takes I and a
                                target and returns its priority.
                                Uses AI priority if empty
  Returns:
    target - the highest priority target
    priority - the priority of that target
--]]
function Combat.pickTarget(I, mainframe, priorityFunc)
  priorityFunc = priorityFunc or function(_, tar) return tar.Priority end
  local target, priority
  for i in MathUtil.range(I:GetNumberOfTargets(mainframe)) do
    local tar = I:GetTargetInfo(mainframe, i)
    local newPri = priorityFunc(I, tar)
    if not target or newPri > priority then
      target = tar
      priority = newPri
    end
  end
  return target
end

--[[
  Arguments:
    I - the variable passed to Update
    direction - the global direction of fire
    wepId - the ID of the weapon
    subObjId (optional) - if the weapon is on a subobject,
                            the ID of the subobject. If it
                            is on nested subobjects, the one
                            directly connected to the weapon
  Returns:
    valid - whether or not the direction is within the firing constraints
--]]
function CheckConstraints(I, direction, wepId, subObjId)
  local con
  if subObjId then
    con = I:GetWeaponConstraintsOnSubConstruct(subObjId, wepId)
  else
    con = I:GetWeaponConstraints(wepId)
  end
  local fore = I:GetConstructForwardVEctor()
  local up = I:GetConstructUpVector()
  local constructRot = Quaternion.LookRotation(fore, up)
  direction = Quaternion.Inverse(constructRot) * direction
  if con.InParentConstructSpace and subObjId then
    local rot = I:GetSubConstructInfo(subObjId).localRotation
    direction = Quaternion.inverse(rot) * direction
  end
  local azi = angleOnPlane(Vector3.forward, direction, Vector3.up)
  local aziDir = direction
  aziDir.z = 0
  local elevation = Mathf.Atan2(direction.z, aziDir.magnitude)
  local aziValid = azi > con.MinAzimuth and azi < con.MaxAzimuth
  local eleValid = elevation > con.MinElevation and elevation < con.MaxElevation
  if con.FlipAzimuth then aziValid = not aziValid end
  if aziValid and eleValid then return true end
  azi = azi + 180
  ele = 180 - ele
  if ele > 180 then ele = ele - 360 end
  if ele < -180 then ele = ele + 360 end
  aziValid = azi > con.MinAzimuth and azi < con.MaxAzimuth
  eleValid = elevation > con.MinElevation and elevation < con.MaxElevation
  if con.FlipAzimuth then aziValid = not aziValid end
  if aziValid and eleValid then return true end
  return false
end
