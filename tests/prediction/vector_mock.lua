local vectorMeta = {}
vectorMeta.__add = function(a,b) return Vector3(a.x+b.x,a.y+b.y,a.z+b.z) end
vectorMeta.__sub = function(a,b) return Vector3(a.x-b.x,a.y-b.y,a.z-b.z) end
vectorMeta.__mul = function(a,b)
  if type(a) == "number" then a,b = b,a end
  return Vector3(a.x*b,a.y*b,a.z*b)
end
vectorMeta.__div = function(a,b) return Vector3(a.x/b,a.y/b,a.z/b) end
vectorMeta.__index = function(v,k)
  if k == "sqrMagnitude" then return v.x^2+v.y^2+v.z^2 end
  if k == "magnitude" then return math.sqrt(v.x^2+v.y^2+v.z^2) end
end
Vector3 = setmetatable({}, {__call = function(_,x,y,z)
  return setmetatable({x=x,y=y,z=z},vectorMeta)
end})
Vector3.zero = Vector3(0,0,0)
Vector3.Dot = function(a,b) return a.x*b.x+a.y*b.y+a.z*b.z end
Quaternion = {identity = {}}
