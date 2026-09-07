-- Runtime tags for the counterbattery data pipeline.
--
-- These are values, not in-place mutations of the source target object.  The
-- path-like `type` names describe the product's enrichment stage.
local CounterbatteryTypes = {}

function CounterbatteryTypes.product(path, targetId, stamp, fields)
  local product = {
    type = path,
    targetId = targetId,
    stamp = stamp,
  }
  for key, value in pairs(fields or {}) do
    product[key] = value
  end
  return product
end

function CounterbatteryTypes.collection(path, stamp, byId, fields)
  local product = CounterbatteryTypes.product(path, nil, stamp, fields)
  product.byId = byId or {}
  return product
end
