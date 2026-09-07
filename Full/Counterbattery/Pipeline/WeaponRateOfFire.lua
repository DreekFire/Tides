-- target/track-cluster/rate-of-fire
-- Persistent timing estimates for inferred cannon/weapon trajectory clusters.
local TargetWeaponRateOfFire = {}

local function median(values)
  if #values == 0 then return nil end
  table.sort(values)
  local middle = math.floor((#values + 1) / 2)
  if #values % 2 == 1 then return values[middle] end
  return 0.5 * (values[middle] + values[middle + 1])
end

local function copySortedUnique(values)
  local copy = {}
  for _, value in ipairs(values) do copy[#copy + 1] = value end
  table.sort(copy)
  local unique = {}
  for _, value in ipairs(copy) do
    if #unique == 0 or value ~= unique[#unique] then unique[#unique + 1] = value end
  end
  return unique
end

local function lineMatchDistance(lineDistance, first, second)
  if not first or not second then return math.huge end
  return lineDistance(first, second)
end

function TargetWeaponRateOfFire.new(config)
  config = config or {}
  return setmetatable({
    -- A weapon id is only meaningful for one target.  Keeping records per
    -- target prevents two enemies with parallel firing lines from sharing a
    -- shot history.
    recordsByTarget = {},
    nextId = 1,
    historySeconds = config.rateOfFireHistorySeconds or 30,
    matchDistance = config.rateOfFireMatchDistance or (config.trackClusterDistance or 5) * 2,
  }, {__index = TargetWeaponRateOfFire})
end

function TargetWeaponRateOfFire:findRecord(records, line, lineDistance, assigned)
  local best
  local bestDistance = self.matchDistance
  for _, record in ipairs(records) do
    if not assigned[record] then
      local distance = lineMatchDistance(lineDistance, record.line, line)
      if distance <= bestDistance then
        best = record
        bestDistance = distance
      end
    end
  end
  return best
end

function TargetWeaponRateOfFire:update(targetId, clusters, stamp, lineDistance)
  local records = self.recordsByTarget[targetId]
  if not records then
    records = {}
    self.recordsByTarget[targetId] = records
  end
  local assigned = {}
  local products = {}
  for _, cluster in ipairs(clusters) do
    local record = self:findRecord(records, cluster.line, lineDistance, assigned)
    if not record then
      record = {id = self.nextId, line = cluster.line, times = {}}
      self.nextId = self.nextId + 1
      records[#records + 1] = record
    end
    assigned[record] = true
    record.line = cluster.line
    for _, member in ipairs(cluster.members or {}) do
      if member.originTime ~= nil then record.times[#record.times + 1] = member.originTime end
    end

    local times = copySortedUnique(record.times)
    local oldest = stamp - self.historySeconds
    local retained = {}
    for _, time in ipairs(times) do
      if time >= oldest then retained[#retained + 1] = time end
    end
    record.times = retained
    local intervals = {}
    for index = 2, #retained do
      local interval = retained[index] - retained[index - 1]
      if interval > 0 then intervals[#intervals + 1] = interval end
    end
    local intervalEstimate = median(intervals)
    local rate = intervalEstimate and 1 / intervalEstimate or nil
    products[#products + 1] = CounterbatteryTypes.product(
      "target/track-cluster/rate-of-fire", targetId, stamp, {
        weaponId = record.id,
        shotCount = #retained,
        interval = intervalEstimate,
        rate = rate,
        confidence = math.min(1, #intervals / 3),
        firstShotTime = retained[1],
        lastShotTime = retained[#retained],
      })
  end
  return products
end

_G.TargetWeaponRateOfFire = TargetWeaponRateOfFire
