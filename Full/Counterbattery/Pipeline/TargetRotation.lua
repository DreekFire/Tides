-- target/rotation
-- Rotation tracking from corresponding aimpoint triplets.
local TargetRotation = {}

function TargetRotation.new(config)
  return setmetatable({
    aimPointTrackers = config.aimPointTrackers,
    maxTurnRateTracking = config.maxTurnRateTracking,
    TICKS_PER_S = config.TICKS_PER_S,
    trackLossTime = 0,
    ransacInlierSqrThreshold = config.ransacInlierSqrThreshold or 0.1,
    ransacLegSqrTolerance = config.ransacLegSqrTolerance or 0.1,
    ransacLegRelativeTolerance = config.ransacLegRelativeTolerance or 0.01,
    ransacMinTriangleAreaRatio = config.ransacMinTriangleAreaRatio or 1e-6,
    ransacSeed = config.ransacSeed or 1,
  }, {__index = TargetRotation})
end

function TargetRotation:update(I, tracks, aimpoints, stamp)
  local byId = {}
  for id, track in pairs(tracks.byId) do
    local en = track.state
    local dRot, consensusPoints
    if #self.aimPointTrackers < 3 then
      I:Log("Need at least 3 AIs to track rotation")
      dRot = nil
    else
      dRot, consensusPoints = self:getRotation(en.oldAimpoints, en.aimpoints)
    end

    if dRot then
      RingBuffer.push(en.rotation, dRot * (en.rotation[en.rotation.size] or Quaternion.identity))
    else
      RingBuffer.push(en.rotation, en.rotation[en.rotation.size] or Quaternion.identity)
      self.trackLossTime = self.trackLossTime + 1 / self.TICKS_PER_S
      I:Log(string.format("track loss time: %.2f", self.trackLossTime))
    end

    byId[id] = CounterbatteryTypes.product("target/rotation", id, stamp, {
      frame = "target-local",
      deltaRotation = dRot,
      rotation = en.rotation[en.rotation.size],
      consensusPoints = consensusPoints,
      state = en,
    })
  end
  return CounterbatteryTypes.collection("target/rotation", stamp, byId, {
    frame = "target-local",
  })
end

local function validPoint(point)
  local pointType = type(point)
  return (pointType == "table" or pointType == "userdata")
    and type(point.x) == "number"
    and type(point.y) == "number"
    and type(point.z) == "number"
    and point.x == point.x
    and point.y == point.y
    and point.z == point.z
end

local function combinationCount(n)
  return n * (n - 1) * (n - 2) / 6
end

-- Return the zero-based lexicographic combination at rank. This lets small
-- RANSAC problems exhaustively test every triangle without using global RNG.
local function tripletAtRank(rank, n)
  local i, j
  for candidateI = 1, n - 2 do
    local count = (n - candidateI) * (n - candidateI - 1) / 2
    if rank < count then
      i = candidateI
      break
    end
    rank = rank - count
  end
  for candidateJ = i + 1, n - 1 do
    local count = n - candidateJ
    if rank < count then
      j = candidateJ
      break
    end
    rank = rank - count
  end
  return i, j, j + 1 + rank
end

local function nextRandom(state)
  -- Park-Miller is deterministic and does not perturb FtD's global RNG.
  return (state * 48271) % 2147483647
end

local function triangleAreaRatio(a, b)
  local denominator = math.max(1, a.sqrMagnitude * b.sqrMagnitude)
  return Vector3.Cross(a, b).sqrMagnitude / denominator
end

function TargetRotation:getRotation(oldPts, newPts, iterLim)
  if oldPts == nil or newPts == nil or #oldPts ~= #newPts then
    return nil
  end
  local nTrack = #oldPts
  iterLim = math.floor(iterLim or 25)
  if nTrack < 3 or iterLim < 1 then
    return nil
  end

  for idx = 1, nTrack do
    if not validPoint(oldPts[idx]) or not validPoint(newPts[idx]) then
      return nil
    end
  end

  local legSqrTolerance = self.ransacLegSqrTolerance or 0.1
  local legRelativeTolerance = self.ransacLegRelativeTolerance or 0.01
  local minTriangleAreaRatio = self.ransacMinTriangleAreaRatio or 1e-6
  local inlierSqrThreshold = self.ransacInlierSqrThreshold or 0.1
  local maxTurnRate = self.maxTurnRateTracking or math.huge
  local ticksPerSecond = self.TICKS_PER_S or 1

  local function validLegLength(oldLeg, newLeg)
    local scale = math.max(1, oldLeg.sqrMagnitude, newLeg.sqrMagnitude)
    return math.abs(oldLeg.sqrMagnitude - newLeg.sqrMagnitude)
      <= legSqrTolerance + legRelativeTolerance * scale
  end

  local function makeHypothesis(i, j, k)
    local oldA = oldPts[j] - oldPts[i]
    local oldB = oldPts[k] - oldPts[i]
    local newA = newPts[j] - newPts[i]
    local newB = newPts[k] - newPts[i]
    local oldC = oldPts[k] - oldPts[j]
    local newC = newPts[k] - newPts[j]

    if not validLegLength(oldA, newA)
        or not validLegLength(oldB, newB)
        or not validLegLength(oldC, newC)
        or triangleAreaRatio(oldA, oldB) <= minTriangleAreaRatio
        or triangleAreaRatio(newA, newB) <= minTriangleAreaRatio then
      return nil
    end

    local oldRot = Quaternion.LookRotation(oldA, oldB)
    local newRot = Quaternion.LookRotation(newA, newB)
    if oldRot == nil or newRot == nil then
      return nil
    end
    local deltaRot = newRot * Quaternion.Inverse(oldRot)
    local w = math.max(-1, math.min(1, math.abs(deltaRot.w)))
    local turnRate = 2 * math.acos(w) * 180 / math.pi * ticksPerSecond
    if turnRate >= maxTurnRate then
      return nil
    end
    return deltaRot
  end

  local function score(deltaRot, anchor)
    local points = {}
    local totalError = 0
    for idx = 1, nTrack do
      local diff = newPts[idx] - newPts[anchor]
      local oldDiff = oldPts[idx] - oldPts[anchor]
      local error = (diff - deltaRot * oldDiff).sqrMagnitude
      if error <= inlierSqrThreshold then
        points[#points + 1] = idx
        totalError = totalError + error
      end
    end
    return #points, totalError, points
  end

  local function better(count, error, bestCount, bestError)
    return bestCount == nil
      or count > bestCount
      or (count == bestCount and error < bestError)
  end

  local totalTriplets = combinationCount(nTrack)
  local attempts = math.min(iterLim, totalTriplets)
  local randomState = math.floor(self.ransacSeed or 1) % 2147483647
  if randomState <= 0 then randomState = 1 end
  local seen = {}
  local bestRotation, bestCount, bestError, bestPoints

  for attempt = 1, attempts do
    local rank
    if totalTriplets <= iterLim then
      rank = attempt - 1
    else
      randomState = nextRandom(randomState)
      rank = randomState % totalTriplets
      while seen[rank] do
        rank = (rank + 1) % totalTriplets
      end
    end
    seen[rank] = true

    local i, j, k = tripletAtRank(rank, nTrack)
    local deltaRot = makeHypothesis(i, j, k)
    if deltaRot then
      local count, error, points = score(deltaRot, i)
      if better(count, error, bestCount, bestError) then
        bestRotation, bestCount, bestError, bestPoints = deltaRot, count, error, points
      end
      if bestCount == nTrack then
        break
      end
    end
  end

  local minimumInliers = math.max(3, math.floor(nTrack / 2) + 1)
  if bestRotation == nil or bestCount < minimumInliers then
    return nil
  end

  -- Refit deterministically from the best-conditioned triangle in the
  -- consensus set, then score it against every correspondence again.
  for refinement = 1, 2 do
    local supportI, supportJ, supportK, supportQuality
    for a = 1, #bestPoints - 2 do
      for b = a + 1, #bestPoints - 1 do
        for c = b + 1, #bestPoints do
          local i, j, k = bestPoints[a], bestPoints[b], bestPoints[c]
          local oldA = oldPts[j] - oldPts[i]
          local oldB = oldPts[k] - oldPts[i]
          local newA = newPts[j] - newPts[i]
          local newB = newPts[k] - newPts[i]
          local quality = math.min(triangleAreaRatio(oldA, oldB), triangleAreaRatio(newA, newB))
          if supportQuality == nil or quality > supportQuality then
            supportI, supportJ, supportK, supportQuality = i, j, k, quality
          end
        end
      end
    end

    if supportI == nil then
      break
    end
    local refitRotation = makeHypothesis(supportI, supportJ, supportK)
    if refitRotation == nil then
      break
    end
    local refitCount, refitError, refitPoints = score(refitRotation, supportI)
    if better(refitCount, refitError, bestCount, bestError) then
      bestRotation, bestCount, bestError, bestPoints = refitRotation, refitCount, refitError, refitPoints
    else
      break
    end
  end

  if bestCount < minimumInliers then
    return nil
  end
  return bestRotation, bestPoints
end
