Stats = Stats or {}

function Stats._statsAssertFiniteWeight(weight)
  assert(type(weight) == "number" and weight == weight and math.abs(weight) ~= math.huge,
    "distribution weights must be finite numbers")
  assert(weight >= 0, "distribution weights must be non-negative")
end

function Stats._statsCovarianceIndex(cov, row, col)
  assert(type(cov) == "table" and cov.shape and cov.stride and cov.data,
    "distribution covariance must be an NdArray")
  assert(#cov.shape == 2 and cov.shape[1] == cov.shape[2],
    "distribution covariance must be a square rank-2 NdArray")
  return (cov.offset or 0) + (row - 1) * cov.stride[1]
      + (col - 1) * cov.stride[2] + 1
end

function Stats._statsCovarianceGet(cov, row, col)
  return cov.data[Stats._statsCovarianceIndex(cov, row, col)]
end

function Stats._statsCovarianceSet(cov, row, col, value)
  cov.data[Stats._statsCovarianceIndex(cov, row, col)] = value
end

function Stats._statsSampleValue(sample, variable, index)
  if type(sample) == "table" and sample.shape and sample.stride and sample.data then
    assert(#sample.shape == 1, "vector samples must be rank-1 NdArrays")
    return sample.data[(sample.offset or 0) + (index - 1) * sample.stride[1] + 1]
  end
  return sample[variable]
end

function Stats._statsVectorMean(distr)
  local mean = NdArray.zeros({#distr.vars}, "float")
  for i, variable in ipairs(distr.vars) do
    mean.data[i] = distr.mean[variable]
  end
  return mean
end

function Stats._statsMergeMoments(distr, incomingN, incomingMean, incomingCov)
  local oldN = distr.n
  assert(type(oldN) == "number" and oldN >= 0 and oldN == oldN and math.abs(oldN) ~= math.huge,
    "distribution count must be a finite non-negative number")
  assert(incomingN > 0, "cannot merge a distribution with zero weight")

  local newN = oldN + incomingN
  if not distr.vars then
    local oldMean = distr.mean
    local delta = incomingMean - oldMean
    local newMean = oldMean + incomingN / newN * delta
    local newCov = (oldN * distr.cov + incomingN * incomingCov
        + oldN * incomingN / newN * delta * delta) / newN
    distr.mean = newMean
    distr.cov = newCov
  else
    local nVars = #distr.vars
    local oldMean = Stats._statsVectorMean(distr)
    local delta = NdArray.zeros({nVars}, "float")
    local newMean = NdArray.zeros({nVars}, "float")
    for i = 1, nVars do
      delta.data[i] = incomingMean.data[i] - oldMean.data[i]
      newMean.data[i] = oldMean.data[i] + incomingN / newN * delta.data[i]
    end

    for i, variable in ipairs(distr.vars) do
      distr.mean[variable] = newMean.data[i]
      for j = i, nVars do
        local correction = oldN * incomingN / newN * delta.data[i] * delta.data[j]
        local value = (oldN * Stats._statsCovarianceGet(distr.cov, i, j)
            + incomingN * Stats._statsCovarianceGet(incomingCov, i, j) + correction) / newN
        Stats._statsCovarianceSet(distr.cov, i, j, value)
        Stats._statsCovarianceSet(distr.cov, j, i, value)
      end
    end
  end
  distr.n = newN
end

function Stats.Distribution(vars)
  local distr = { n = 0, vars = vars }
  if vars then
    local mean = {}
    local nVars = #vars
    local cov = NdArray.zeros({nVars, nVars}, "float")
    for i, v in ipairs(vars) do
      mean[v] = 0
    end
    distr.mean = mean
    distr.cov = cov
  else
    distr.mean = 0
    distr.cov = 0
  end
  return distr
end

function Stats.updateDistribution(distr, sample, weight)
  weight = weight or 1
  Stats._statsAssertFiniteWeight(weight)
  if weight == 0 then return end

  if distr.vars then
    local mean = NdArray.zeros({#distr.vars}, "float")
    for i, variable in ipairs(distr.vars) do
      mean.data[i] = Stats._statsSampleValue(sample, variable, i)
    end
    local cov = NdArray.zeros({#distr.vars, #distr.vars}, "float")
    Stats._statsMergeMoments(distr, weight, mean, cov)
  else
    Stats._statsMergeMoments(distr, weight, sample, 0)
  end
end

function Stats.updateDistributionBatched(distr, samples, weights)
  if #samples == 0 then return end
  local wSum = 0
  for j=1, #samples do
    local weight = weights and weights[j] or 1
    Stats._statsAssertFiniteWeight(weight)
    wSum = wSum + weight
  end
  if wSum == 0 then return end

  if distr.vars then
    local nVars = #distr.vars
    local mean = NdArray.zeros({nVars}, "float")
    for sampleIndex, sample in ipairs(samples) do
      local weight = weights and weights[sampleIndex] or 1
      for i, variable in ipairs(distr.vars) do
        mean.data[i] = mean.data[i] + weight * Stats._statsSampleValue(sample, variable, i)
      end
    end
    for i = 1, nVars do
      mean.data[i] = mean.data[i] / wSum
    end

    local cov = NdArray.zeros({nVars, nVars}, "float")
    for sampleIndex, sample in ipairs(samples) do
      local weight = weights and weights[sampleIndex] or 1
      for i, variableI in ipairs(distr.vars) do
        local centeredI = Stats._statsSampleValue(sample, variableI, i) - mean.data[i]
        for j = i, nVars do
          local centeredJ = Stats._statsSampleValue(sample, distr.vars[j], j) - mean.data[j]
          Stats._statsCovarianceSet(cov, i, j, Stats._statsCovarianceGet(cov, i, j)
              + weight * centeredI * centeredJ / wSum)
        end
      end
    end
    for i = 1, nVars do
      for j = i + 1, nVars do
        Stats._statsCovarianceSet(cov, j, i, Stats._statsCovarianceGet(cov, i, j))
      end
    end
    Stats._statsMergeMoments(distr, wSum, mean, cov)
  else
    local mean = 0
    for i, sample in ipairs(samples) do
      mean = mean + sample * (weights and weights[i] or 1)
    end
    mean = mean / wSum
    local cov = 0
    for i, sample in ipairs(samples) do
      local centered = sample - mean
      cov = cov + (weights and weights[i] or 1) * centered * centered / wSum
    end
    Stats._statsMergeMoments(distr, wSum, mean, cov)
  end
end

function Stats.mergeDistribution(distr, other)
  assert(type(other) == "table" and type(other.n) == "number", "expected a distribution to merge")
  assert(other.n >= 0 and other.n == other.n and math.abs(other.n) ~= math.huge,
    "distribution count must be a finite non-negative number")
  if other.n == 0 then return end

  if distr.vars then
    assert(other.vars and #other.vars == #distr.vars, "distribution dimensions must match")
    local mean = NdArray.zeros({#distr.vars}, "float")
    for i, variable in ipairs(distr.vars) do
      assert(other.vars[i] == variable, "distribution variable order must match")
      mean.data[i] = other.mean[variable]
    end
    Stats._statsMergeMoments(distr, other.n, mean, other.cov)
  else
    assert(not other.vars, "distribution dimensions must match")
    Stats._statsMergeMoments(distr, other.n, other.mean, other.cov)
  end
end

function Stats.mean(distr)
  return distr.mean
end

function Stats.covariance(distr)
  return distr.cov
end

function Stats.namedCovariance(distr, var1, var2)
  if not distr.vars then return distr.cov end
  local nVars = distr.vars and #distr.vars or 1
  for i=1,nVars do
    if distr.vars[i] == var1 then
      for j=1,nVars do
        if distr.vars[j] == var2 then
          return Stats._statsCovarianceGet(distr.cov, i, j)
        end
      end
    end
  end
end

--[[
  Returns:
    z - a normally distributed random number
--]]
function Stats.normal()
  local z, z1 = Stats.boxMuller()
  return z
end

--[[
  Arguments:
    z - a z-score
  Returns:
    d - the standard normal probability density function at z
]]
function Stats.normalPDF(z)
  return math.exp(-0.5 * z * z) / math.sqrt(2 * math.pi)
end

--[[
  Arguments:
    z - a z-score
  Returns:
    p - the p-value corresponding to that z-score. Approximately calculated using Zelen and Severo (1964) approximation
--]]
function Stats.normalCDF(z)
  local b0 = 0.2316419
  local b1 = 0.319381530
  local b2 = -0.356563782
  local b3 = 1.781477937
  local b4 = -1.821255978
  local b5 = 1.330274429
  local t = 1 / (1 + b0 * z)
  return 1 - Stats.normalPDF(z) * (b1 * t + b2 * t^2 + b3 * t^3 + b4 * t^4 + b5 * t^5)
end

--[[
  Arguments:
    p - a p-value
  Returns:
    z - the z-score corresponding to that p-value. Approximately calculated using Shore (1982) approximation
--]]
function Stats.inverseNorm(p)
  local pRight = p >= 0.5 and p or -p
  local z = 5.55556 * (1 - ((1 - pRight) / pRight) ^ 0.1186) -- Shore (1982) approximation for normal quantile
  if p < 0.5 then z = -z end
  return z
end

--[[
  Returns:
    z1, z2 - two normally distributed random numbers
--]]
function Stats.boxMuller()
  local u1 = math.random()
  local u2 = math.random() -- idk what algorithm math.random uses, but it might be an LCG since it's simple and common
  u2 = math.random() -- it's also not very high-quality, which also describes LCGs. Box-Muller performs poorly when the
  u2 = math.random() -- input comes from two consecutive numbers of an LCG, so we throw away values in between.
  local r = math.sqrt(-2 * math.log(u1))
  local theta = 2 * math.pi * u2
  return r * math.cos(theta), r * math.sin(theta)
end
