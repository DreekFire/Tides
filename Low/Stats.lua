function Stats.Distribution(vars)
  local distr = { n = 0, vars = vars }
  if vars then
    local mean = {}
    local cov = {}
    local nVars = #vars
    for i, v in ipairs(vars) do
      mean[v] = 0
      for j=1, nVars do
        cov[(i - 1) * nVars + j] = 0
      end
    end
    distr.mean = mean
    distr.cov = Matrix3.Matrix3(cov)
  else
    distr.mean = 0
    distr.cov = 0
  end
  return distr
end

function Stats.updateDistribution(distr, sample, weight)
  local oldN = distr.n
  weight = weight or 1
  distr.n = distr.n + weight

  if distr.vars then
    local oldMeans = {}
    local nVars = distr.vars and #distr.vars or 1
    for i, v in ipairs(distr.vars) do
      -- Update mean
      oldMeans[i] = distr.mean[v]
      local mu = oldMeans[i] + sample[v] * weight / distr.n

      -- Update covariance
      for j=i, nVars do
        local v2 = distr.vars[j]
        local mu2 = distr.mean[v2]
        local upd = (weight or 1) * (sample[v] - mu) * (sample[v2] - mu2)
        distr.cov[(i - 1) * nVars + j] = (distr.cov[(i - 1) * nVars + j] * oldN + upd) / distr.n
        distr.cov[(j - 1) * nVars + i] = distr.cov[(i - 1) * nVars + j]
      end

      distr.mean[v] = mu
    end
  else
    local mu = distr.mean + sample * weight / distr.n
    distr.cov = (distr.cov * oldN + weight * (sample - mu) * (sample - distr.mean)) / distr.n

    distr.mean = mu
  end
end

function Stats.updateDistributionBatched(distr, samples, weights)
  if #samples == 0 then return end
  local sT = {}
  local nVars = distr.vars and #distr.vars or 1

  local wSum = 0
  for j=1, #samples do
    wSum = wSum + (weights and weights[j] or 1)
  end
  distr.n = distr.n + wSum

  local oldN = distr.n
  if distr.vars then
    for i, var in ipairs(distr.vars) do
      local col = {}
      for j=1, #samples do
        col[j] = samples[j][var]
      end
      sT[i] = col
    end

    for i, v in ipairs(distr.vars) do
      -- Update mean
      local sum = 0
      for j, s in ipairs(sT[i]) do
        sum = sum + s * (weights and weights[j] or 1)
      end
      local mu = distr.mean[v] + sum / distr.n

      -- Update covariance
      for j=i, nVars do
        local mu2 = distr.mean[distr.vars[j]]
        sum = 0
        for s=1, #samples do
          sum = sum + (weights and weights[s] or 1) * (sT[i][s] - mu) * (sT[j][s] - mu2)
        end
        distr.cov[(i - 1) * nVars + j] = (distr.cov[(i - 1) * nVars + j] * oldN + sum) / distr.n
        distr.cov[(j - 1) * nVars + i] = distr.cov[(i - 1) * nVars + j]
      end

      distr.mean[v] = mu
    end
  else
    local sum = 0
    for i, s in ipairs(samples) do
      sum = sum + s * (weights and weights[i] or 1)
    end

    local mu = distr.mean + sum / distr.n

    sum = 0
    for i, s in ipairs(samples) do
      sum = sum + (weights and weights[i] or 1) * (s - mu) * (s - distr.mean)
    end
    distr.cov = (distr.cov * oldN + sum) / distr.n
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
          return distr.cov[(i - 1) * nVars + j]
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
