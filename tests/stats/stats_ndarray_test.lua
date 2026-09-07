local distribution = Stats.Distribution({"w", "x", "y", "z"})
assert(#distribution.cov.shape == 2 and distribution.cov.shape[1] == 4
    and distribution.cov.shape[2] == 4, "distribution covariance should be an NdArray")
assert(distribution.cov.dtype == "float", "distribution covariance should use floating-point storage")

Stats.updateDistribution(distribution, {w = 1, x = 2, y = 3, z = 4})

for index = 1, distribution.cov.size do
    assert(distribution.cov.data[index] == 0,
        "distribution covariance should update through NdArray storage")
end
assert(Stats.namedCovariance(distribution, "x", "y") == 0,
    "named covariance should read NdArray storage")

local function close(actual, expected, message)
  assert(math.abs(actual - expected) < 1e-10,
    message .. ": expected " .. tostring(expected) .. ", got " .. tostring(actual))
end

local function compareScalar(left, right, message)
  close(left.n, right.n, message .. " count")
  close(left.mean, right.mean, message .. " mean")
  close(left.cov, right.cov, message .. " covariance")
end

local function compareVector(left, right, message)
  close(left.n, right.n, message .. " count")
  for i, variable in ipairs(left.vars) do
    assert(variable == right.vars[i], message .. " variable order")
    close(left.mean[variable], right.mean[variable], message .. " mean " .. variable)
    for j = 1, #left.vars do
      close(left.cov.data[(i - 1) * #left.vars + j],
        right.cov.data[(i - 1) * #left.vars + j],
        message .. " covariance " .. variable .. "/" .. right.vars[j])
    end
  end
end

-- Weighted scalar updates use population covariance and agree with a batched update.
local scalarSequential = Stats.Distribution()
Stats.updateDistribution(scalarSequential, 1, 2)
Stats.updateDistribution(scalarSequential, 5, 3)
close(scalarSequential.n, 5, "weighted scalar count")
close(scalarSequential.mean, 17 / 5, "weighted scalar mean")
close(scalarSequential.cov, 96 / 25, "weighted scalar covariance")

local scalarBatched = Stats.Distribution()
Stats.updateDistributionBatched(scalarBatched, {1, 5}, {2, 3})
compareScalar(scalarBatched, scalarSequential, "weighted scalar batch")

local scalarMerged = Stats.Distribution()
Stats.mergeDistribution(scalarMerged, scalarSequential)
compareScalar(scalarMerged, scalarSequential, "merged scalar distribution")

-- A second batch must merge against the pre-existing count, not the post-update count.
local scalarReference = Stats.Distribution()
for i, sample in ipairs({1, 5, 7, 9}) do
  Stats.updateDistribution(scalarReference, sample, ({2, 3, 4, 1})[i])
end
Stats.updateDistributionBatched(scalarBatched, {7, 9}, {4, 1})
compareScalar(scalarBatched, scalarReference, "incremental scalar batch")

-- Named vector samples are the shape used by the counterbattery density path.
local samples = {{x = 1, y = 2}, {x = 5, y = -1}, {x = 3, y = 4}}
local weights = {2, 1, 3}
local vectorSequential = Stats.Distribution({"x", "y"})
for i, sample in ipairs(samples) do
  Stats.updateDistribution(vectorSequential, sample, weights[i])
end
close(vectorSequential.mean.x, 8 / 3, "weighted vector x mean")
close(vectorSequential.mean.y, 5 / 2, "weighted vector y mean")
close(Stats.namedCovariance(vectorSequential, "x", "x"), 17 / 9,
  "weighted vector x covariance")
close(Stats.namedCovariance(vectorSequential, "y", "y"), 13 / 4,
  "weighted vector y covariance")
close(Stats.namedCovariance(vectorSequential, "x", "y"), -5 / 6,
  "weighted vector xy covariance")

local vectorBatched = Stats.Distribution({"x", "y"})
Stats.updateDistributionBatched(vectorBatched, samples, weights)
compareVector(vectorBatched, vectorSequential, "weighted vector batch")

-- The internal vector path also accepts a rank-1 NdArray sample.
local ndarrayVector = Stats.Distribution({"x", "y"})
Stats.updateDistribution(ndarrayVector, NdArray.array({4, 8}, "float"), 1)
close(ndarrayVector.mean.x, 4, "NdArray vector x mean")
close(ndarrayVector.mean.y, 8, "NdArray vector y mean")

-- Merging two already-aggregated distributions uses the same parallel-moments formula.
local merged = Stats.Distribution({"x", "y"})
local firstHalf = Stats.Distribution({"x", "y"})
Stats.updateDistributionBatched(firstHalf, {samples[1], samples[2]}, {2, 1})
local secondHalf = Stats.Distribution({"x", "y"})
Stats.updateDistribution(secondHalf, samples[3], 3)
Stats.mergeDistribution(merged, firstHalf)
Stats.mergeDistribution(merged, secondHalf)
compareVector(merged, vectorSequential, "merged vector distributions")

-- Zero-weight observations are no-ops, including an all-zero batch.
local beforeZero = Stats.Distribution()
Stats.updateDistribution(beforeZero, 10)
Stats.updateDistribution(beforeZero, 999, 0)
Stats.updateDistributionBatched(beforeZero, {100, 200}, {0, 0})
close(beforeZero.n, 1, "zero-weight count")
close(beforeZero.mean, 10, "zero-weight mean")
close(beforeZero.cov, 0, "zero-weight covariance")
