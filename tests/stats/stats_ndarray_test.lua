local distribution = Stats.Distribution({"w", "x", "y", "z"})
assert(#distribution.cov.shape == 2 and distribution.cov.shape[1] == 4
    and distribution.cov.shape[2] == 4, "distribution covariance should be an NdArray")

Stats.updateDistribution(distribution, {w = 1, x = 2, y = 3, z = 4})

for index = 1, distribution.cov.size do
    assert(distribution.cov.data[index] == 0,
        "distribution covariance should update through NdArray storage")
end
assert(Stats.namedCovariance(distribution, "x", "y") == 0,
    "named covariance should read NdArray storage")
