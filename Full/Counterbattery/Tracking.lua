function decomposeThreeSquares(n)
    local candidates = {}
    if n % 8 == 7 then
        for i=0,math.sqrt(n) do
            local m = n - i * i
            -- Check if m can be decomposed into sum of 2 squares.
            -- This can be checked by removing factors of 2 and checking if it is congruent to 1 mod 4.
            local powersTwo = 0
            while m % 2 == 0 do
                m = m / 2
                powersTwo = powersTwo + 1
            end
            if m % 4 == 1 then
                table.insert(candidates, n)
            end
        end
    end

    return candidates
end

function checkDecomposition(candidates, ptA, ptB, checkPoints, n)
    local relPoints = {}
    local bases = {}
    local alts = {}
    local diff = ptB - ptA
    for i, pt in ipairs(checkPoints) do
        local relPt = pt - ptA
        table.insert(relPoints, relPt)
        local projection = Vector3.Project(relPt, diff)
        table.insert(bases, projection.magnitude)
        table.insert(alts, (relPt - projection).magnitude)
    end
    local sqrDist = diff.sqrMagnitude
    for i, cand in ipairs(candidates) do
        local candAlt = math.sqrt(sqrDist - cand * cand)
        for j, relPt in relPoints do
            local maxProjection = Vector3.Project(Vector3(alts[j], bases[j], 0), Vector3(candAlt, cand, 0)).magnitude
            local minProjection = Vector3.Project(Vector3(alts[j], bases[j], 0), Vector3(-candAlt, cand, 0)).magnitude
            local rangeMax = math.ceil(maxProjection)
            local rangeMin = math.floor(minProjection)
        end
    end
end

-- too slow to try all combinations of triples
-- instead, wait until we get two aimpoints within a small distance (i.e. 10 blocks) of each other
-- find the first number of the triple and check pairs
-- axis can be found by intersection of planes normal to each sample vector
-- with offset determined by the first square and unit sphere