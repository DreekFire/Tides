function Search.interpolatedSearch(list, left, right, target, findClosest, iterLim)
    iterLim = iterLim or 50
    local a, b, split
    local totalIter = 0
    while right > left do
      a = list[left]
      if a == target then return left end
      if a > target then return findClosest and left or nil end
      b = list[right]
      if b == target then return right end
      if b < target then return findClosest and right or nil end
      split = math.floor((target - a) / (b - a) * (right - left) + left)
      split = math.min(math.max(split, left + 1), right - 1)
      if list[split] == target then return split end
      if target < list[split] then
        if findClosest and math.abs(list[split - 1] - target) > math.abs(list[split] - target) then
          return split
        end
        right = split - 1
      else
        if findClosest and math.abs(list[split + 1] - target) > math.abs(list[split] - target) then
          return split
        end
        left = split + 1
      end
      totalIter = totalIter + 1
      if totalIter > iterLim then
        break
      end
    end
    return findClosest and left or nil
  end
