-- A priority queue is a data structure which retrieves the smallest (or largest) value stored
-- A heap priority queue is a commonly used efficient implementation of a priority queue
-- These functions implement a min-heap, which retrieves the smallest value
-- If you need the largest one, you can just store the negative of the values or provide a custom comparator

function Heapq.Heapq(initial, comparator)
  local heapq = {}
  heapq.data = initial
  heapq.comp = comparator or function(a, b)
    return a < b
  end
  local n = #heapq.data
  heapq.size = n
  for idx = math.floor(n / 2), 1, -1 do
    Heapq.siftDown(heapq, idx)
  end
  return heapq
end

function Heapq.siftDown(heapq, position)
  local heaped = false
  local s = position
  local n = #heapq.data
  while not heaped do
    heaped = true
    local left = 2 * s
    local right = 2 * s + 1
    local argmin = s
    if left <= n and heapq.comp(heapq.data[left], heapq.data[argmin]) then
      argmin = left
      heaped = false
    end
    if right <= n and heapq.comp(heapq.data[right], heapq.data[argmin]) then
      argmin = right
      heaped = false
    end
    if not heaped then
      local swap = heapq.data[argmin]
      heapq.data[argmin] = heapq.data[s]
      heapq.data[s] = swap
      s = argmin
    end
  end
end

function Heapq.siftUp(heapq, position)
  local heaped = false
  local s = position
  while not heaped do
    heaped = true
    local parent = math.floor(s / 2)
    if heapq.comp(heapq.data[s], heapq.data[parent]) then
      local swap = heapq.data[parent]
      heapq.data[parent] = heapq.data[s]
      heapq.data[s] = swap
      s = parent
      heaped = false
    end
  end
end

function Heapq.insert(heapq, item)
  heapq.data[heapq.size + 1] = item
  heapq.size = heapq.size + 1
  Heapq.siftUp(heapq, heapq.size)
end

function Heapq.pop(heapq)
  local res = heapq.data[1]
  heapq.data[1] = heapq.data[heapq.size]
  heapq.data[heapq.size] = nil
  heapq.size = heapq.size - 1
  Heapq.siftDown(heapq, 1)
  return res
end

function Heapq.peek(heapq)
  return heapq.data[1]
end

function Heapq.size(heapq)
  return heapq.size
end
