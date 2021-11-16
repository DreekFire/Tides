local RingBuffer = {}
--[[
  A ringbuffer is an efficient FIFO buffer in the form of a circular
  array. This allows for fast removal from the ends (though the
  convention is that removal occurs from the head and addition occurs
  at the tail) and fast access of random elements in the array.
]]--

-- Creates a new ring buffer with specified capacity.
function RingBuffer.RingBuffer(capacity)
  local rb = {}
  rb.buf = {}
  rb.capacity = capacity
  rb.tail = 1
  rb.head = 1
  return rb
end

-- Checks if a RingBuffer is full.
-- If it is full, no more elements may be added without overwriting
-- previous elements.
function RingBuffer.isFull(rb)
  -- lua "modulus" (actually remainder) matches sign of divisor
  -- so no need to worry about negative numbers
  return (rb.head - rb.tail) % rb.capacity == 1
end

-- Checks if a RingBuffer is empty.
function RingBuffer.isEmpty(rb)
  return rb.head == rb.tail
end

-- Adds a value to the tail of the RingBuffer.
function RingBuffer.push(rb, value)
  rb.buf[rb.tail] = value
  if RingBuffer.isFull(rb) then
    rb.head = rb.head % rb.capacity + 1
  end
  rb.tail = rb.tail % rb.capacity + 1
end

-- Removes and returns a value from the head of the RingBuffer.
-- Returns nil if empty.
function RingBuffer.pop(rb)
  if RingBuffer.isEmpty(rb) then return nil end
  local val = rb.buf[rb.head]
  rb.buf[rb.head] = nil
  rb.head = rb.head % rb.capacity + 1
  return val
end