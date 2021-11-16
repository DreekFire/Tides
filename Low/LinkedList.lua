local LinkedList = {}
-- A LinkedList is a chain of elements where each element contains a reference to the next one in the list.
-- This is a doubly linked list so each element also contains a reference to the previous element.

-- Creates a new linked list.
function LinkedList.LinkedList()
  local llelem = {}
  llelem.value = nil
  llelem.next = llelem
  llelem.prev = llelem
  return llelem
end

-- Adds value to the front of lst.
function LinkedList.pushFront(lst, value)
  local llelem = {}
  llelem.value = value
  LinkedList.connect(llelem, lst.next)
  LinkedList.connect(lst, llelem)
end

-- Adds value to the back of lst.
function LinkedList.pushBack(lst, value)
  local llelem = {}
  llelem.value = value
  LinkedList.connect(lst.prev, llelem)
  LinkedList.connect(llelem, lst)
end

-- Removes and returns the value at the front of lst.
-- Returns nil if empty.
function LinkedList.popFront(lst)
  local val = lst.next.value
  LinkedList.connect(lst, lst.next.next)
  return val
end

-- Removes and returns the value at the back of lst.
-- Returns nil if empty.
function LinkedList.popBack(lst)
  local val = lst.prev.value
  LinkedList.connect(lst.prev.prev, lst)
  return val
end

-- Returns the value at the front of lst. Does not remove it.
-- Returns nil if empty.
function LinkedList.peekFront(lst)
  return lst.next.val
end

-- Returns the value at the back of lst. Does not remove it.
-- Returns nil if empty.
function LinkedList.peekBack(lst)
  return lst.prev.val
end

-- Joins two list elements. Internal use, you probably don't need this.
function LinkedList.connect(e1, e2)
  e1.next = e2
  e2.prev = e1
end

-- Converts a linked list into a lua list (a table with numeric keys).
function LinkedList.toArray(lst)
  local i = 1
  local arr = {}
  local llelem = lst.next
  while llelem ~= lst do
    arr[i] = llelem.value
    llelem = llelem.next
  end
  return arr
end
