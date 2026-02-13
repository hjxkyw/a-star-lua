#!/usr/bin/env lua
-- a_star.lua
-- --------------------------------------------------------------------------------

-- ==============================================================================
-- PART 1: The Generic Search Framework
-- ==============================================================================

-- Class: Node
-- A Node represents a specific "promise" in the search tree.
local Node = {}
Node.__index = Node

function Node.new(args)
  local self = setmetatable({}, Node)
  self.location = args.location
  self.parent = args.parent
  self.action = args.action
  self.g_cost = args.g_cost or 0
  self.h_cost = args.h_cost or 0
  return self
end

function Node:f_cost()
  return self.g_cost + self.h_cost
end

-- ==============================================================================
-- PART 2: Implementation (Grid and Coordinates)
-- ==============================================================================

-- Class: Location
-- Represents a physical 2D coordinate (x, y) on the map.
local Location = {}
Location.__index = Location

function Location.new(x, y)
  local self = setmetatable({}, Location)
  self.x = x
  self.y = y
  return self
end

-- Returns a unique string key used for tracking explored locations.
function Location:key()
  return self.x .. "," .. self.y
end

function Location:__tostring()
  return string.format("(Col %d, Row %d)", self.x, self.y)
end

-- Manhattan Distance
function Location:dist(p)
  return math.abs(self.x - p.x) + math.abs(self.y - p.y)
end

function Location:__eq(other)
  return self.x == other.x and self.y == other.y
end

-- Class: TerrainMap
-- Manages the grid environment, terrain costs, and the visual rendering logic.
local TerrainMap = {}
TerrainMap.__index = TerrainMap

TerrainMap.COST_GRASS = 1
TerrainMap.COST_MUD   = 10

function TerrainMap.new(args)
  local self = setmetatable({}, TerrainMap)
  self.width = args.width
  self.height = args.height
  self.start = args.start
  self.goal = args.goal
  self.mud_tiles = {}

  -- TWEAK equivalent: Randomly generate mud tiles
  math.randomseed(os.time())
  for x = 0, self.width - 1 do
    for y = 0, self.height - 1 do
      local p = Location.new(x, y)
      if not (p == self.start or p == self.goal) then
        if math.random() < 0.3 then
          self.mud_tiles[p:key()] = true
        end
      end
    end
  end

  return self
end

function TerrainMap:initial_location()
  return self.start
end

function TerrainMap:is_goal(loc)
  return loc == self.goal
end

function TerrainMap:heuristic(loc)
  return loc:dist(self.goal)
end

function TerrainMap:is_mud(loc)
  return self.mud_tiles[loc:key()] or false
end

-- method: successors
function TerrainMap:successors(loc)
  local moves = {}
  local directions = {
    {0, 1, "Down"}, {0, -1, "Up"}, {1, 0, "Right"}, {-1, 0, "Left"}
  }

  for _, d in ipairs(directions) do
    local nx = loc.x + d[1]
    local ny = loc.y + d[2]

    if nx >= 0 and nx < self.width and ny >= 0 and ny < self.height then
      local p = Location.new(nx, ny)
      local cost = self.mud_tiles[p:key()] and self.COST_MUD or self.COST_GRASS
      table.insert(moves, {d[3], p, cost})
    end
  end

  return moves
end

-- method: render
function TerrainMap:render(agent_loc, path_actions, frontier_nodes, closed_set, handle)
  local path_keys = {}

  if #path_actions > 0 then
    local curr = self.start
    for _, act in ipairs(path_actions) do
      if act == "Up" then curr = Location.new(curr.x, curr.y - 1)
      elseif act == "Down" then curr = Location.new(curr.x, curr.y + 1)
      elseif act == "Left" then curr = Location.new(curr.x - 1, curr.y)
      elseif act == "Right" then curr = Location.new(curr.x + 1, curr.y)
      end

      local m
      if self.mud_tiles[curr:key()] then
        if act == "Up" then m = " ⬆ "
        elseif act == "Down" then m = " ⬇ "
        elseif act == "Left" then m = " ⬅ "
        else m = " ➡ "
        end
      else
        if act == "Up" then m = " ↑ "
        elseif act == "Down" then m = " ↓ "
        elseif act == "Left" then m = " ← "
        else m = " → "
        end
      end

      if curr:key() ~= agent_loc:key() then
        path_keys[curr:key()] = m
      end
    end
  end

  local frontier_keys = {}
  for _, node in ipairs(frontier_nodes) do
    local pk = node.location:key()
    if pk ~= agent_loc:key() and not path_keys[pk] then
      frontier_keys[pk] = true
    end
  end

  handle:write("\n" .. string.rep("=", 30) .. "\n")
  handle:write("A* GRID STATE\n")
  handle:write(string.rep("-", 30) .. "\n")

  for y = 0, self.height - 1 do
    for x = 0, self.width - 1 do
      local p = Location.new(x, y)
      local k = p:key()

      if k == agent_loc:key() then
        handle:write(" ● ")
      elseif k == self.goal:key() then
        handle:write(" ★ ")
      elseif k == self.start:key() then
        handle:write(" ○ ")
      elseif path_keys[k] then
        handle:write(path_keys[k])
      elseif frontier_keys[k] then
        if self.mud_tiles[k] then
          handle:write(" M ")
        else
          handle:write(" F ")
        end
      elseif closed_set[k] then
        handle:write(" - ")
      elseif self.mud_tiles[k] then
        handle:write(" ~ ")
      else
        handle:write(" . ")
      end
    end
    handle:write("\n")
  end

  handle:write(string.rep("-", 30) .. "\n")
end

-- ==============================================================================
-- PART 3: Data Structures (Priority Queue)
-- ==============================================================================

-- Class: MinCostHeap
local MinCostHeap = {}
MinCostHeap.__index = MinCostHeap

function MinCostHeap.new(compare)
  local self = setmetatable({}, MinCostHeap)
  self.items = {}
  self.compare = compare
  return self
end

function MinCostHeap:push(item)
  table.insert(self.items, item)
  self:_sift_up(#self.items)
end

function MinCostHeap:pop()
  if self:is_empty() then return nil end
  local root = self.items[1]
  local last = table.remove(self.items)

  if not self:is_empty() then
    self.items[1] = last
    self:_sift_down(1)
  end
  return root
end

function MinCostHeap:is_empty()
  return #self.items == 0
end

function MinCostHeap:to_list()
  return self.items
end

function MinCostHeap:_sift_up(idx)
  while idx > 1 do
    local p = math.floor(idx / 2)
    if self.compare(self.items[idx], self.items[p]) < 0 then
      self.items[idx], self.items[p] = self.items[p], self.items[idx]
      idx = p
    else
      break
    end
  end
end

function MinCostHeap:_sift_down(idx)
  local count = #self.items
  while true do
    local l = 2 * idx
    local r = 2 * idx + 1
    local s = idx

    if l <= count and self.compare(self.items[l], self.items[s]) < 0 then
      s = l
    end
    if r <= count and self.compare(self.items[r], self.items[s]) < 0 then
      s = r
    end

    if s ~= idx then
      self.items[idx], self.items[s] = self.items[s], self.items[idx]
      idx = s
    else
      break
    end
  end
end

-- ==============================================================================
-- PART 4: MAIN LOGIC
-- ==============================================================================

-- sub: reconstruct_path
local function reconstruct_path(end_node)
  local path = {}
  local node = end_node
  while node.parent do
    table.insert(path, 1, node.action)
    node = node.parent
  end
  return path
end

-- sub: run_experiment
local function run_experiment(map, out_handle, options)
  local interactive = options.interactive
  local auto_play = options.auto_play or false
  local wait_time = options.wait_time or 1
  local show_menu = options.show_menu
  if show_menu == nil then show_menu = true end

  -- 1. Initialize start node and frontier
  local root = Node.new({
    location = map:initial_location(),
    h_cost = map:heuristic(map:initial_location())
  })

  local frontier = MinCostHeap.new(function(a, b) return a:f_cost() - b:f_cost() end)
  frontier:push(root)

  local best = {[map:initial_location():key()] = 0}
  local closed_set = {}
  local step_count = 0

  while not frontier:is_empty() do
    -- 2. Pick the best node
    local curr = frontier:pop()
    step_count = step_count + 1

    -- 3. Mark as closed
    closed_set[curr.location:key()] = true

    local terrain = map:is_mud(curr.location) and "(MUD)" or "(GRASS)"
    local act_text = curr.action and ("Moved " .. curr.action) or "Started"

    out_handle:write(string.format("\nSTEP %d:\n", step_count))
    out_handle:write(string.format("  CHOSEN: %s to %s %s\n", act_text, tostring(curr.location), terrain))
    out_handle:write(string.format("  COST DETAIL: f=%s (g=%s + h=%s)\n", curr:f_cost(), curr.g_cost, curr.h_cost))

    -- 4. Check for success
    if map:is_goal(curr.location) then
      local final_path = reconstruct_path(curr)
      map:render(curr.location, final_path, frontier:to_list(), closed_set, out_handle)

      local path_length = #final_path
      -- Count keys in closed_set for total explored
      local total_explored = 0
      for _ in pairs(closed_set) do total_explored = total_explored + 1 end

      local efficiency = (path_length / total_explored) * 100

      out_handle:write(string.format("\nSUCCESS! Goal reached at %s.\n", tostring(curr.location)))
      out_handle:write("--------------------------------------------------\n")
      out_handle:write("EXPLORATION DIAGNOSTICS:\n")
      out_handle:write(string.format("  Path Length:       %d steps\n", path_length))
      out_handle:write(string.format("  Total Explored:    %d points\n", total_explored))
      out_handle:write(string.format("  Search Efficiency: %.2f%%\n", efficiency))
      out_handle:write("--------------------------------------------------\n")
      return
    end

    -- 5. Expand neighbors
    local current_best = best[curr.location:key()]
    if not (current_best and curr.g_cost > current_best) then
      for _, t in ipairs(map:successors(curr.location)) do
        local action, loc, move_cost = t[1], t[2], t[3]
        local new_g = curr.g_cost + move_cost
        local key = loc:key()

        if not best[key] or new_g < best[key] then
          best[key] = new_g
          frontier:push(Node.new({
            location = loc,
            parent = curr,
            action = action,
            g_cost = new_g,
            h_cost = map:heuristic(loc)
          }))
        end
      end
    end

    -- 6. Visualization
    map:render(curr.location, reconstruct_path(curr), frontier:to_list(), closed_set, out_handle)
    out_handle:write(string.format("CURRENT LOCATION: %s %s\n", tostring(curr.location), terrain))

    if show_menu then
      local f_list = {}
      for _, n in ipairs(frontier:to_list()) do table.insert(f_list, n) end
      -- Sort for display purposes only
      table.sort(f_list, function(a, b) return a:f_cost() < b:f_cost() end)

      out_handle:write("  MENU FOR NEXT STEP (Current Frontier):\n")
      if #f_list > 0 then
        for i, n in ipairs(f_list) do
          local marker = (i == 1) and " -> " or "    "
          out_handle:write(string.format("%s %s: f=%s (g=%s + h=%s)\n",
            marker, tostring(n.location), n:f_cost(), n.g_cost, n.h_cost))
        end
      else
        out_handle:write("    (Empty)\n")
      end
    end

    -- 7. Pause
    if interactive then
      if auto_play then
        local start = os.clock()
        while os.clock() - start < wait_time do end
      else
        io.write("\n[Press ENTER for next step...]")
        io.read()
      end
    end
  end
end

-- --------------------------------------------------------------------------------

local function main(...)
  local args = {...}
  local arg_val = args[1]

  -- Setup Map
  local map = TerrainMap.new({
    width = 10,
    height = 10,
    start = Location.new(0, 0),
    goal = Location.new(9, 9)
  })

  if not arg_val then
    -- Default
    run_experiment(map, io.stdout, {interactive = true, auto_play = false, show_menu = true})
  elseif arg_val == "-" then
    -- Auto-play
    print("Auto-play mode (1.0s delay)...")
    run_experiment(map, io.stdout, {interactive = true, auto_play = true, wait_time = 1, show_menu = false})
  elseif arg_val:match("^%-%d+%.?%d*$") then
    -- Dash + Number
    local num = tonumber(arg_val:sub(2))
    print(string.format("Auto-play mode (%ss delay)...", num))
    run_experiment(map, io.stdout, {interactive = true, auto_play = true, wait_time = num, show_menu = false})
  else
    -- Filename
    local f = io.open(arg_val, "w")
    if f then
      print(string.format("Logging expansion to %s...", arg_val))
      run_experiment(map, f, {interactive = false, auto_play = false, show_menu = true})
      f:close()
      print("Done.")
    else
      print("Error opening file.")
    end
  end
end

main(...)

-- --------------------------------------------------------------------------------
-- the end
