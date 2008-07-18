
------------------------------------------------------------------------
--  fsm.lua - Lua Finite State Machines (FSM)
--
--  Created: Fri Jun 13 11:25:36 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
--  $Id$
--
------------------------------------------------------------------------

--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Library General Public License for more details.
--
--  Read the full text in the LICENSE.GPL file in the doc directory.

require("fawkes.modinit")

--- This module provides a generic finite state machine (FSM).
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

--- @class State
-- State for an FSM.
State = {}

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function State:new(o)
   assert(o, "State requires a table as argument")
   assert(o.name, "State requires a name")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self
   return o
end

--- Init function.
-- The init function is called when you enter a state. It may take an arbitrary
-- number of arguments, which may be passed from other states.
function State:init()
end


--- Loop function.
-- This function is called in every loop. You can for example check conditions
-- for state changes.
-- @return return nil to stay in this state, or another state to change the state
-- to this state. Additionally you can return any number of arguments (only if
-- the returned state is not nil). These arguments are passed to the init()
-- function of the state we transit to.
function State:loop()
end


--- Exit function.
-- This function is called by the FSM just before the state is left. If any special
-- operations are necessary (like retracting an arm) for a safe transition out of
-- this state add them here.
function State:exit()
end


--- Convenience method to create a new state.
-- Creates a new state in the environment of the caller of this function with the
-- given name.
-- @param name name of the state and the variable in the environment of the caller
-- that holds this state
-- @param init_table all values from this table will be added to the newly created
-- state.
function new_state(name, init_table)
   local e = getfenv(2)
   local s = State:new{name = name}
   if init_table then
      for k,v in pairs(init_table) do
	 s[k] = v
      end
   end
   e[name] = s
end

--- @class FSM Finite State Machine
-- Representation with utility methods of a FSM.
FSM = { current = nil, debug = false }

--- Constructor.
-- Create a new FSM with the given table as the base.
-- @param o table with initializations. The table may not have a metatable set and
-- it must have the fields name (name of the FSM) and start (start state of the FSM).
-- @return initialized FSM
function FSM:new(o)
   assert(o, "FSM requires a table as argument")
   assert(o.name, "FSM requires a name")
   assert(o.start, "FSM requires a start state")
   assert(type(o.debug) == "boolean", "FSM debug value must be a boolean")
   o = o or {}
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   if o.debug then
      print_debug(o.name .. ": Initialize start state " .. o.start.name)
   end
   o.current = o.start
   o.current:init()

   return o
end

--- Run the FSM.
-- Runs the loop() function of the current state and executes possible state
-- transitions that result from running the loop.
function FSM:loop()
   if self.current then
      -- if self.debug then
      --    print_debug(self.name .. ": Executing loop for state " .. self.current.name)
      -- end
      self:trans(self.current:loop())
   end
end

--- Execute state transition.
-- If next is not nil, it calls the exit() function of the current state, and
-- then calls the init() function of the next state with any given additional
-- parameters. After that, the current state becomes next.
-- @param next next state, if nil no transition is executed
-- @param ... any number of extra arguments, passed to next:init().
function FSM:trans(next, ...)
   if next then
      if self.debug then
	 print_debug(self.name .. ": Transition", self.current.name, "->", next.name)
      end
      self.current:exit()
      local init_rv = {next:init(...)}
      self.current = next
      if #init_rv > 0 then
	 return self:trans(unpack(init_rv))
      end
   else
      if ... then
	 error("State " .. self.current.name .. " returned parameters but " ..
		    "not a state transition (nil)")
      end
   end
end
