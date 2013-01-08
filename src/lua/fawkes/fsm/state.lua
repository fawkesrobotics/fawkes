
------------------------------------------------------------------------
--  state.lua - Simple FSM state
--
--  Created: Sat Dec 27 16:25:31 2008 (copied from fsm.lua)
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

--- This module provides a generic FSM state.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

--- State
-- State for an FSM.
State = {}

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function State:new(o)
   assert(o, "State requires a table as argument")
   assert(o.name, "State requires a name")
   assert(o.fsm, "State " .. o.name .. " requires a FSM")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.dotattr = o.dotattr or {}
   o.transitions = o.transitions or {}
   assert(type(o.transitions) == "table", "State:new: transitions member must be of type table")

   o.last_trans = nil

   return o
end


--- Add a transition.
-- On this simple state adding transitions is informational only but strongly
-- recommended for automatic graph visualization of the FSM.
-- @param state state to add a transition to
-- @return added transition (table)
function State:add_transition(state)
   local transition = {state = state}
   table.insert(self.transitions, transition)
   return transition
end


--- Get the last activated transition.
-- This information needs to be recorded by the particular state implementation.
-- The information is required when tracing is used to be able to record transition
-- edges in the graph and not only nodes.
-- @return last executed transition
function State:last_transition()
   return self.last_trans
end


--- Add multiple transitions.
-- On this simple state adding transitions is informational only but strongly
-- recommended for automatic graph visualization of the FSM.
-- This is a convenience method to add a number of transitions at once.
-- @param ... states you want to add a transition to
function State:add_transitions(...)
   for _,s in ... do
      table.insert(self.transitions, {state = s})
   end
end


--- Clear transitions.
-- This removes all transitions from the transition table.
function State:clear_transitions()
   self.transitions = {}
end

--- Get transitions to a specified state.
-- This will return the transition to the given state. If there are multiple
-- transitions an error is thrown.
-- @param state state the queried transitions must point to
-- @return transitions that matches the given state
function State:get_transition(state)
   local transition = nil
   local to_state = type(state) == "table" and state.name or state
   for _,t in ipairs(self.transitions) do
      local to = type(t.state) == "table" and t.state.name or t.state
      if not state or to == to_state then
	 assert(not transition, "Two transition to state " .. to_state)
	 transition = t
      end
   end
   return transition
end


--- Get transitions to a specified state.
-- @param state state the queried transitions must point to
-- @return array of transitions that match the given state
function State:get_transitions(state)
   local transitions = {}
   local to_state = type(state) == "table" and state.name or state
   for _,t in ipairs(self.transitions) do
      if not state or t.state.name == to_state then
	 table.insert(transitions, t)
      end
   end
   return transitions
end


--- Execute init routines.
-- This executes the init method. This may contain some general code executed for
-- every state for derived states.
-- @param ... Any parameters, passed to the init() function.
function State:do_init(...)
   return self:init(...)
end


--- Execute exit routines.
-- This executes the exit method. This may contain some general code executed for
-- every state for derived states.
function State:do_exit()
   self:exit()
end


--- Execute loop routines.
-- This executes the exit method. This may contain some general code executed for
-- every state for derived states.
function State:do_loop()
   return self:loop()
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


--- Reset function.
-- This function is called by the FSM when the FSM is resetted.
function State:reset()
   self.last_trans = nil
end


--- Prepare the state.
-- This method is called once and only once in the FSMs and states lifetime.
function State:prepare()
end
