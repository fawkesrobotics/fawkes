
------------------------------------------------------------------------
--  fsm.lua - Lua Finite State Machines (FSM)
--
--  Created: Fri Jun 13 11:25:36 2008
--  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
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

-- Re-export State and JumpState for convenience
local statemod   = require("fawkes.fsm.state")
State            = statemod.State
local jumpstmod  = require("fawkes.fsm.jumpstate")
JumpState        = jumpstmod.JumpState

local fsmgrapher = require("fawkes.fsm.grapher")


--- FSM Finite State Machine
-- Representation with utility methods of a FSM.
FSM = { current = nil, debug = false, export_states_to_parent = true }


--- Constructor.
-- Create a new FSM with the given table as the base.
-- @param o table with initializations. The table may not have a metatable set and
-- it must have the fields name (name of the FSM) and start (name of start state
-- of the FSM).
-- @return initialized FSM
function FSM:new(o)
   assert(o, "FSM requires a table as argument")
   assert(o.name, "FSM requires a name")
   assert(o.start, "FSM requires start state")
   if o.debug then
     assert(type(o.debug) == "boolean", "FSM debug value must be a boolean")
   end
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.recursion_limit     = 10
   o.current_recursion   = 0
   o.persistent_vars     = {}
   o.vars                = {}
   o.states              = {}
   o.state_changed       = false
   o.trace               = {}
   o.tracing             = true
   o.error               = ""
   o.exit_state          = o.exit_state
   o.fail_state          = o.fail_state
   o.prepared            = false
   o.max_num_traces      = 40

   return o
end


--- Check if state was reached in current trace.
-- @param state state to check if a transition originated from or went to this one
-- @return two values, first is true if state is included in
-- current trace, false otherwise, second is a list of traces where the
-- given state was involved. Note that the indexes of the traces in
-- the list are not consecutive!
function FSM:traced_state(state)
   if not self.tracing then return false end

   local traces = {}

   for i,t in ipairs(self.trace) do
      if t.from == state or t.to == state then
	 traces[i] = t
      end
   end

   if next(traces) then
      return true, traces
   else
      return false, {}
   end
end


--- Check if the given transition was used in current trace.
-- @param trans transition to check
-- @return two values, first is true if transition is included in
-- current trace, false otherwise, second is a list of traces where the
-- given transition was involved. Note that the indexes of the traces in
-- the list are not consecutive!
function FSM:traced_trans(trans)
   if not self.tracing then return false end

   local traces = {}

   for i,t in ipairs(self.trace) do
      if t.transition == trans then
	 traces[i] = t
      end
   end

   if next(traces) then
      return true, traces
   else
      return false, {}
   end
end


--- Check if the given state or transition was used in current trace.
-- @param sot state or transition to check, if a string assumed to be a state name
-- @return two values, first is true if state/transition is included in
-- current trace, false otherwise, second is a list of traces where the
-- given state/transition was involved. Note that the indexes of the traces in
-- the list are not consecutive!
function FSM:traced(sot)
   if not self.tracing then return false end

   local traces = {}
   if type(sot) == "string" then
      sot = self.states[sot]
   end

   for i,t in ipairs(self.trace) do
      if t.from == sot or t.transition == sot then
	 traces[i] = t
      end
   end

   if next(traces) then
      return true, traces
   else
      return false, {}
   end
end

--- Get graph representation of FSM.
-- This creates a graph representation of this FSM using the graphviz library.
-- The graph is returned as a string in the dot graph language.
-- @return graph representation as string in the dot graph language
function FSM:graph()
   return fsmgrapher.dotgraph(self)
end


--- Check if the FSM changed since last call.
-- The FSM has changed if there has been a transition to a new state since the
-- last call to reset() or changed(). Note that changed() will only ever return
-- true at most once per cycle as it resets the changed flag during execution.
-- @return true if the FSM has changed, false otherwise
function FSM:changed()
   local rv = self.state_changed
   self.state_changed = false
   return rv
end


--- Mark FSM as changed.
-- This can be used to cause a redrawing of the FSM graph if a state changed its
-- label for example.
function FSM:mark_changed()
   self.state_changed = true
end


--- Set change state of FSM.
-- @param changed true to mark FSM as changed, false otherwise
function FSM:set_changed(changed)
   if changed ~= nil then
      self.state_changed = changed
   else
      self.state_changed = true
   end
end

--- Convenience method to create a new state.
-- Creates a new instance of State and assigns it to the states table. Any variables
-- that exist in the (optional) vars table are put into state as local variables.
-- @param name name of the state and optionally the variable in the environment
-- of the caller that holds this state
-- @param vars all values from this table will be added to the newly created state.
-- @param export_to if passed a table this method will assign the state to a field
-- with the name of the state in that table.
function FSM:new_state(name, vars, export_to)
   assert(self.states[name] == nil, "FSM:new_state: State with name " .. name .. " already exists")
   local o = {name = name, fsm = self}
   if vars then
      assert(type(vars) == "table", "FSM:new_state: vars parameter must be a table")
      for k,v in pairs(vars) do
	 o[k] = v
      end
   end
   local s = State:new(o)
   self.states[name] = s

   if export_to then export_to[name] = s end

   return s
end


--- Enable or disable debugging.
-- Value can be either set for an instance of SkillHSM, or globally for SkillHSM
-- to set the new default value which applies to newly generated SkillHSMs.
-- @param debug true to enable debugging, false to disable
function FSM:set_debug(debug)
   self.debug = debug
end


--- Set error of FSM.
-- Sets the error for the FSM and prints it as warning.
-- @param error error message
function FSM:set_error(error)
   self.error = error
   if self.debug and error ~= nil and error ~= "" then
      print_warn("FSM %s: %s", self.name, self.error)
   end
end


--- Remove all states.
function FSM:clear_states()
   self.states = {}
   self.state_changed = true
end


--- Run the FSM.
-- Runs the loop() function of the current state and executes possible state
-- transitions that result from running the loop.
function FSM:loop()
   self.current_recursion = 0
   if not self.current then
      -- FSM was just reset, initialize start state
      self.current = self.states[self.start]
      assert(self.current, "Start state " .. tostring(self.start) .. " does not exist")
      self:trans(self.current:do_init())
      self.state_changed = true
   end
   if self.current.name ~= self.fail_state then
      -- This is really noisy...
      -- if self.debug then
      --    print_debug(self.name .. ": Executing loop for state " .. self.current.name)
      -- end
      self:trans(self.current:do_loop())
   elseif self.error == "" and self.previous then
      local t = self.previous:last_transition()
      if t and t.description then
	 self:set_error(t.description)
      else
	 self:set_error("reached fail state " .. self.fail_state .. " via unknown transition");
      end
   end
end


--- Execute state transition.
-- If next_state is not nil, it calls the exit() function of the current state, and
-- then calls the init() function of the next state with any given additional
-- parameters. After that, the current state becomes next.
-- @param next_state next state, if nil no transition is executed
-- @param ... any number of extra arguments, passed to next:init().
function FSM:trans(next_state, ...)
   if next_state then
      if self.tracing then
	 local trans = self.current:last_transition()
	 if #self.trace > self.max_num_traces then
	    table.remove(self.trace)
	 end
	 table.insert(self.trace, {from = self.current, transition = trans, to = next_state})
      end

      -- Some sanity check to avoid infinite loops
      self.current_recursion = self.current_recursion + 1
      assert(self.current_recursion < self.recursion_limit,
	     "Recursion limit (" .. tostring(self.recursion_limit) ..
	     ") reached, infinite loop?")

      self.state_changed = true
      if self.debug then
	 print_debug("%s: Transition %s -> %s",
		     self.name, self.current.name, next_state.name)
      end
      self.current:do_exit()
      self.previous = self.current
      self.current  = next_state
      return self:trans(next_state:do_init(...))
   end
end


--- Resets the internal trace.
-- Tracing information will be discarded. This can be used to restart the
-- trace without causing a whole FSM reset.
function FSM:reset_trace()
   self.trace = {}
   self.state_changed = true
end

--- Reset FSM.
-- Unsets the current state, if there was one the exit() routine is called. All
-- states of this FSM are reset, traces and variables are reset, FSM is marked as
-- changed. The next call to loop() will initialize the start state and set it as
-- current state.
function FSM:reset()
   if self.current then
      self.current:do_exit()
   end

   for k,_ in pairs(self.vars) do
      self.vars[k] = nil
   end
   -- Do not do the following, it breaks jump conditions that are passed as
   -- Lua string
   --self.vars          = {}

   if not self.prepared then
      for n,s in pairs(self.states) do
	 s:prepare()
      end
      self.prepared = true
   end
   for n,s in pairs(self.states) do
      s:reset()
   end

   self.trace         = {}
   self.current       = nil
   self.state_changed = true
   self.error         = ""

   if self.debug then
      print_debug("FSM '%s' reset done", self.name)
   end
end

--- Check if given object is an instance of FSM class.
-- @return true if obj is an instance of FSM class
function FSM.is_instance(obj)
   local mt = getmetatable(obj)
   while mt do
      if mt == FSM then return true end
      mt = getmetatable(mt)
   end
   return false
end
