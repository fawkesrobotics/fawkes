
------------------------------------------------------------------------
--  hsm.lua - Lua Hybrid State Machines (HSM)
--
--  Created: Wed Aug 25 13:55:11 2010
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--             2010  Carnegie Mellon University
--             2010  Intel Labs Pittsburgh
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

--- This module provides a Hybrid State Machine (HSM).
-- The HSM class in this module implements a subset of HSM functionality.
-- The most notable difference from the generic FSM driver is that states
-- and transitions are explicitly defined and individually represented
-- (in the generic driver this is not the case for transitions, they are
-- given only implicitly).
-- <br /><br />
-- To use this class, instantiate the HSM and call HSM:define_states() to
-- create all the states of the state machine. Afterwards, call
-- HSM:add_transitions() to add transitions among the states.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

local jumpstmod  = require("fawkes.fsm.jumpstate")
JumpState        = jumpstmod.JumpState
local fsmmod     = require("fawkes.fsm")
FSM              = fsmmod.FSM

HSM = {}

--- Create a new HSM.
-- Parameters are the same as for FSM.
-- @see FSM:new()
function HSM:new(o)
   local o = FSM:new(o)

   setmetatable(o, self)
   setmetatable(self, FSM)
   self.__index = self

   o.default_transitions = {}

   return o
end

--- Simple state generation not supported for SkillHSM.
-- Throws an error. Only jump states can be created for SkillHSMs.
function HSM:new_state()
   error("Only jump states can be created for a HSM")
end

--- Define states of this HSM.
-- This method takes a table describing the states that this state machines
-- shall have.
-- @param table that defines the states. It can have two named entries:
-- <ul>
-- <li>export_to: Define a table to which the states will be added. You can for
-- example pass _G to have the states exported as global variables.</li>
-- <li>closure Closure used for jump conditions, e.g. variables you want to make
-- available to jump conditions which are given as strings and compiled into
-- functions. By default only the state itself, the variables, the FSM, and
-- some selected functions are available.</li>
-- </ul>
-- <br />
-- The table then contains state definitions as a list. Each entry in the list
-- is either a string or a table. If it is a string, the state will be created
-- as instance of JumpState. If it is a table, the table must have two
-- positional entries. The first must be a string with the name of the state,
-- the second must be the class of the state. Note that the latter is not a
-- string, but rather the table of the class definition. The table can also
-- contain any number of named arguments, which are passed verbatim to the
-- constructor of the class when instantiating the state. The fields name, fsm,
-- and closure are overwritten if they exist.
-- <br /><br />
-- Example:
-- <code>
-- local hsm = HSM:new{name="Example", start="MY_START"}
-- hsm:define_states{
--   export_to=_G,
--   closure={my_var="closure demo"},
--   "MY_START", {"MY_END", JumpState}
-- }
-- </code>
function HSM:define_states(states)
   local export_to = states.export_to
   local closure   = states.closure

   -- export already existing states
   if export_to then
      for name, state in pairs(self.states) do
	 export_to[name] = state
      end
   end

   for _, s in ipairs(states) do
      local name, class
      if type(s) == "string" then
	 name = s
	 class = JumpState
      else
	 name  = s[1]
	 class = s[2]
      end
      assert(not self.states[name], "HSM[" .. self.name .. "] state with name "
	     .. tostring(name) .. " already defined")
      assert(class, "HSM[" .. self.name .. "] class instance for state "
	     .. tostring(name) .. " not defined")

      -- Generate base table for new instance
      local o = {}
      -- copy the values, otherwise instances would use the definition table
      if type(s) == "table" then
	 for k, v in pairs(s) do o[k] = v end
	 table.remove(o, 1, 2) -- remove positional args
      end
      o.name    = name
      o.fsm     = self
      o.closure = closure

      -- Create state and assert as subclass
      local s = class:new(o)
      assert(JumpState.is_instance(s), "State class is not a sub-class of JumpState")

      -- add to structures and possibly export
      self.states[name] = s
      self:apply_deftrans(s)
      if export_to then export_to[s.name] = s end
   end
end


--- Define transitions.
-- This method takes a table of transitions to add. All states used for
-- transitions must have been defined using HSM:define_states().
-- @param trans list of transitions. Each transition is defined as a table.
-- There are two forms. For normal transitions at least two entries positional
-- entries are defined. First the originating state of the transition, and
-- second the destination state. Either a third positional argument or the
-- cond field of the table are set to the jump condition. The jump condition
-- can either be a string, which is compiled into a function and prepended
-- with "return", e.g. the expression must evaluate to a boolean value. In
-- the string you may use the variables vars (FSM variables table), fsm (the
-- FSM the originating state is assigned to), and self (the originating state).
-- If the condition is not a string it must be a boolean function getting the
-- originating state as its only parameter.
-- <br /><br />
-- A timeout field can be set. It is either defined as the number of seconds to
-- advance to the destination state after entering the originating state, or as
-- a table with two entries, the first holding the timeout in seconds, and the
-- second the name of the state to advance to if the timeout runs out.
-- <br /><br />
-- Instead of setting the condition in the cond field, one can set it in the
-- precond field to treat it additionally as a precondition, or set in in the
-- precond_only field, to treat it exclusively as a precondition. Preconditions
-- cause checknig of the jump condition already just before entering the state
-- (and before executing its init() method).
-- <br /><br />
-- A dotattr string field can be set with dot graph attributes for rendering.
-- A desc string field can be set to an alternative description of the
-- transition used in the graph display.
function HSM:add_transitions(trans)
   for _,t in ipairs(trans) do
      if t[2] then -- Normal from -> to transition
	 assert(t[1], "Must have an originating state")
	 assert(not (t[3] and t.cond), "Only one of cond field and third index may be set as condition")

	 local trans_string = tostring(t[1]) .." -> " .. tostring(t[2])

	 local from  = assert(self.states[t[1]],
			      "Originating state does not exist for transition "
				 .. trans_string)
	 local to    = assert(self.states[t[2]],
			      "Destination state does not exist for transition "
				 .. trans_string)
	 local cond  = t[3] or t.cond or t.precond or t.precond_only
	 assert(cond or t.timeout,
		"You must have a condition or a timeout for transition "
		   .. trans_string)

	 -- If we only get a time as timeout assume jump to normal to state
	 if t.timeout then
	    local timeout_to, timeout_time, timeout_err
	    if type(t.timeout) == "number" then
	       timeout_time = t.timeout
	       timeout_to   = to.name
	    else
	       assert(type(t.timeout) == "table", "Timeout must be number or table")
	       timeout_time = t.timeout.time or t.timeout[1]
	       timeout_to   = t.timeout.to   or t.timeout[2] or to.name
	       timeout_err  = t.timeout.error
	    end
	    assert(self.states[timeout_to], "Timeout destination state "
		   .. tostring(timeout_to)
		   .. " does not exist for transition " .. trans_string)

	    from:set_timeout(timeout_time, timeout_to, timeout_err)
	 end

	 -- We might have no condition but still a useful transition,
	 -- i.e. if a timeout is set
	 if cond then
	    local new_t
	    if t.precond_only then
	       new_t = from:add_new_precondition(to, cond, t.desc, t.error)
	    else
	       new_t = from:add_new_transition(to, cond, t.desc, t.error)
	       if t.precond then from:add_precondition(new_t) end
	    end
	    if t.dotattr then new_t.dotattr = t.dotattr end
	    if t.hide then new_t.hide = true end
	 end

      else -- default transition
	 local to = assert(self.states[t[1]], "Destination state " .. tostring(t[1])
			   .. " does not exist for default transition")
	 assert(t.cond, "Default transition must have a jump condition")
	 self:add_default_transition(to.name, t.cond, t.desc)
      end
   end
end


--- Convenience method to create a new jump state.
-- It is recommended NOT to use this method, but rather use define_states() and
-- add_transitions().
-- <br />
-- Creates a new instance of JumpState and assigns it to the states table with
-- the given initial transitions. Any variables that exist in the (optional) vars
-- table are put into state as local variables.
-- If FSM.export_states_to_parent is true the state is exported to the callers
-- environment by assigning it to a variable with the states name.
-- @param name name of the state and optionally the variable in the environment
-- of the caller that holds this state
-- @param transitions Initial transitions for this state (may be modified or set
-- later)
-- @param vars all values from this table will be added to the newly created state.
-- @param export_to if passed a table this method will assign the state to a field
-- with the name of the state in that table.
function HSM:new_jump_state(name, transitions, vars, export_to)
   assert(name, "HSM:new_jump_state: Name must be given")
   assert(self.states[name] == nil, "HSM:new_state: State with name " .. name .. " already exists")
   local o = {name = name, fsm = self, transitions = transitions}
   if vars then
      assert(type(vars) == "table", "HSM:new_state: vars parameter must be a table")
      for k,v in pairs(vars) do
	 o[k] = v
      end
   end
   local s = JumpState:new(o)
   self:apply_deftrans(s)
   self.states[name] = s
   if export_to then e[name] = s end

   return s
end


--- Add default transition.
-- A default transition is a transition that is added to every added state. Here,
-- the transition is a HSM transition consisting of a target state, a jump
-- condition and an optional description.
-- Any state already added gets a new transition, as will any state added later.
-- @param state state to switch to if jumpcond holds
-- @param jumpcond jump condition function, see description above.
-- @param description a string representation of the jump condition, can
-- be a plain copy of the code as string or a verbal description, used for
-- debugging and graph generation
function HSM:add_default_transition(state, jumpcond, description)
   if self.debug then
      printf("%s: Adding default transition -> %s on %s (%s)",
	     self.name, tostring(state), tostring(jumpcond), tostring(description))
   end
   table.insert(self.default_transitions, {state=state, jumpcond=jumpcond, description=description})
   for _,st in pairs(self.states) do
      self:apply_deftrans(st)
   end
end


--- Apply default transitions to given state.
-- @param state state to assign default transitions to
function HSM:apply_deftrans(state)
   assert(type(state) == "table" and state.name, "Passed state must be a state object")

   for i,t in ipairs(self.default_transitions) do
      local compto = type(t.state) == "table" and state or state.name

      if compto ~= t.state
	 and state.name ~= self.exit_state and state.name ~= self.fail_state then

	 local exists = false
	 for _,t2 in ipairs(state.transitions) do
	    if t2.deftransindex == i then
	       exists = true
	       break;
	    end
	 end
	 if not exists then
	    printf("Adding transition %s -> %s (%s, %s)", state.name, tostring(t.state),
		   tostring(t.jumpcond), tostring(t.description))
	    local tr = state:add_new_transition(t.state, t.jumpcond, t.description)
	    tr.deftransindex = i
	 end
      end
   end
end
