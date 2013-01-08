
------------------------------------------------------------------------
--  jumpstate.lua - FSM Jump state to build Hybrid State Machines (HSM)
--
--  Created: Thu Dec 04 10:40:54 2008
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

--- This module provides a state to create a Hybrid State Machine (HSM)
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)
local fsmmod = require("fawkes.fsm")
assert(fsmmod, "fsmmod is nil")
local State = fsmmod.State
assert(State, "State for JumpState is nil")


--- JumpState
-- Hybrid State Machine jump state for FSM.  This class provides a
-- generic state that makes a Finite State Machine a Hybrid State
-- Machine (iff only jump states are used). Jump states are states
-- that do not return a follow state in loop(), but rather set a
-- number of predefined transitions. Each transition has a jump
-- condition and the target state. On a call to do_loop() all jump
-- conditions are evaluated. If one matches this transition is
-- executed. If multiple could match then only the very first
-- transition is executed. The order is determined by the order the
-- transitions where added.
-- @author Tim Niemueller
JumpState = { comment           = "",
              clear_transitions = State.clear_transitions,
	      get_transition    = State.get_transition,
	      get_transitions   = State.get_transitions,
	      last_transition   = State.last_transition,
	      init              = State.init,
	      exit              = State.exit,
	      loop              = State.loop,
	      reset             = State.reset,
	      do_exit           = State.do_exit}

--- Create new jump state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function JumpState:new(o)
   assert(o, "JumpState requires a table as argument")
   assert(o.name, "JumpState requires a name")
   assert(o.fsm, "JumpState " .. o.name .. " requires a FSM")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.dotattr = o.dotattr or {}
   o.transitions = o.transitions or {}
   o.loops = o.loops or {}
   o.inits = o.inits or {}

   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")

   return o
end

--- Execute init routines.
-- This executes the init method. This may contain some general code
-- executed for every state for derived states. 
-- @param ... Any parameters, passed to the init() function.
function JumpState:do_init(...)
   local rv = { self:try_transitions(true) }
   if next(rv) then return unpack(rv) end
   self:init(...)
   for _,i in ipairs(self.inits) do
      i(self)
   end
   return self:try_transitions()
end


--- Execute jump state loop.
-- This will execute the loop and afterwards check if any jump condition
-- of the registered transitions holds. Iff one holds the checks are immediately
-- abort and the transition is executed.
-- @return new state if jump condition holds or false otherwise
function JumpState:do_loop()
   self:loop()
   for _,l in ipairs(self.loops) do
      l(self)
   end

   return self:try_transitions()
end


--- Create a new transition.
-- The transition is created, but not added.
-- @param state state to switch to if jumpcond holds
-- @param jumpcond jump condition function, see description above.
-- @param description a string representation of the jump condition, can
-- be a plain copy of the code as string or a verbal description, used for
-- debugging and graph generation
-- @param errmsg An optional error message which is added to the FSM errors if
-- transition is executed
function JumpState:create_transition(state, jumpcond, description, errmsg)
   assert(state, self.name .. ": Follow state is nil while adding '" .. tostring(description) .. "'")
   assert(state.name or type(state) == "string", self.name .. ": Follow state does not have a valid name while adding '" .. tostring(description) .. "'")
   assert(jumpcond, self.name .. ": Jump condition is nil while adding '" .. tostring(description) .. "'")
   --printf("%s: When '%s' -> %s (%s)", self.name, description, state.name, tostring(self.transitions))
   local jc
   if type(jumpcond) == "function" then
      jc = jumpcond
      description = description or ""
   elseif type(jumpcond) == "string" then
      jc = assert(loadstring("return " .. jumpcond),
		  self.name .. ": compiling jump condition '" .. jumpcond ..
		     "' failed")
      --local fe = getfenv(jc)
      local fe = { string=string, math=math, table=table,
		   os={time=os.time, date=os.date, clock=os.clock, difftime=os.difftime},
		   next=next, rawequal=rawequal, type=type,
		   state=self, self=self, vars=self.fsm.vars, fsm=self.fsm }
      if self.closure then
	 for k,v in pairs(self.closure) do fe[k] = v end
      end
      setfenv(jc, fe)
      description = description or jumpcond
   elseif type(jumpcond) == "boolean" then
      assert(jumpcond == true, self.name .. ": adding a jump condition for "..
	     "false does not make any sense, it would never fire")
      jc = JumpState.jumpcond_true
      description = description or "Unconditional"
   else
      error(self.name .. ": type of jump condition must be function, string or boolean")
   end

   return {state       = state,
	   jumpcond    = jc,
	   description = description,
	   error       = errmsg}
end

--- Add new transition.
-- Creates and adds a transition for this state. Jump conditions are executing
-- after loop() has run to check if a transition should happen. The jump condition
-- is a function whose first return value must be boolean. Iff the return value is
-- true then the condition holds/fires and the transition is executed. No further
-- jump conditions will be tried in that case. All (optional) following return
-- values are passed verbatim to the init() function of the state the transition
-- points to.
-- @param state state to switch to if jumpcond holds
-- @param jumpcond jump condition function, see description above.
-- @param description a string representation of the jump condition, can
-- be a plain copy of the code as string or a verbal description, used for
-- debugging and graph generation
-- @param errmsg An optional error message which is added to the FSM errors if
-- transition is executed
function JumpState:add_new_transition(state, jumpcond, description, errmsg)
   local transition = self:create_transition(state, jumpcond, description, errmsg)
   transition.post = true
   table.insert(self.transitions, transition)
   return transition
end

--- Add new precondition
-- Creates and adds a precondition for this state. Jump conditions are executing
-- before init() is run to check if a transition should happen.
-- @param state state to switch to if jumpcond holds
-- @param jumpcond jump condition function, see description above.
-- @param description a string representation of the jump condition, can
-- be a plain copy of the code as string or a verbal description, used for
-- debugging and graph generation
-- @param errmsg An optional error message which is added to the FSM errors if
-- transition is executed
function JumpState:add_new_precondition(state, jumpcond, description, errmsg)
   local transition = self:create_transition(state, jumpcond, description, errmsg)
   transition.pre = true
   table.insert(self.transitions, transition)
   return transition
end

--- Add transition as precondition.
-- Then the transition is added to the list of preconditions. That is it
-- is checked just before init() would be run. If any of the jump conditions of
-- the precondition transition fires the transition is executed immediately and
-- the state is never run (init() and loop() are not called.
-- @param transition transition to add
function JumpState:add_precondition(transition)
   transition.pre = true
   for _, t in ipairs(self.transitions) do
      if t == transition then return end
   end
   table.insert(self.transitions, transition)
end



--- Add existing transition.
-- The given transition must already exist in the list of transitions of this
-- state. Then the transition is added to the list of preconditions. That is it
-- is checked just before init() would be run. If any of the jump conditions of
-- the precondition transition fires the transition is executed immediately and
-- the state is never run (init() and loop() are not called.
-- @param transition transition to add
function JumpState:add_transition(transition, toomuch)
   assert(not toomuch, "JumpState:add_transition(): Passed too many arguments, "..
	  "need to use add_new_transition()?")
   transition.post = true
   for _, t in ipairs(self.transitions) do
      if t == transition then return end
   end
   table.insert(self.transitions, transition)
end




--- Prepare the state.
-- This method is called once and only once in the FSMs and states lifetime.
-- It is used for example to resolve forward declaration of states (when a string
-- with the states name was given instead of the state object). If that fails an
-- error is thrown.
function JumpState:prepare()
   for _,t in ipairs(self.transitions) do
      if type(t.state) == "string" then
	 local name = t.state
	 t.state = self.fsm.states[name]
	 assert(t.state and t.state.name, "JumpState.prepare: failed to resolve "..
		"forward declaration of state " .. name .. ", no such state in FSM")
      end
   end
end

--- Try all transitions and return a follow state if applicable.
-- This tries for all added transitions if the jump condition fires. If it does
-- the follow state is returned with any parameters the jump condition might have
-- supplied.
-- @return follow state or nil to stay in current state as first argument, possibly
-- any number of additional arguments that should be passed to the follow state
function JumpState:try_transitions(precond)
   local transtable = transtable or self.transitions
   --print("Trying conditions for " .. self.name)
   for _,t in ipairs(transtable) do
      if t.pre and precond or t.post and not precond then
	 if not t.jumpcond then
	    if not t.state then
	       print_error("Transition %s-> ? does have neither jump condition nor target state",
			   self.name);
	    else
	       print_error("Transition %s -> %s (%s) does not have a valid jump condition",
			   self.name, t.state.name, t.description)
	    end
	 else
	    local rv = { t.jumpcond(self) }
	    local jcfires = rv[1]
	    table.remove(rv, 1)
	    if jcfires then
	       if self.fsm then
		  if self.fsm.debug then
		     print("Jump condition '" .. tostring(t.description) .. "' FIRES, returning " .. t.state.name)
		  end
		  if t.error then
		     local sep = ""
		     if self.fsm.error ~= "" then sep = ", " end
		     self.fsm.error = self.fsm.error .. sep .. t.error
		  end
	       end
	       self.last_trans = t
	       return t.state, unpack(rv)
	    end
	 end
      end
   end

   -- Remain in current state, no jump condition fired
   return nil
end


--- Jump condition that returns always true.
-- This is a convenience method that avoid having to write such a function every
-- time it is needed (for example when using and explicit start state that only
-- transitions to another state or when just sending a BlackBoard message and then
-- advancing to a wait state).
-- @return true
function JumpState:jumpcond_true()
   return true
end

--- Fires on timeout.
-- @return true if the time ran out
function JumpState:jumpcond_timeout()
   return os.difftime(os.time(), self.timeout_start) >= self.timeout_time
end

--- Initializes timeout value.
function JumpState:init_timeout()
   self.timeout_start = os.time()
end


--- Setup timeout transition.
-- @param to state to go to if the timeout expires
-- @param time in seconds
function JumpState:set_timeout(time, to, errmsg)
   assert(time, "No timeout value given")
   assert(type(time) == "number", "Timeout value must be a number")
   assert(to, "No timeout target state given")
   self.timeout_time = time
   self.timeout_to   = to
	 
   table.insert(self.inits, self.init_timeout)
   self.timeout_transition = self:add_new_transition(to, self.jumpcond_timeout, "Timeout ("..time.." sec)", errmsg)
   self.timeout_transition.dotattr = { style = "dashed" }
end


--- Checks a number of interfaces for no writer.
-- This jump condition can be used to check a number of interfaces for a
-- non-existent writer. For this to work you need to set the nowriter_interfaces
-- member variable of the state to an array of interfaces to check.
-- @return true if any of the interfaces in the nowriter_interfaces table does not
-- have a writer, false if no interfaces given or all interfaces have a writer
function JumpState:jumpcond_nowriter()
   if self.nowriter_interfaces and type(self.nowriter_interfaces) == "table" then
      for _,i in ipairs(self.nowriter_interfaces) do
	 if not i:has_writer() then
	    return true
	 end
      end
   else
      printf("nowriter fail")
   end
   return false
end


--- Check if given object is an instance of FSM class.
-- @return true if obj is an instance of FSM class
function JumpState.is_instance(obj)
   local mt = getmetatable(obj)
   while mt do
      if mt == JumpState then return true end
      mt = getmetatable(mt)
   end
   return false
end
