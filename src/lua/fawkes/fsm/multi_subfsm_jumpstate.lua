
------------------------------------------------------------------------
--  multi_subfsm_jumpstate.lua - HSM state to execute multiple Sub-FSMs
--
--  Created: Fri Sep 10 11:52:46 2010
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

--- MultiSubFSMJumpState
-- This special jump state allows for executing multiple FSM/HSM while
-- the state is active. It can execute transition based on the state of
-- the sub-FSMs. It makes some assumptions. The sub-FSMs must all have
-- designated exit and fail states. If any of the sub-FSMs is in a fail
-- state, the execution is considered a failure. If all sub-FSMs are
-- in their exit state, the execution is considered successful. It is
-- "in progress" otherwise.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

require("fawkes.fsm.jumpstate")

-- Convenience shortcuts
local JumpState     = fawkes.fsm.jumpstate.JumpState


MultiSubFSMJumpState = {}


--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function MultiSubFSMJumpState:new(o)
   assert(o, "MultiSubFSMJumpState requires a table as argument")
   assert(o.name, "MultiSubFSMJumpState requires a name")
   assert(o.fsm, "MultiSubFSMJumpState " .. o.name .. " must be assigned to a FSM")
   assert(o.subfsms and #o.subfsms > 0, "MultiSubFSMJumpState " .. o.name .. " requires sub-FSMs")
   assert(not getmetatable(o), "Meta table already set for MultiSubFSMJumpState " .. o.name)
   setmetatable(o, self)
   setmetatable(self, JumpState)
   self.__index = self

   o.transitions   = o.transitions or {}
   o.dotattr       = o.dotattr or {}
   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")
   assert(type(o.dotattr) == "table", "Dot attributes for " .. o.name .. " not a table")

   local names = {}
   for _, s in ipairs(o.subfsms) do
      assert(s.exit_state, "Sub-FSM " .. s.name .. " does not define an exit state")
      assert(s.fail_state, "Sub-FSM " .. s.name .. " does not define a fail state")
      table.insert(names, s.name)
   end
   --o.dotattr.label = o.name .. "\\n" .. table.concat(names, " | ")

   o.final_transition   = o:add_new_transition(o.exit_to, o.final, "FSMs succeeded")
   o.failure_transition = o:add_new_transition(o.fail_to, o.failed, "FSM failed")

   return o
end

--- Check if sub-FSM succeeded.
-- @return true if the current state of the sub-FSM is the final state
function MultiSubFSMJumpState:final()
   for _,s in ipairs(self.subfsms) do
      if s.current.name ~= s.exit_state then
	 return false
      end
   end
   return true
end

--- Check if sub-FSM failed.
-- @return true if the current state of the sub-FSM is the failure state
function MultiSubFSMJumpState:failed()
   for _,s in ipairs(self.subfsms) do
      if s.current.name == s.fail_state then
	 return true
      end
   end
   return false
end

--- Init SubFSM State. Note, that this will only cause transitions for
-- preconditions, but not for regular transitions. This is done because the
-- sub-FSM hasn't been run, yet.
function MultiSubFSMJumpState:do_init(...)
   local rv = { self:try_transitions(true) }
   if next(rv) then return unpack(rv) end
   for _,s in ipairs(self.subfsms) do
      s:reset()
   end

   self:init(...)

   for i,s in ipairs(self.subfsms) do
      if self.fsm.vars[s.name] then
	 for k, v in pairs(self.fsm.vars[s.name]) do s.vars[k] = v end
      else
	 print_debug("MultiSubFSM[%s]: no arguments for %s", self.name, s.name)
      end
   end
end

--- Execute loop.
function MultiSubFSMJumpState:do_loop()
   self:loop()
   local changed = false
   for _,s in ipairs(self.subfsms) do
      s:loop()
      if s:changed() then changed = true end
   end
   self.fsm:set_changed(changed)
   return self:try_transitions()
end

--- Resets the sub-FSM.
function MultiSubFSMJumpState:do_exit()
   JumpState.do_exit(self)
   local errors = {}
   for _,s in ipairs(self.subfsms) do
      local error = s.error
      if error and error ~= "" then
	 table.insert(errors, s.name .. ": ".. error)
	 print_warn("MultiSubFSMJumpState[%s]: %s failed (error: %s)",
		    self.name, s.name, error)
      end
   end

   if self.fsm.error and self.fsm.error ~= "" then
      self.fsm.error = self.fsm.error .. " ("..table.concat(errors, "|")..")"
   else
      self.fsm.error = table.concat(errors, "|")
   end
end

-- Resets the sub-FSM.
function MultiSubFSMJumpState:reset()
   JumpState.reset(self)
   for _,s in ipairs(self.subfsms) do
      s:reset()
   end
end


--- Get string representation.
-- @return string of the form MultiSubFSMJumpState[FSM/State]@Current, where FSM
-- will be replaced by the FSM's name, State by this state's name and Current
-- by the name of the current state of the sub-FSM.
function MultiSubFSMJumpState:__tostring()
   local states = {}
   for _,s in ipairs(self.subfsms) do
      table.insert(states, s.current.name)
   end
   return string.format("MultiSubFSMJumpState[%s/%s]@(%s)", self.fsm.name, self.name, table.concat(states, "|"))
end
