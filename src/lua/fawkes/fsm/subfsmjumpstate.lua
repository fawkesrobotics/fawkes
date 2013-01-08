
------------------------------------------------------------------------
--  subfsmjumpstate.lua - HSM state to execute Sub-FSMs
--
--  Created: Fri Mar 20 11:12:11 2009
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

--- Jump states to build Hybrid State Machines (HSM) for skills.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

require("fawkes.fsm.jumpstate")

-- Convenience shortcuts
local JumpState     = fawkes.fsm.jumpstate.JumpState


--- SubFSMJumpState
-- This special jump state allows for executing another FSM/HSM while the state
-- is active. It can execute transition based on the state of the sub-FSM.
-- @author Tim Niemueller
SubFSMJumpState = {}


--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SubFSMJumpState:new(o)
   assert(o, "SubFSMJumpState requires a table as argument")
   assert(o.name, "SubFSMJumpState requires a name")
   assert(o.fsm, "SubFSMJumpState " .. o.name .. " must be assigned to a FSM")
   assert(o.subfsm, "SubFSMJumpState " .. o.name .. " requires a sub-FSM")
   assert(not getmetatable(o), "Meta table already set for SubFSMJumpState " .. o.name)
   setmetatable(o, self)
   setmetatable(self, JumpState)
   self.__index = self

   o.transitions   = o.transitions or {}
   o.dotattr       = o.dotattr or {}
   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")
   assert(type(o.dotattr) == "table", "Dot attributes for " .. o.name .. " not a table")

   if o.subfsm.exit_state and o.exit_to then
      o.final_transition = o:add_new_transition(o.exit_to, o.jumpcond_fsm_done, "FSM succeeded")
   end
   if o.subfsm.fail_state and o.fail_to then
      o.failure_transition = o:add_new_transition(o.fail_to, o.jumpcond_fsm_failed, "FSM failed")
   end

   return o
end

--- Check if sub-FSM succeeded.
-- @return true if the current state of the sub-FSM is the final state
function SubFSMJumpState:jumpcond_fsm_done()
   return self.subfsm.current.name == self.subfsm.exit_state
end

--- Check if sub-FSM failed.
-- @return true if the current state of the sub-FSM is the failure state
function SubFSMJumpState:jumpcond_fsm_failed()
   return self.subfsm.current.name == self.subfsm.fail_state
end

--- Init SubFSM State. Note, that this will only cause transitions for
-- preconditions, but not for regular transitions. This is done because the
-- sub-FSM hasn't been run, yet.
function SubFSMJumpState:do_init(...)
   local rv = { self:try_transitions(true) }
   if next(rv) then return unpack(rv) end
   self.subfsm:reset()
   self:init(...)
   for k, v in pairs(self.fsm.vars) do self.subfsm.vars[k] = v end
end

--- Execute loop.
function SubFSMJumpState:do_loop()
   self:loop()
   self.subfsm:loop()
   self.fsm:set_changed(self.subfsm:changed())
   return self:try_transitions()
end

--- Resets the sub-FSM.
function SubFSMJumpState:do_exit()
   JumpState.do_exit(self)
   if self.subfsm.error and self.subfsm.error ~= "" then
      self.fsm.error = self.subfsm.error
   end
end

-- Resets the sub-FSM.
function SubFSMJumpState:reset()
   JumpState.reset(self)
   self.subfsm:reset()
end


--- Get string representation.
-- @return string of the form SubFSMJumpState[FSM/State]@Current, where FSM
-- will be replaced by the FSM's name, State by this state's name and Current
-- by the name of the current state of the sub-FSM.
function SubFSMJumpState:__tostring()
   return string.format("SubFSMJumpState[%s/%s]@%s", self.fsm.name, self.name, self.fsm.current.name)
end
