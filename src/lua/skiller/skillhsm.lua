
----------------------------------------------------------------------------
--  skillhsm.lua - Hybrid State Machine for skills, closely related to FSM
--
--  Created: Mon Dec 22 11:53:39 2008
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
--
--  $Id$
--
----------------------------------------------------------------------------

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

--- Hybrid State Machine for skills.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)
local fsmmod = require("fawkes.fsm")
local jsmod = require("fawkes.fsm.jumpstate")
local sjsmod = require("skiller.skill_jumpstates")

local FSM         = fsmmod.FSM
JumpState         = jsmod.JumpState
SkillJumpState    = sjsmod.SkillJumpState
SubSkillJumpState = sjsmod.SubSkillJumpState

SkillHSM = {current = nil,
	    debug   = false,
	    export_states_to_parent = true,
	    set_debug        = FSM.set_debug,
	    set_error        = FSM.set_error,
	    loop             = FSM.loop,
	    trans            = FSM.trans,
	    reset            = FSM.reset,
	    changed          = FSM.changed,
	    mark_changed     = FSM.mark_changed,
	    graph            = FSM.graph,
	    traced           = FSM.traced,
	    traced_state     = FSM.traced_state,
	    traced_trans     = FSM.traced_trans,
	    reset_trace      = FSM.reset_trace,
	    add_state        = FSM.add_state,
	    remove_state     = FSM.remove_state,
	    get_start_state  = FSM.get_start_state
	   }

function SkillHSM:new(o)
   local f = FSM:new(o)
   setmetatable(o, self)
   self.__index = self

   f:clear_states()

   return f
end

--- Clear all states.
-- Removes all states. If no_default_states is not set a FINAL and FAILED state
-- are added.
function SkillHSM:clear_states()
   self.states = {}
   if not self.no_default_states then
      self.exit_state = "FINAL"
      self.fail_state = "FAILED"

      local es = SkillJumpState:new{name = "FINAL", fsm = self}
      local fs = SkillJumpState:new{name = "FAILED", fsm = self}

      self.states["FINAL"] = es
      self.states["FAILED"] = fs

      if self.export_states_to_parent then
	 local e = getfenv(2)
	 if e == _M then
	    e = getfenv(3)
	 end
	 e["FINAL"] = es
	 e["FAILED"] = fs
      end
   end
   self.state_changed = true
end

--- Simple state generation not supported for SkillHSM.
-- Throws an error. Only jump states can be created for SkillHSMs.
function SkillHSM:new_state()
   error("Only jump states can be created for a SkillHSM")
end


--- Create a new jump states for a skill HSM.
-- @see FSM:new_jump_state
-- @param name name of the state and the variable in the environment of the caller
-- that holds this state
-- @param subskill single or multiple subskills. If subskill is nil then a
-- SkillJumpState with no sub-skills is instantiated. If subskill is a table then
-- a SkillJumpState with this table as the list of sub-skills is instantiated. If
-- subskill is not nil and not a table it is assumed that it is the module of a
-- single skill and a SubSkillJumpState is instantiated with final_goto and
-- failure_goto as the outcome states.
-- @param final_goto The state the SubSkillJumpState jumps to on successful
-- execution
-- @param failure_goto The state the SubSkillJumpState jumps to on failed
-- execution
function SkillHSM:new_jump_state(name, subskill, final_state, failure_state, args)
   local s
   if subskill ~= nil then
      assert(type(subskill) == "table", "subskill(s) must be a table")
      if subskill.execute and subskill.reset then
	 --printf("Creating SubSkillJumpState 1 name: " .. name)
	 s = SubSkillJumpState:new{name          = name,
				   fsm           = self,
				   skill         = subskill,
				   final_state   = final_state,
				   failure_state = failure_state,
				   args          = args}
      else
	 --printf("Creating SkillJumpState 2 name: " .. name)
	 s = SkillJumpState:new{name = name, fsm = self,
				subskills     = subskill,
				final_state   = final_state,
				failure_state = failure_state}
      end
   else
      --printf("Creating SkillJumpState 3 name: " .. name)
      s = SkillJumpState:new{name = name, fsm = self}
   end

   self.states[name] = s

   if self.export_states_to_parent then
      local e = getfenv(2)
      e[name] = s
   end

   return s
end

function SkillHSM:new_init_state(name, next_state)
   local s = JumpState:new{name=name, fsm = self}
   s:add_transition(next_state, function (state) return true end, "Goto next state")

   self.states[name] = s

   if self.export_states_to_parent then
      local e = getfenv(2)
      e[name] = s
   end

   return s
end
