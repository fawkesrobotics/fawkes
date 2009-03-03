
------------------------------------------------------------------------
--  skill_jumpstates.lua - HSM skill specifically for skills
--
--  Created: Wed Dec 10 14:09:48 2008
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

--- Jump states to build Hybrid State Machines (HSM) for skills.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

require("fawkes.fsm.jumpstate")
require("skiller.skill_states")
local skillstati = require("skiller.skillstati")

-- Convenience shortcuts
local JumpState     = fawkes.fsm.jumpstate.JumpState
local SkillState    = skiller.skill_states.SkillState
local SubSkillState = skiller.skill_states.SubSkillState



--- @class SkillJumpState
-- Skill jump states to build up Hybrid State Machines (HSM) specifically for
-- the use in skills.
-- Note: Other than for FSM states, the HSM states ignore the return value of the
-- loop() function. You need to add transitions with jump conditions for state
-- transitions and cannot return a state to switch to after the loop.
-- @author Tim Niemueller
SkillJumpState = { add_transition     = JumpState.add_transition,
		   add_precondition   = JumpState.add_precondition,
		   add_precond_trans  = JumpState.add_precond_trans,
		   get_transitions    = JumpState.get_transitions,
		   clear_transitions  = JumpState.clear_transitions,
		   try_transitions    = JumpState.try_transitions,
		   last_transition    = JumpState.last_transition,
		   do_loop            = JumpState.do_loop,
		   init               = JumpState.init,
		   loop               = JumpState.loop,
		   exit               = JumpState.exit,
		   reset              = JumpState.reset,
		   jumpcond_true      = JumpState.jumpcond_true,
		   prepare            = JumpState.prepare,
		   add_subskill       = SkillState.add_subkill,
		   do_exit            = SkillState.do_exit
		 }


--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SkillJumpState:new(o)
   assert(o, "SkillJumpState requires a table as argument")
   assert(o.name, "SkillJumpState requires a name")
   assert(o.fsm, "SkillJumpState " .. o.name .. " requires a FSM")
   assert(not getmetatable(o), "Meta table already set for SkillJumpState " .. o.name)
   setmetatable(o, self)
   self.__index = self

   o.subskills     = o.subskills or {}
   o.transitions   = o.transitions or {}
   o.dotattr       = o.dotattr or {}
   o.preconditions = {}
   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")
   assert(type(o.subskills) == "table", "Subskills for " .. o.name .. " not a table")
   assert(type(o.dotattr) == "table", "Dot attributes for " .. o.name .. " not a table")

   if o.final_state then
      o.final_transition = o:add_transition(o.final_state, SubSkillJumpState.jumpcond_final, "Skills succeeded")
   end
   if o.failure_state then
      o.failure_transition = o:add_transition(o.failure_state, SubSkillJumpState.jumpcond_failure, "Skills failed")
   end
   return o
end


--- Execute init routines.
-- This resets any subskills that have been added for this state and then executes
-- the state's init() routine. Do not overwrite do_init(), rather implement init().
-- @param ... Any parameters, passed to init()
function SkillJumpState:do_init(...)
   local rv = { self:try_transitions(self.preconditions) }
   if next(rv) then return unpack(rv) end

   for _, s in ipairs(self.subskills) do
      s.reset()
   end
   self.skill_status = S_RUNNING
   self:init(...)

   return self:try_transitions()
end


function SkillJumpState:set_skill_name(skill_name)
   if self.final_transition then
      self.final_transition.description = skill_name .. "() succeeded"
   end

   if self.failure_transition then
      self.failure_transition.description = skill_name .. "() failed"
   end
end

--- @class SubSkillJumpState
-- This special jump state executes a specific skill and has exactly two
-- transitions, on for successful skill execution and one for failure.
-- The functions init(), loop(), and exit() are not called.
-- Any parameters given to this sub-skill state during the transition are passed
-- verbatim to the skill's execute() routine.
SubSkillJumpState = { add_transition     = JumpState.add_transition,
		      add_precondition   = JumpState.add_precondition,
		      add_precond_trans  = JumpState.add_precond_trans,
		      try_transitions    = JumpState.try_transitions,
		      get_transitions    = JumpState.get_transitions,
		      last_transition    = JumpState.last_transition,
		      init               = JumpState.init,
		      loop               = JumpState.loop,
		      exit               = JumpState.exit,
		      prepare            = JumpState.prepare,
		      do_exit            = SubSkillState.do_exit
		     }


function SubSkillJumpState:jumpcond_final()
   return self.skill_status == skillstati.S_FINAL
end


function SubSkillJumpState:jumpcond_failure()
   if self.skill_status == skillstati.S_FAILED then
      local error = self.skill.error
      if self.skill.fsm then
	 error = self.skill.fsm.error
      end
      if error and error ~= "" then
	 self.fsm:set_error(self.name .. "() failed, " .. error)
      end
      return true
   else
      return false
   end
end

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SubSkillJumpState:new(o)
   assert(o, "SubSkillJumpState requires a table as argument")
   assert(o.name, "SubSkillJumpState requires a name")
   assert(o.fsm, "SubSkillJumpState " .. o.name .. " requires a FSM")
   assert(o.skill, "SubSkillJumpState " .. o.name .. " requires a skill to execute for state " .. o.name)
   assert(o.final_state, "SubSkillJumpState " .. o.name .. " requires state to go to on success for state " .. o.name)
   assert(o.failure_state, "SubSkillJumpState " .. o.name .. " requires state to go to on failure for state " .. o.name)
   assert(not getmetatable(o), "Meta table already set for SubSkillJumpState " .. o.name)
   setmetatable(o, self)
   self.__index = self
   -- Could be used to make clear that there is no call of init, exit, loop etc.
   -- self.__newindex = self.protect

   o.skill_status  = skillstati.S_RUNNING
   o.dotattr       = o.dotattr or {}
   o.transitions   = {}
   o.preconditions = {}

   o.base_args = o.args or {}
   o.args = o:baseargs()

   JumpState.add_transition(o, o.final_state, o.jumpcond_final, o.skill.name .. "() succeeded")
   JumpState.add_transition(o, o.failure_state, o.jumpcond_failure, o.skill.name .. "() failed")

   return o
end


function SubSkillJumpState:baseargs()
   local args = {}
   for k,v in pairs(self.base_args) do
      args[k] = v
   end
   return args
end

--- Execute init routines.
-- This resets the sub-skill
-- @param ... Any parameters, passed to init()
function SubSkillJumpState:do_init(...)
   local args = { ... }
   if next(args) then
      self.args = args
   end

   local rv = { self:try_transitions(self.preconditions) }
   if next(rv) then return unpack(rv) end

   self.skill.reset()
   self.skill_status = skillstati.S_RUNNING
   self:init()

   return self:try_transitions()
end


--- Execute loop.
function SubSkillJumpState:do_loop()
   self:loop()
   -- status might have been changed in custom loop()
   if self.skill_status == skillstati.S_RUNNING then
      if self.debug then
	 local s = self.skill.name .. "{"
	 local first = true
	 for k,v in pairs(self.args) do
	    s = s .. string.format("%s%s = %s", first and "" or ", ", k, tostring(v))
	    first = false
	 end
	 s = s .. "}"
	 printf("%s: executing %s", self.name, s)
      end
      self.skill_status = self.skill(self.args)
   end
   return self:try_transitions()
end


function SubSkillJumpState:reset()
   JumpState.reset(self)
   self.skill.reset()
   self.skill_status = skillstati.S_INACTIVE
   self.args = self:baseargs()
end
