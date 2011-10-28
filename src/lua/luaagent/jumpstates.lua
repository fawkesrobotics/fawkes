
------------------------------------------------------------------------
--  jumpstates.lua - LuaAgent JumpState
--
--  Created: Fri Jan 02 15:31:45 2009
--  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
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
require("luaagent.skillqueue")
local skillstati = require("skiller.skillstati")
local skjsmod = require("skiller.skill_jumpstate")

-- Convenience shortcuts
local JumpState      = fawkes.fsm.jumpstate.JumpState
local SkillJumpState = skjsmod.SkillJumpState
local SkillQueue     = luaagent.skillqueue.SkillQueue


--- @class SkillJumpState
-- Skill jump states to build up Hybrid State Machines (HSM) specifically for
-- the use in skills.
-- Note: Other than for FSM states, the HSM states ignore the return value of the
-- loop() function. You need to add transitions with jump conditions for state
-- transitions and cannot return a state to switch to after the loop.
-- @author Tim Niemueller
AgentSkillExecJumpState = { set_transition_labels = SkillJumpState.set_transition_labels }



--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function AgentSkillExecJumpState:new(o)
   assert(o, "AgentSkillExecJumpState requires a table as argument")
   assert(o.name, "AgentSkillExecJumpState requires a name")
   assert(not o.skiller, "Calling with custom skiller interface no longer supported")
   assert(not getmetatable(o), "Meta table already set for object")

   setmetatable(o, self)
   setmetatable(self, JumpState)
   self.__index = self

   o.transitions   = o.transitions or {}
   o.preconditions = o.preconditions or {}
   o.dotattr       = {}
   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")
   assert(type(o.preconditions) == "table", "Preconditions for " .. o.name .. " not a table")

   o.skills = o.skills or {}
   o.loops  = o.loops or {}
   o.inits  = o.inits or {}
   o.skill_queue = SkillQueue:new{name=o.name, skills=o.skills, fsm=o.fsm}

   o.hide_final_transition = o.hide_final_transition or false
   o.hide_failure_transition = o.hide_failure_transition or false
   assert(type(o.hide_final_transition) == "boolean", "Hide final transition for " .. o.name .. " not a boolean")
   assert(type(o.hide_failure_transition) == "boolean", "Hide failure transition for " .. o.name .. " not a boolean")
   
   o.skill_status = skillstati.S_RUNNING

   o:set_transition_labels()

   return o
end


--- Add a skill.
-- @param skillname name of the skill to add
-- @param params skill parameters, an array with elements generated via fixp() and
-- fsmp() functions.
-- @see SkillQueue:add_skill()
function AgentSkillExecJumpState:add_skill(skillname, params)
   self.skill_queue:add_skill(skillname, params)
end


--- Init state.
-- Calls init(). if params were passed (table containing parameters
-- suitable for SkillQueue:set_params()) they are passed to the internal
-- skill queue. The skill queue is executed and intermediate skill status
-- is S_RUNNING.
function AgentSkillExecJumpState:do_init()
   self.skill_status = skillstati.S_INACTIVE

   -- Note that this also already calls init() and checks the regular
   -- non-precondition transitions!
   local rv = { JumpState.do_init(self) }
   if next(rv) then return unpack(rv) end

   if self.args then
      self.skill_queue:set_args(self.args)
   end
   self.skill_queue:execute()
   self.skill_status = skillstati.S_RUNNING
   self:set_transition_labels()
end

--- On leaving state.
-- Stops executed skills and resets the skill queue. Calls state's exit()
-- method.
function AgentSkillExecJumpState:do_exit()
   self.skill_queue:stop()
   self.skill_queue:reset()
   self:exit()
   self.fsm.error = self.error
end

--- Execute loop.
function AgentSkillExecJumpState:do_loop()
   self.skill_status = self.skill_queue:status()
   self.error        = self.skill_queue:error()

   return JumpState.do_loop(self)
end


function AgentSkillExecJumpState:prepare()
   JumpState.prepare(self);
   if not self.final_state and self.final_to then self.final_state = self.final_to end
   if not self.failure_state and self.fail_to then self.failure_state = self.fail_to end

   if type(self.final_state) == "string" then
      --printf("Setting prematurely declared final state %s", self.final_state)
      local tmpstr = self.final_state
      self.final_state        = self.fsm.states[self.final_state]
      assert(self.final_state, "Prematurely defined final state "..tmpstr..
                               " does not exist")
   end
   if type(self.failure_state) == "string" then
      local tmpstr = self.failure_state
      --printf("Setting prematurely declared failure state %s", self.failure_state)
      self.failure_state      = self.fsm.states[self.failure_state]
      assert(self.failure_state, "Prematurely defined failure state "..tmpstr..
                                 " does not exist", tmpstr)
   end

   local skills = (#self.skills == 1) and "Skill" or "Skills"
   if self.final_state and self.failure_state and self.final_state == self.failure_state then
      self.transition = self:add_new_transition(self.final_state, self.finished, skills .. " finished")
      self.transition.hide = (self.hide_final_transition and self.hide_failure_transition)
   else
      if self.final_state then
         self.final_transition   = self:add_new_transition(self.final_state, self.final, skills .. " succeeded")
         self.final_transition.hide = self.hide_final_transition
      end
      if self.failure_state then
         self.failure_transition = self:add_new_transition(self.failure_state, self.failed, skills .. " failed")
         self.failure_transition.hide = self.hide_failure_transition
      end
   end

   self:set_transition_labels()
   self.fsm:mark_changed()
end

function AgentSkillExecJumpState:reset()
   JumpState.reset(self)
   self.skill_status = skillstati.S_INACTIVE
end

--- Fires if skill(s) finished successfully.
function AgentSkillExecJumpState:final()
   return self.skill_status == skillstati.S_FINAL
end


--- Fires if any called skill failed.
function AgentSkillExecJumpState:failed()
   return self.skill_status == skillstati.S_FAILED
end

--- Fires if the skills are finished.
-- @return true if the status is not S_RUNNING and not S_INACTIVE,
-- i.e. at least one skill failed or all succeeded.
function AgentSkillExecJumpState:finished()
   return self.skill_status ~= skillstati.S_RUNNING
      and self.skill_status ~= skillstati.S_INACTIVE
end

