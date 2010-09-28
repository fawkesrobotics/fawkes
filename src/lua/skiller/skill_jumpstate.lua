
------------------------------------------------------------------------
--  skill_jumpstates.lua - HSM state specifically for skills
--
--  Created: Wed Dec 10 14:09:48 2008
--  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
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

--- SkillJumpState.
-- Skill jump states to build up Hybrid State Machines (HSM)
-- specifically for the use in skills. SkillJumpState (SJS) provide
-- specific tools to deal with sub-skills. SJS may operate either a
-- single sub-skill, which is automatically executed and reset in the
-- loop and appropriate transitions on success or failure are
-- followed, or it may have a list of skills to execute from which to
-- generate the combined status value, or it may hold a list of
-- sub-skills, which are reset automatically only when entering or
-- leaving the state, or it may. These modes may not be intermixed.
--
-- For single skill execution, you can either pass an args argument to
-- the new() method which are passed for skill execution. You can
-- implement the init() and loop() methods, to generate or calculate
-- the parameters. They are executed before the sub-skill is
-- executed. You can pass an args table to the constructor which are
-- used for execution if no arguments are set during the init()
-- method.
--
-- For multi-skill execution you must pass a skills table to the
-- constructor.  The table must contain one entry per skill to
-- execute. Each entry is a table, with the element at index 1 being
-- the skill (functable) and index 2 an (optional) table or arguments
-- passed verbatim. It acts as the base arguments for the skills.
-- They may be overridden by setting the self.skills[S].args table,
-- where S is the skill name in question.
--
-- For multiple sub-skills you must call the sub-skills by yourself in
-- the loop() method.
-- Note: Other than for FSM states, the HSM states ignore the return
-- alue of the loop() function. You need to add transitions with jump
-- conditions for state transitions and cannot return a state to switch
-- to after the loop.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

require("fawkes.fsm.jumpstate")
local skillstati = require("skiller.skillstati")

-- Convenience shortcuts
local JumpState     = fawkes.fsm.jumpstate.JumpState


SkillJumpState = {}

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SkillJumpState:new(o)
   assert(o, "SkillJumpState requires a table as argument")
   assert(o.name, "SkillJumpState requires a name")
   assert(o.fsm, "SkillJumpState " .. o.name .. " requires a FSM")
   assert(not getmetatable(o), "Meta table already set for SkillJumpState " .. o.name)

   if o.skill or o.skills then
      assert(o.final_state, "SkillJumpState " .. o.name .. " requires success target state for sub-skill execution")
      o.failure_state = o.failure_state or "FAILED"
   end


   assert(not (o.skill or o.skills or o.subskills) or
       o.skill and not o.skills and not o.subskills or
       o.skills and not o.skill and not o.subskills or
       o.subskills and not o.skill and not o.skills,
    "SkillJumpState " .. o.name .. " may only operate in a specific mode")

   setmetatable(o, self)
   setmetatable(self, JumpState)
   self.__index = self

   o.skill_status  = skillstati.S_RUNNING
   o.subskills     = o.subskills or {}
   o.transitions   = o.transitions or {}
   o.dotattr       = o.dotattr or {}
   o.base_args     = o.args or {}
   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")
   assert(type(o.subskills) == "table", "Subskills for " .. o.name .. " not a table")
   assert(type(o.dotattr) == "table", "Dot attributes for " .. o.name .. " not a table")

   if o.final_state then
      o.final_transition = o:add_new_transition(o.final_state, o.jumpcond_skill_final)
      if o.fintrans_dotattr then o.final_transition.dotattr = o.fintrans_dotattr end
   end
   if o.failure_state then
      o.failure_transition = o:add_new_transition(o.failure_state, o.jumpcond_skill_failed)
      if o.failtrans_dotattr then o.failure_transition.dotattr = o.failtrans_dotattr end
   end

   o:set_transition_labels()

   return o
end

function SkillJumpState:set_transition_labels()
   if self.skills and #self.skills > 0 then
      local snames = {}
      for _,s in ipairs(self.skills) do
	 if s[1] == nil then
	    error("Skill entry is nil, forgot to add skill to dependencies?")
	 elseif type(s[1]) == "string" then
	    table.insert(snames, s[1])
	 else
	    table.insert(snames, s[1].name)
	 end
      end
      if self.failure_transition then
	 self.failure_transition.description = table.concat(snames, " or ") .. " failed"
      end
      if self.final_transition then
	 self.final_transition.description   = table.concat(snames, " and ") .. " succeeded"
      end
      self.dotattr.comment = table.concat(snames, ", ")
   elseif self.skill then
      self.final_transition.description = self.skill.name .. " succeeded";
      self.failure_transition.description = self.skill.name .. " failed";
   else
      if self.failure_transition then
	 self.failure_transition.description = "Skills failed"
      end
      if self.final_transition then
	 self.final_transition.description   = "Skills succeeded"
      end
   end
   self.fsm:mark_changed()
end

--- Add a subskill to this state.
-- Skills which are added as subskills are automatically reset during init and
-- exit.
-- @param subskill subskill to add
function SkillJumpState:add_subskill(subskill)
   table.insert(self.subskills, subskill)
end


function SkillJumpState:jumpcond_skill_final()
   return self.skill_status == skillstati.S_FINAL
end

function SkillJumpState:jumpcond_skill_failed()
   if self.skill_status == skillstati.S_FAILED then
      local error = ""
      if self.skill then
	 error = self.skill.error
	 if self.skill.fsm then
	    error = self.skill.fsm.error
	 end
      end

      if error and error ~= "" then
	 self.fsm:set_error(self.name .. "() failed, " .. error)
      end
      return true
   else
      return false
   end
end


--- Execute init routines.
-- This resets any subskills that have been added for this state and then executes
-- the state's init() routine. Do not overwrite do_init(), rather implement init().
function SkillJumpState:do_init()
   self.args = {}

   -- Try preconditions
   local rv = { self:try_transitions(true) }
   if next(rv) then return unpack(rv) end

   self:skill_reset()
   self.skill_status = skillstati.S_RUNNING

   for k, v in pairs(self.fsm.vars) do self.args[k] = v end
   if self.skills then
      for i, s in ipairs(self.skills) do
	 s.args = self.args
      end
   end
   self:init()

   if self.skills then
      local t = {}
      for _, skill in ipairs(self.skills) do
	 table.insert(t, self:skillstring(skill))
      end
      print_debug("%s: executing %s", self.name, table.concat(t, "; "))
   elseif self.skill then
      print_debug("%s: executing %s", self.name, self:skillstring(self.skill))
   end

   return self:try_transitions()
end


--- Execute exit routine.
-- This resets any subskills that have been added for this state and then executes
-- the state's exit() routine. Do not overwrite do_exit(), rather implement exit().
function SkillJumpState:do_exit()
   for _, s in ipairs(self.subskills) do
      s.reset()
   end
   if self.skills then
      for _, s in ipairs(self.skills) do
	 s.args = nil
	 s.status = skillstati.S_RUNNING
	 s[1].reset()
      end
   end
   if self.skill then
      self.skill.reset()
   end
   self:exit()
end

function SkillJumpState:skillstring(skill)
   local s = skill.name .. "{"
   local first = true
   for k,v in pairs(self.args or self.base_args) do
      s = s .. string.format("%s%s = %s", first and "" or ", ", k, tostring(v))
      first = false
   end
   s = s .. "}"
   return s
end

--- Execute loop.
function SkillJumpState:do_loop()
   self:loop()

   -- status might have been changed in custom loop(), execute the following only
   -- for single sub-skill execution and status S_RUNNING
   if self.skill_status == skillstati.S_RUNNING then
      if self.skill then
	 if self.fsm.debug then
	    print_debug("%s: executing %s", self.name, self:skillstring(self.skill))
	 end
	 self.skill_status = self.skill(self.args or self.base_args)
      elseif self.skills then
	 local all_final = true
	 for _, s in ipairs(self.skills) do
	    if s.status == skillstati.S_RUNNING then
	       s.status = s[1](s.args or s[2])
	       if s.status == skillstati.S_FAILED then
		  self.skill_status = s.status
		  all_final = false
		  break;
	       elseif s.status == skillstati.S_RUNNING then
		  all_final = false
	       end
	    end
	 end
	 if all_final then
	    self.skill_status = skillstati.S_FINAL
	 end
      end
   end

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

function SkillJumpState:skill_reset()
   if self.skill then self.skill.reset() end
   for _, s in ipairs(self.subskills) do
      s.reset()
   end
   if self.skills then
      for _, s in ipairs(self.skills) do
	 s.args = nil
	 s.status = skillstati.S_RUNNING
	 s[1].reset()
      end
   end
   self.args = {}
end

function SkillJumpState:reset()
   JumpState.reset(self)
   self:skill_reset()
   self.skill_status = skillstati.S_INACTIVE
end
