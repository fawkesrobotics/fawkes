
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

--- SubSkillJumpState.
--
-- WARNING: documentation currently out-of-sync, rewrite in progress.
--
-- Skill jump states to build up Hybrid State Machines (HSM)
-- specifically for the use in skills. SubSkillJumpState (SJS) provide
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

local skillenv = require("skiller.skillenv")

-- Convenience shortcuts
local JumpState     = fawkes.fsm.jumpstate.JumpState


SubSkillJumpState = {}

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SubSkillJumpState:new(o)
   assert(o, "SubSkillJumpState requires a table as argument")
   assert(o.name, "SubSkillJumpState requires a name")
   assert(o.fsm, "SubSkillJumpState " .. o.name .. " requires a FSM")
   assert(not getmetatable(o), "Meta table already set for SubSkillJumpState " .. o.name)

   --if o.skill or o.skills then
   assert(o.final_to, "SubSkillJumpState " .. o.name .. " requires final_to state")
   o.fail_to = o.fail_to or "FAILED"
   --end

   --assert(not (o.skill or o.skills or o.subskills) or
   --    o.skill and not o.skills and not o.subskills or
   --    o.skills and not o.skill and not o.subskills or
   --    o.subskills and not o.skill and not o.skills,
   -- "SubSkillJumpState " .. o.name .. " may only operate in a specific mode")
   assert(o.skills, "No skills given")
   for _,s in ipairs(o.skills) do
      if type(s[1]) == "string" then
         s[1] = skillenv.get_skill_module(s[1])
      end
   end

   setmetatable(o, self)
   setmetatable(self, JumpState)
   self.__index = self

   o.skill_status  = skillstati.S_RUNNING
   --o.subskills     = o.subskills or {}
   o.transitions   = o.transitions or {}
   o.loops         = o.loops or {}
   o.inits         = o.inits or {}
   o.dotattr       = o.dotattr or {}
   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")
   --assert(type(o.subskills) == "table", "Subskills for " .. o.name .. " not a table")
   assert(type(o.dotattr) == "table", "Dot attributes for " .. o.name .. " not a table")
   assert(type(o.loops) == "table", "Loops for " .. o.name .. " not a table")
   assert(type(o.inits) == "table", "Inits for " .. o.name .. " not a table")

   if o.final_to and o.fail_to and o.final_to == o.fail_to then
      o.final_transition =
         o:add_new_transition(o.final_to, o.jumpcond_skill_done)
      o.hide_final_transition = o.hide_final_transition or false
      assert(type(o.hide_final_transition) == "boolean",
             "Hide final transition for " .. o.name .. " not a boolean")
      o.final_transition.hide = o.hide_final_transition
      o.final_fail_trans = true
      if o.fintrans_dotattr then
         o.final_transition.dotattr = o.fintrans_dotattr
      end
   else
      if o.final_to then
         o.final_transition =
            o:add_new_transition(o.final_to, o.jumpcond_skill_final)
         o.hide_final_transition = o.hide_final_transition or false
         assert(type(o.hide_final_transition) == "boolean",
                "Hide final transition for " .. o.name .. " not a boolean")
         o.final_transition.hide = o.hide_final_transition
         if o.fintrans_dotattr then
            o.final_transition.dotattr = o.fintrans_dotattr
         end
      end
      if o.fail_to then
         o.failure_transition =
            o:add_new_transition(o.fail_to, o.jumpcond_skill_failed)
         o.hide_failure_transition = o.hide_failure_transition or false
         assert(type(o.hide_failure_transition) == "boolean",
                "Hide failure transition for " .. o.name .. " not a boolean")
         o.failure_transition.hide = o.hide_failure_transition
         if o.failtrans_dotattr then
            o.failure_transition.dotattr = o.failtrans_dotattr
         end
      end
   end

   o:set_transition_labels()

   return o
end

function SubSkillJumpState:set_transition_labels()
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
      self.skill_names = table.concat(snames, ", ")
      if self.failure_transition then
	 self.failure_transition.description =
            table.concat(snames, " or ") .. " failed"
      end
      if self.final_transition then
	 self.final_transition.description =
            table.concat(snames, " and ")
         if self.final_fail_trans then
            self.final_transition.description =
               self.final_transition.description .. " done"
         else
            self.final_transition.description =
               self.final_transition.description .. " succeeded"
         end
      end
      self.dotattr.comment = table.concat(snames, ", ")
   else
      self.skill_names = ""
      if self.failure_transition then
	 self.failure_transition.description = "Skills failed"
      end
      if self.final_transition then
	 self.final_transition.description   = "Skills succeeded"
      end
   end
   self.fsm:mark_changed()
end

function SubSkillJumpState:jumpcond_skill_done()
   return self:jumpcond_skill_final() or self:jumpcond_skill_failed()
end

function SubSkillJumpState:jumpcond_skill_final()
   return self.skill_status == skillstati.S_FINAL
end

function SubSkillJumpState:jumpcond_skill_failed()
   if self.skill_status == skillstati.S_FAILED then
      local error = ""

      if error and error ~= "" then
	 self.fsm:set_error(self.name .. "()/" .. self.skill_names ..
                            " failed: " .. error)
      end
      return true
   else
      return false
   end
end


--- Execute init routines.
-- This resets any skills that have been added for this state and then executes
-- the state's init() routine. Do not overwrite do_init(), rather implement init().
function SubSkillJumpState:do_init()
   -- Try preconditions
   local rv = { self:try_transitions(true) }
   if next(rv) then return unpack(rv) end

   self:skill_reset()
   self.skill_status = skillstati.S_RUNNING
   
   self.args = {}

   -- store expected size of arguments table to compare after run of init()
   local exp_sizeof_args = 0
   local act_sizeof_args = 0

   for _, s in ipairs(self.skills) do
			if s[1] ~= nil then
				 local sname = ""
				 if type(s[1]) == "string" then
						sname = s[1]
				 else
						sname = s[1].name
				 end
				 self.args[sname] = {}
             exp_sizeof_args = exp_sizeof_args + 1
			end
	 end
   self:init()
   -- compare size of expected argument table with actual argument table
   for skillname, skill in pairs(self.args) do
      act_sizeof_args = act_sizeof_args + 1
   end
   if exp_sizeof_args ~= act_sizeof_args then 
      print_warn("Expected size of self.args: " .. tostring(exp_sizeof_args) .. ", actual size of self.args: "
      .. tostring(act_sizeof_args) .. ". Make sure you set the correct key in self.args[key]")
   end

   for _, s in ipairs(self.skills) do
      local set_already = false
      local args = {}

      for k, v in pairs(s) do
         if k ~= 1 then
            set_already = true
            if type(k) == "number" and type(v) == "table" then
	       -- assuming this is the default args table, passed in define_states
               for k2, v2 in pairs(v) do
                  if type(v2) == "table" then
		     args[k2] = table.deepcopy(v2)
		  else
		     args[k2] = v2
		  end
               end

            elseif type(k) ~= "string" or (k ~= "status" and k ~= "args") then
	       print_warn("You have set the subskill "..s[1].name.."'s field '"..tostring(k).."'."
			  .." Please pass the subskill's arguments in the 'args' field of subskills,"
			  .." or the 'args' field of the state that envokes the subskill(s).")
               args[k] = v
	    end
         end
      end

      -- Set args from "self.args[skill] = {arg1=arg,...}",
      --            or "self.skills[..].args = {arg1=arg,...}"
      if self.args[s[1]] or self.args[s[1].name] or s.args then
         sargs = self.args[s[1]] or self.args[s[1].name] or s.args
         if type(sargs) == "table" then
            set_already = true
            for k, v in pairs(sargs) do
               args[k] = v
            end
         end
      end

      if not set_already then
         for k, v in pairs(self.fsm.vars) do args[k] = v end
      end

      s.__args = args
   end

   if self.skills then
      local t = {}
      for _, skill in ipairs(self.skills) do
	 table.insert(t, self:skillstring(skill))
      end
      print_debug("%s: executing %s", self.name, table.concat(t, "; "))
   --elseif self.skill then
   --   print_debug("%s: executing %s", self.name, self:skillstring(self.skill))
   end

   for _,i in ipairs(self.inits) do
      i(self)
   end

   return self:try_transitions()
end


--- Execute exit routine.
-- This calls the state's exit() routine and then resets any subskill that have been
-- added for this state. Do not overwrite do_exit(), rather implement exit().
function SubSkillJumpState:do_exit()
   self:exit()
   for _, s in ipairs(self.skills) do
      s.__args = nil
      s.status = skillstati.S_RUNNING
      s[1].reset()
   end
end

function SubSkillJumpState:skillstring(skill)
   local s = skill[1].name .. "{"
   local first = true
   for k,v in pairs(skill.__args ) do
      s = s .. string.format("%s%s = %s", first and "" or ", ", k, tostring(v))
      first = false
   end
   s = s .. "}"
   return s
end

--- Execute loop.
function SubSkillJumpState:do_loop()
   self:loop()

   -- status might have been changed in custom loop(), execute the following only
   -- for single sub-skill execution and status S_RUNNING
   if self.skill_status == skillstati.S_RUNNING then
      local all_final = true
      for _, s in ipairs(self.skills) do
         if s.status == skillstati.S_RUNNING then
            s.status = s[1](s.__args)
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

   for _,l in ipairs(self.loops) do
      l(self)
   end

   return self:try_transitions()
end


function SubSkillJumpState:set_skill_name(skill_name)
   if self.final_transition then
      self.final_transition.description = skill_name .. "() succeeded"
   end

   if self.failure_transition then
      self.failure_transition.description = skill_name .. "() failed"
   end
end

function SubSkillJumpState:skill_reset()
   if self.skills then
      for _, s in ipairs(self.skills) do
	 s.args = nil
	 s.__args = nil
	 s.status = skillstati.S_RUNNING
	 s[1].reset()
      end
   end
end

function SubSkillJumpState:reset()
   JumpState.reset(self)
   self:skill_reset()
   self.skill_status = skillstati.S_INACTIVE
end
