
----------------------------------------------------------------------------
--  serialexec.lua - Skill to execute other skills one after another
--
--  Created: Tue Jan 20 00:34:35 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
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

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "serialexec"
fsm                = SkillHSM:new{name=name, start="SERIALEXEC"}
depends_skills     = nil
depends_interfaces = nil

documentation      = [==[Serial execution skill.
      
This skill takes a list of skills to execute and executes them one after another.
this should be used only rarely and is mostly meant to be used to execute kind
of a mini-demo from the SkillGUI.
      
The argument is a list of skills, where each skill is represented by an array
with two elements, the first being the skill (table or name) and the second being
the table of arguments for the skill (optional).

Example:
serialexec{skills={{say, {text="Foo Bar"}},{getup},{relgoto,{x=1.0, y=0.0}}}}
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("SERIALEXEC")

function jumpcond_noskills(state)
   return not next(state.fsm.vars.skills)
end

function state_init(self)
   local next = nil
   printf("Number of skills: %d", #self.fsm.vars.skills)
   for i = #self.fsm.vars.skills, 1, -1 do
      local st = self.fsm.vars.skills[i]
      local skill  = st[1]
      local params = st[2]
      printf("Creating state for skill %s", skill.name)
      local final = next or FINAL
      next = fsm:new_jump_state(tostring(i) .. ": " .. skill.name, skill, final, FAILED, params)
   end
   SERIALEXEC:add_transition(next, JumpState.jumpcond_true, "Start")
end

function state_reset(self)
   fsm:clear_states()
   SERIALEXEC = fsm:new_jump_state("SERIALEXEC")
   SERIALEXEC.init  = state_init
   SERIALEXEC.reset = state_reset
   SERIALEXEC:add_precond_trans(FAILED, jumpcond_noskills, "No skills")
end

SERIALEXEC.init  = state_init
SERIALEXEC.reset = state_reset
