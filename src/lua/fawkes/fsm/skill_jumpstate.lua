
------------------------------------------------------------------------
--  skill_jumpstates.lua - HSM jumpstate specifically class for executing skills
--
--  Created: Wed Dec 16 14:45:48 2015
--  Copyright  20015  Bahram Maleki-Fard
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

--- This module serves as a parent class that implements a specific type
--  of a state that can execute a skill. Examples of those specific
--  implementations are execution of a subskill from within a skill
--  (SubskillJumpState) or a state of luaagent that calls a skill
--  (AgentSkillExecJumpState).
-- @author Bahram Maleki-Fard
module(..., fawkes.modinit.module_init)

SkillJumpState = {}
SkillJumpState.__index = SkillJumpState

function SkillJumpState:implemented_by(impl_class)
   assert(impl_class, "SkillJumpState requires an implementation class as argument")

   for k,v in pairs(impl_class) do
     SkillJumpState[k] = v
   end
   SkillJumpState.__index = SkillJumpState
end
