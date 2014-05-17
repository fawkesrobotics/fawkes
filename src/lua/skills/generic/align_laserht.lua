
----------------------------------------------------------------------------
--  align_laserht.lua - Align to a line detected by laserht
--
--  Created: Fri May 16 22:16:17 2014
--  Copyright  2014  Bahram Maleki-Fard
--                   Tim Niemueller
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
name               = "align_laserht"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"relgoto"}
depends_interfaces = {
   {v = "laserht", type="ObjectPositionInterface", id="LaserLine"}
}

documentation      = [==[Align to straight line in laser data.

align_laserht{}
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- States
fsm:define_states{
   export_to=_M,
   closure={laserht=laserht},

   {"INIT",     JumpState},

   {"ALIGN", SkillJumpState, skills={{"relgoto"}},
             final_to="FINAL", fail_to="FAILED"}
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", precond="not laserht:has_writer()", desc="no writer"},
   {"INIT", "ALIGN", cond=true, desc="initialized"}
}

function INIT:init()
end

function ALIGN:init()
   -- for restoring, save current data from interfaces
   self.args["relgoto"] = {x=0, y=0, ori=laserht:bearing()}
end
