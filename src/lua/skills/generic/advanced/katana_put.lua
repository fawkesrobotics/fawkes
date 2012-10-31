
----------------------------------------------------------------------------
--  katana_put.lua - Katana skill to put object to a place
--
--  Created: Thu Mar 03 14:32:47 2011
--  Copyright  2011  Bahram Maleki-Fard
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
name               = "katana_put"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {"katana", "or_object"}
depends_interfaces = {
   {v = "katanaarm", type = "KatanaInterface"}
}

documentation      = [==[Katana putting skill

This skill puts an object that the katana holds to a given destination.


Possible call modes:

katana_put{x=X, y=Y, z=Z [, object=OBJECT]}
 X: target x-coordinate (mm)
 Y: target y-coordinate (mm)
 Z: target z-coordinate (mm
 OBJECT: name of object, if known. otherwise all attached bodies are release
 )
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Constants
local SLOW_DOWN_VELOCITY = 0.5

-- States
fsm:define_states{
   export_to=_M,
   closure={katanaarm=katanaarm},

   {"INIT", JumpState},
   {"MOVE", SkillJumpState, skills={{katana}},
   	    final_to="TO_OPEN_GRIPPER", fail_to="MOVE_WITH_THETA"},
   {"MOVE_WITH_THETA", SkillJumpState, skills={{katana}},
	    final_to="TO_OPEN_GRIPPER", fail_to="FAILED"},
   {"OPEN_GRIPPER", SkillJumpState, skills={{katana}},
   	    final_to="RELEASE_OBJECT", fail_to="FAILED"},
   {"RELEASE_OBJECT", SkillJumpState, skills={{katana}},
   	    final_to="FINAL", fail_to="FAILED"},

   {"TO_OPEN_GRIPPER", JumpState}

}
-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", cond="not katanaarm:has_writer()", desc="no writer", precond_only=true},

   {"INIT", "MOVE", cond="vars.x and vars.y and vars.z", desc="move katana"},
   {"INIT", "FAILED", cond=true, desc="insufficient arguments"},

   {"TO_OPEN_GRIPPER", "OPEN_GRIPPER", timeout=3.0}
}

function MOVE:init()
   self.args = {x=self.fsm.vars.x,
		y=self.fsm.vars.y,
		z=self.fsm.vars.z}
end

function MOVE_WITH_THETA:init()
   katanaarm:read()
   self.args = {x=self.fsm.vars.x,
		y=self.fsm.vars.y,
		z=self.fsm.vars.z,
		theta=katanaarm:theta()}
end

function OPEN_GRIPPER:init()
   self.args = {gripper="open"}
end

function RELEASE_OBJECT:init()
   if self.fsm.vars.object then
      self.args = {release=true, name=self.fsm.vars.object}
   else
      self.args = {release_all=true}
   end
end
