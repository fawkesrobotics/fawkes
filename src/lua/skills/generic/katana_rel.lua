
----------------------------------------------------------------------------
--  katana_rel.lua - Katana straight movement
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
name               = "katana_rel"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {"katana"}
depends_interfaces = {
   {v = "katanaarm", type = "KatanaInterface"}
}

documentation      = [==[Katana straight movement skill

This skill moves the katana arm by a given relative position (no rotation taken).


Possible call modes:

katana_rel{x=X, y=Y, z=Z}
 Moves katana on straight line (relative goto). At least one argument needs to be given
 X: relative x position
 Y: relative y position
 Z: relative z position
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:define_states{
   export_to=_M,
   closure={katanaarm=katanaarm},

   {"INIT", JumpState},
   {"MOVE", SkillJumpState, skills={{katana}},
            final_to="FINAL", fail_to="FAILED"}
}

fsm:add_transitions{
   {"INIT", "FAILED", cond="not katanaarm:has_writer()", precond_only=true, desc="no writer"},
   {"INIT", "FAILED", cond="(not vars.x) and (not vars.y) and (not vars.z)", precond_only=true, desc="insufficient arguments"},

   {"INIT", "MOVE", cond=true, precond_only=true, desc="move"}
}

function MOVE:init()
   katanaarm:read()

   -- get relative position (translation)
   local x = self.fsm.vars.x or 0.0
   local y = self.fsm.vars.y or 0.0
   local z = self.fsm.vars.z or 0.0

   -- get rotation
   local psi = katanaarm:theta(), katanaarm:psi()
   -- phi is defined by x and y, and automatically calculated by katana skill. value needs to be precise, therefore
   -- setting it right here could cause katana-skill to fail.

   local theta       = self.fsm.vars.theta or katanaarm:theta()
   local theta_error = self.fsm.vars.theta_error or 0.0
   local straight    = self.fsm.vars.straight    or false
   local frame       = self.fsm.vars.frame or "/katana/kni"

   if frame ~= "/katana/gripper" then
     -- calculate global position (translation)
      x = x + katanaarm:x()
      y = y + katanaarm:y()
      z = z + katanaarm:z()
   end

   self.args = {x=x, y=y, z=z, theta=theta, frame=frame, theta_error=theta_error, straight=straight}
end
