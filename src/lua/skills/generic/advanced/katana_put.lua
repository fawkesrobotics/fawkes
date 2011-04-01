
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

-- functions
fsm:add_transitions {
   closure={p=p, katanaarm=katanaarm},

   {"INIT", "FAILED", "not katanaarm:has_writer()", desc="no writer", precond=true},

   {"INIT", "MOVE", "vars.x and vars.y and vars.z", desc="move katana"},
   {"INIT", "FAILED", true, desc="insufficient arguments"},

   {"MOVE", "TO_OPEN_GRIPPER", skill=katana, fail_to="MOVE_WITH_THETA", desc="reached destination"},
   {"MOVE_WITH_THETA", "TO_OPEN_GRIPPER", skill=katana, fail_to="FAILED", desc="reached destination"},

   {"TO_OPEN_GRIPPER", "OPEN_GRIPPER", wait_sec=3.0},

   {"OPEN_GRIPPER", "RELEASE_OBJECT", fail_to="FAILED", skill=katana, desc="opened gripper"},

   {"RELEASE_OBJECT", "FINAL", fail_to="FAILED", skill=or_object, desc="released"}
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
