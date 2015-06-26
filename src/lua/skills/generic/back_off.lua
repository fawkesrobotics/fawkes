
----------------------------------------------------------------------------
--  back_off.lua - Back off
--
--  Created: Thu Apr 24 17:04:12 2014
--  Copyright  2014  Bahram Maleki-Fard
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
name               = "back_off"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"relgoto"}
depends_interfaces = {
}

documentation      = [==[Back off skill

This skill simply orders the robot to back off a little. Driving backwards
might require special settings, therefore this is put into a separate
skill, instead of simply calling "relgoto" with a negative x-value.

Possible call modes:

back_off{}
back_off{dist=DIST}
  where DIST ist the distance to back off (in meters), default is 0.2

]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
BACK_OFF_DISTANCE = 0.2 -- default back-off distance

-- States
fsm:define_states{
   export_to=_M,
   closure={},

   {"INIT",     JumpState},

   {"BACK_OFF", SkillJumpState, skills={{"relgoto"}},
                final_to="FINAL", fail_to="FAILED"}
}

-- Transitions
fsm:add_transitions {
   {"INIT", "BACK_OFF", cond=true, desc="initialized"}
}

function INIT:init()
   self.fsm.vars.dist = self.fsm.vars.dist or BACK_OFF_DISTANCE
end

function BACK_OFF:init()
   self.args["relgoto"] = {x=-self.fsm.vars.dist, y=0, ori=0, backwards=true}
end

