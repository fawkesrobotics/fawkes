
----------------------------------------------------------------------------
--  intercept_ball.lua - Skill to intercept a ball
--
--  Created: Thu Jan 22 00:07:06 2009
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

local p = require("predicates.soccer.general")

-- Crucial skill information
name               = "intercept_ball"
fsm                = SkillHSM:new{name=name, start="CHECK_VISIBILITY"}
depends_skills     = { "relgoto" }
depends_interfaces = {
   {v = "wm_ball", type = "ObjectPositionInterface", id = "WM Ball"}
}

documentation      = [==[Ball interception skill.]==]

-- Initialize as skill module
skillenv.skill_module(...)

local BACKOFF_DISTANCE = 0.18

local function relgoto_params(state)
   local bearing, distance = wm_ball:bearing(), wm_ball:distance()
   state.args = {phi  = bearing,
		 dist = math.max(0.0, distance - BACKOFF_DISTANCE),
		 ori  = bearing}
end

-- States
fsm:add_transitions{
   closure={p=p},
   {"CHECK_VISIBILITY", "FINAL",
    cond="p.ball_visible and p.ball_close and p.ball_infront", precond=true},
   {"CHECK_VISIBILITY", "INTERCEPT_BALL", cond="p.ball_visible"},
   {"CHECK_VISIBILITY", "FAILED", cond="not p.ball_visible"},   
   {"INTERCEPT_BALL", "CHECK_AGAIN", skill=relgoto, init=relgoto_params},
   {"CHECK_AGAIN", "FINAL", cond="p.ball_visible"},
   {"CHECK_AGAIN", "FAILED", cond="not p.ball_visible"},  
}
