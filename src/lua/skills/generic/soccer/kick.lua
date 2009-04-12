
----------------------------------------------------------------------------
--  intercept_ball.lua - Skill to intercept a ball
--
--  Created: Thu Jan 22 00:07:06 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
--
--  $Id: intercept_ball.lua 2077 2009-04-01 10:08:50Z tim $
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

module(..., skillenv.module_init)

local p = require("predicates.soccer.general")
local BACKOFF_DISTANCE = 0.10

name               = "kick"
fsm                = SkillHSM:new{name=name, start="DECIDE", debug=true}
depends_skills     = {"relgoto"}
depends_interfaces = {
  {v="kicker", type="KickerInterface", id="Kicker"},
  {v="wm_ball", type="ObjectPositionInterface", id="WM Ball"},
}
documentation      = [==[Performs a kick.]==]

skillenv.skill_module(...)

fsm:add_transitions{
   closure={p=p, wm_ball=wm_ball},
   {"DECIDE",           "MOVE_TO_BALL", cond="p.ball_visible and "..
                                             "wm_ball:distance() < 2.0",
                                        precond=true},
   {"DECIDE",           "FAILED",       cond=true},
   {"MOVE_TO_BALL",     "CHECK",        skill=relgoto},
   {"CHECK",            "KICK",         cond="p.ball_visible and "..
                                             "p.ball_close and "..
                                             "p.ball_infront"},
   {"CHECK",            "FAILED",       cond=true},
   {"KICK",             "FINAL",        cond=true},
}

function DECIDE:init()
   printf("bearing = %.3f", wm_ball:bearing())
   printf("distance = %.3f", wm_ball:distance())
   printf("p.ball_visibile = %s", tostring(p.ball_visible));
   printf("p.ball_close = %s", tostring(p.ball_close));
   printf("p.ball_infront = %s", tostring(p.ball_infront));
end

function KICK:init()
   printf("bearing = %.3f", wm_ball:bearing())
   printf("distance = %.3f", wm_ball:distance())
   if kicker:has_writer() then
      local left      = self.fsm.vars.left or self.fsm.vars[1]
      local center    = self.fsm.vars.center or self.fsm.vars[2]
      local right     = self.fsm.vars.right or self.fsm.vars[3]
      local intensity = self.fsm.vars.intensity or self.fsm.vars[4]
      local km        = kicker.KickMessage:new(left ~= 0, center ~= 0,
                                               right ~= 0, intensity)
      self.fsm.vars.msgid = kicker:msgq_enqueue_copy(km)
   else
      self.param_fail = true
   end
end

function MOVE_TO_BALL:init()
   printf("MOVE_TO_BALL:init()");
   local bearing, distance = wm_ball:bearing(), wm_ball:distance()
   distance = math.max(0.0, distance - BACKOFF_DISTANCE)
   self.args = {phi = bearing, dist = distance, ori = bearing}
end

function CHECK:init()
   printf("bearing = %.3f", wm_ball:bearing())
   printf("distance = %.3f", wm_ball:distance())
   printf("p.ball_visibile = %s", tostring(p.ball_visible));
   printf("p.ball_close = %s", tostring(p.ball_close));
   printf("p.ball_infront = %s", tostring(p.ball_infront));
end

