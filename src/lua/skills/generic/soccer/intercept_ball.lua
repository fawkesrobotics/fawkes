
----------------------------------------------------------------------------
--  intercept_ball.lua - Skill to intercept a ball
--
--  Created: Thu Jan 22 00:07:06 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
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
fsm                = SkillHSM:new{name=name, start="PARSE_ARGS", debug=true}
depends_skills     = { "relgoto" }
depends_interfaces = {
   {v = "wm_ball", type = "ObjectPositionInterface", id = "WM Ball"},
   {v = "navigator", type = "NavigatorInterface", id = "Navigator"},
   {v = "ball_is_obstacle", type = "SwitchInterface", id = "NavigatorBallIsObstacle"}
}

documentation      = [==[Ball interception skill.

This skill moves the robot into a certain position and orientation
wrt. to the current positions of the ball and the robot, respectively.

Parameters:

ball_dir_x, ball_dir_y: the orientation wrt. to the current pose at
                        which the ball will be after the intercept
ball_dist: the distance to the ball the robot should have after
           executing the skill

]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- (Relative) ball moved
local function ball_moved(state)
   local ball_pos = fawkes.HomPoint:new()
   ball_pos:x( math.cos( wm_ball:bearing() ) * wm_ball:distance() )
   ball_pos:y( math.sin( wm_ball:bearing() ) * wm_ball:distance() )

   local nav_target_pos = fawkes.HomPoint:new()
   nav_target_pos:x( navigator:get_dest_x() )
   nav_target_pos:y( navigator:get_dest_y() )
   local nav_target_ori = navigator:get_dest_ori()

   local ball_dir = fawkes.HomVector:new( state.vars.x, state.vars.y )
   ball_dir:rotate_z( nav_target_ori )

   local intended_ball_pos = nav-target_pos + ball_dir
   local diff = ball_pos - intendended_ball_pos

   if ( math.abs( diff ) > 0.5 ) then
      return true
   else
      return false
   end
end

-- States
fsm:define_states{
   export_to=_M,
   closure={p=p},

   {"PARSE_ARGS",   JumpState},
   {"DO_INTERCEPT", SkillJumpState, skills={{relgoto}},
            final_to="CHECK", fail_to="FAILED"},
   {"CHECK",        JumpState}
}
-- Transitions
fsm:add_transitions{
   {"PARSE_ARGS", "FINAL",
    cond="p.ball_visible and p.ball_close and p.ball_infront", precond=true},
   {"PARSE_ARGS", "DO_INTERCEPT", cond="p.ball_visible"},
   {"PARSE_ARGS", "FAILED", cond=true},
   {"DO_INTERCEPT", "CHECK", cond="self.wait > 60"}, -- needs some testing
   {"CHECK", "FINAL", cond="p.ball_visible and p.ball_close and p.ball_infront"},
   {"CHECK", "FAILED", cond="not( p.ball_visible and p.ball_close and p.ball_infront )"}
}

function PARSE_ARGS:init()
   if self.fsm.vars.ball_dist == nil then
      self.fsm.vars.ball_dist = 0
   end
   if self.fsm.vars.ball_dir_x == nil or self.fsm.vars.ball_dir_y == nil then
      self.fsm.vars.ball_dir = fawkes.HomVector:new( 1, 0 )
   else
      self.fsm.vars.ball_dir = fawkes.HomVector:new( self.fsm.vars.ball_dir_x,
						     self.fsm.vars.ball_dir_y )
      self.fsm.vars.ball_dir:unit()
   end

end

function DO_INTERCEPT:init()
   -- compute target point
   local ball_x = math.cos( wm_ball:bearing() ) * wm_ball:distance()
   local ball_y = math.sin( wm_ball:bearing() ) * wm_ball:distance()
   local x = ball_x - self.fsm.vars.ball_dir:x() * self.fsm.vars.ball_dist
   local y = ball_y - self.fsm.vars.ball_dir:y() * self.fsm.vars.ball_dist
   local ori = math.atan2( self.fsm.vars.ball_dir:x(),
			   self.fsm.vars.ball_dir:y() )
   local m = ball_is_obstacle.EnableSwitchMessage:new()
   ball_is_obstacle:msgq_enqueue_copy(m)
   self.args = { x = x, y = y, ori = ori}
   self.wait = 1
end

function DO_INTERCEPT:loop()
   self.wait = self.wait + 1
end
function CHECK:init()
   local m = ball_is_obstacle.DisableSwitchMessage:new()
   ball_is_obstacle:msgq_enqueue_copy(m)
end
