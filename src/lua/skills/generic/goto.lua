
----------------------------------------------------------------------------
--  goto.lua - generic global goto
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
name               = "goto"
fsm                = SkillHSM:new{name=name, start="GOTO"}
depends_skills     = {"relgoto"}
depends_interfaces = {
   {v = "pose", type = "ObjectPositionInterface"},
   {v = "navigator", type = "NavigatorInterface"}
}

documentation      = [==[Global goto skill.
This skill takes you to a position given in global coordinates in the global world
coordinate system. The orientation is the final orientation, nothing is said about the
intermediate orientation while on the way. The margin is the precision of the goto
command. The goto is considered final if the current (x,y) is in a radius of the
given margin around the destination. For example is the margin is 0.5 then the goto
is considered final if the robot is in a circle of 0.5m radius around the target point.
The default margin is 0.2m.

NOTE: This is pretty much coded in the sky, without much backing from lower level software,
do not consider this code to be working, yet.

There are several forms to call this skill:
1. goto(x, y[, ori[, margin]])
   This will goto the position giving in the global world coordinates.
2. goto{x=X, y=Y[, ori=ORI][, margin=MARGIN]}
   Go to the relative cartesian coordinates (X,Y) with the optional final orientation ORI.

Parameters:
x, y:      global target point
ori:       global orientation
margin:    radius of a circle around the destination point, if the robot is within
           that circle the goto is considered final.

The skill is S_RUNNING as long as the target can still be reached, S_FINAL if the target
has been reached (at least once, the robot might move afterwards for example if it did not
brake fast enough or if another robot crashed into this robot). The skill is S_FAILED if
the navigator started processing another goto message.
]==]

-- Constants
local DEFAULT_ORI = 0.0
local DEFAULT_MARGIN = 0.2

-- Initialize as skill module
skillenv.skill_module(...)

-- Jumpconditions
local function jumpcond_resend_command(state)
   if navigator:msgid() ~= relgoto.fsm.vars.msgid then
      return false
   end

   local target_x_glob = state.fsm.vars.x or state.fsm.vars[1] or pose:world_x()
   local target_y_glob = state.fsm.vars.y or state.fsm.vars[2] or pose:world_y()
   local target_ori_glob = state.fsm.vars.ori or state.fsm.vars[3] or DEFAULT_ORI

   local rx, ry, rori = pose:world_x(), pose:world_y(), pose:world_z()

   local global_to_local = fawkes.HomTransform:new()
   global_to_local:rotate_z( -rori )
   global_to_local:trans( -rx, -ry )

   local proj_target = fawkes.HomPoint:new( target_x_glob, target_y_glob )
   proj_target:transform( global_to_local )

   local target_x_nav = navigator:dest_x()
   local target_y_nav = navigator:dest_y()
   local target_ori_nav = navigator:dest_ori()
   local target = fawkes.HomPoint:new( target_x_nav, target_y_nav)--, target_ori_nav )

   -- check distance
   local margin = state.fsm.vars.margin
   local dist = fawkes.HomVector:new( target - proj_target ):length()

   --[[
   printf("Global target %.2f %.2f %.2f", target_x_glob,
	  target_y_glob, target_ori_glob)
   printf("Current pos   %.2f %.2f %.2f", rx, ry, rori)
   printf("Proj. target  %.2f %.2f", proj_target:x(),
	  proj_target:y())
   printf("Distance:     %.2f Margin:    %.2f", dist, margin)
   --]]

   if dist > margin then
      if state.fsm.vars.counter < 30 then
	 state.fsm.vars.counter = state.fsm.vars.counter + 1
	 return false
      else
	 printf("Recalculating target position")
	 return true
      end
   else
      state.fsm.vars.counter = 0
      return false
   end
end


-- States
fsm:define_states{
   export_to=_M,

   {"GOTO", SkillJumpState, skills={{relgoto}},
            final_to="FINAL", fail_to="FAILED"}
}

-- Transitions
fsm:add_transitions{
   --{"GOTO", final="FINAL", fail="FAILED", {relgoto} },
   {"GOTO", "GOTO", cond=jumpcond_resend_command, desc="recalculated current goto params"}
}



function GOTO:init()
   local x   = self.fsm.vars.x   or self.fsm.vars[1] or pose:world_x()
   local y   = self.fsm.vars.y   or self.fsm.vars[2] or pose:world_y()
   local ori = self.fsm.vars.ori or self.fsm.vars[3] or DEFAULT_ORI
   self.fsm.vars.margin = self.fsm.vars.margin or DEFAULT_MARGIN

   -- global robot pose
   local rx, ry, rori = pose:world_x(), pose:world_y(), pose:world_z()

   --printf("current pos: %.2f %.2f %.2f", rx, ry, rori)

   -- global to local transform
   local global_to_local = fawkes.HomTransform:new()
   global_to_local:rotate_z( -rori )
   global_to_local:trans( -rx, -ry )

   -- transform target
   local target_pos = fawkes.HomPoint:new(x, y)
   local target_ori = fawkes.HomVector:new( math.cos( ori ),
					    math.sin( ori ) )
   target_pos:transform( global_to_local )
   target_ori:transform( global_to_local )

   local relx   = target_pos:x()
   local rely   = target_pos:y()
   local relori = math.atan2( target_ori:y(), target_ori:x() )

   --printf("relative target coords: %.2f %.2f %.2f", relx, rely, relori)

   -- reset counter
   self.fsm.vars.counter = 0

   self.args[relgoto] = {x=relx, y=rely, ori=relori}
end

