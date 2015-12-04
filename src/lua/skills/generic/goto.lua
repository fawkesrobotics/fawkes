
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
   {v = "pose", type = "Position3DInterface"},
   {v = "navigator", type = "NavigatorInterface"}
}

documentation      = [==[Global goto skill.

This skill takes you to a position given the global world coordinate
system. The orientation is the final orientation, nothing is said
about the intermediate orientation while on the way.

The skill can be parameterized in the input coordinate system (in what
frame are the given coordinates, this is the global frame) and the
output coordinate system (this is the local frame). The skill accepts
separate translation and rotation tolerances (acceptable deviations
from desired final position to consider the movement completed).

goto{x=X, y=Y[, ori=ORI][, global_frame="/map"][, local_frame="/base_link"][, trans_tolerance=0.2][, rot_tolerance=0.1]}

Parameters:
x, y:            global target point
ori:             global orientation
global_frame:    global coordinate frame in which x,y,ori are given
local_frame:     local coordinate frame in which the navigation component expects its input
trans_tolerance: translation tolerance
rot_tolerance:   rotation tolerance

The skill is S_RUNNING as long as the target can still be reached, S_FINAL if the target
has been reached (at least once, the robot might move afterwards for example if it did not
brake fast enough or if another robot crashed into this robot). The skill is S_FAILED if
the navigator started processing another goto message.
]==]

local tfutils = require("fawkes.tfutils")

-- Constants
local DEFAULT_ORI = 0.0
local DEFAULT_TRANS_TOLERANCE = 0.2
local DEFAULT_ROT_TOLERANCE = 0.1
local DEFAULT_GLOBAL_FRAME = config:get_string("/frames/fixed")
local DEFAULT_LOCAL_FRAME = config:get_string("/frames/base")

-- Initialize as skill module
skillenv.skill_module(...)

local function target_reached(self)
   return
      math.abs(self.fsm.vars.x - pose:translation(0)) <= self.fsm.vars.trans_tolerance
      and math.abs(self.fsm.vars.y - pose:translation(1)) <= self.fsm.vars.trans_tolerance
      and math.abs(self.fsm.vars.ori - 2 * math.acos(pose:rotation(3))) <= self.fsm.vars.rot_tolerance
end


-- Jumpconditions
local function jumpcond_resend_command(self)
   if navigator:msgid() ~= relgoto.fsm.vars.msgid then
      return false
   end

   if not target_reached(self) then
      if self.fsm.vars.counter < 30 then
	 self.fsm.vars.counter = self.fsm.vars.counter + 1
	 return false
      else
	 printf("Recalculating target position")
	 return true
      end
   else
      self.fsm.vars.counter = 0
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
   {"GOTO", "FINAL", cond=target_reached, desc="Target reached"},
   {"GOTO", "GOTO", cond=jumpcond_resend_command, desc="recalculated current goto params"}
}


function GOTO:init()
   self.fsm.vars.x      = self.fsm.vars.x   or self.fsm.vars[1] or pose:world_x()
   self.fsm.vars.y      = self.fsm.vars.y   or self.fsm.vars[2] or pose:world_y()
   self.fsm.vars.ori    = self.fsm.vars.ori or self.fsm.vars[3] or DEFAULT_ORI
   self.fsm.vars.trans_tolerance = self.fsm.vars.trans_tolerance or DEFAULT_TRANS_TOLERANCE
   self.fsm.vars.rot_tolerance = self.fsm.vars.rot_tolerance or DEFAULT_ROT_TOLERANCE
   self.fsm.vars.global_frame = self.fsm.vars.global_frame or DEFAULT_GLOBAL_FRAME
   self.fsm.vars.local_frame  = self.fsm.vars.local_frame  or DEFAULT_LOCAL_FRAME
   self.fsm.vars.counter = 0

   local rel_pos =
      tfutils.transform({x = self.fsm.vars.x, y = self.fsm.vars.y, ori = self.fsm.vars.ori},
			self.fsm.vars.global_frame, self.fsm.vars.local_frame)

   self.args[relgoto] = {x=rel_pos.x, y=rel_pos.y, ori=rel_pos.ori}
end

