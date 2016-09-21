
----------------------------------------------------------------------------
--  relgoto.lua - generic relative goto
--
--  Created: Thu Aug 14 14:28:19 2008
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
name               = "relgoto"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = nil
depends_interfaces = {
   {v = "navigator", id = "Navigator", type = "NavigatorInterface"}
}

documentation      = [==[Relative goto skill.
This skill takes you to a position given in relative coordinates in the robot-local
coordinate system. The orientation is the final orientation, nothing is said about the
intermediate orientation while on the way. The margin is the precision of the relgoto
command. The relgoto is considered final if the current (x,y) is in a radius of the
given margin around the destination. For example is the margin is 0.5 then the relgoto
is considered final if the robot is in a circle of 0.5m radius around the target point.
The default margin is 0.2m.

There are several forms to call this skill:
1. relgoto(x, y[, ori[, margin]])
   This will goto the position giving in the relative cartesian coordinates, optionally with
   the given orientation.
2. relgoto{x=X, y=Y[, ori=ORI][, margin=MARGIN][, backwards=true]}
   Go to the relative cartesian coordinates (X,Y) with the optional final orientation ORI.
3. relgoto{phi=PHI, dist=DIST[, ori=ORI][, margin=MARGIN][, backwards=true]}
   Same as 1., but with named arguments.

Parameters:
phi, dist: robot-relative polar coordinates of target point
x, y:      robot-relative cartesian coordinates of target point
ori:       orientation of robot at destination, radian offset from current value
           clock-wise positive
margin:    radius of a circle around the destination point, if the robot is within
           that circle the goto is considered final.
backwards: allow backwards driving for this command

The skill is S_RUNNING as long as the target can still be reached, S_FINAL if the target
has been reached (at least once, the robot might move afterwards for example if it did not
brake fast enough or if another robot crashed into this robot). The skill is S_FAILED if
the navigator started processing another goto message.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Constants
TIMEOUT_NAVI = 2.0

-- Jumpconditions
function jc_msgidfail(state)
   return state.fsm.vars.msgid ~= navigator:msgid()
end

function jc_navifail(state)
   return state.fsm.vars.msgid == 0 or not navigator:has_writer()
end

function jc_navifinal(state)
   --printf("msgid: %d/%d  final: %s", self.fsm.vars.msgid, navigator:msgid(), tostring(navigator:is_final()))
   return state.fsm.vars.msgid == navigator:msgid() and navigator:is_final()
end


-- States
fsm:define_states{
   export_to=_M,
   closure={navigator=navigator},

   {"INIT", JumpState},
   {"RELGOTO", JumpState},

   {"CHECK", JumpState},
   {"CHECK_MSGFAIL", JumpState},
}

-- Transitions
fsm:add_transitions{
   {"INIT", "FAILED", precond="not navigator:has_writer()", desc="No writer for navigator interface"},
   {"INIT", "FAILED", cond="vars.param_fail", desc="Invalid/insufficient parameteres"},
   {"INIT", "RELGOTO", cond=true, desc="Initialized"},

   {"RELGOTO", "CHECK", cond=true, desc="sent message"},

   {"CHECK", "CHECK_MSGFAIL", timeout=TIMEOUT_NAVI, desc="check msgid"},
   {"CHECK", "FAILED", cond=jc_navifail,  desc="Navigator failure"},
   {"CHECK", "FINAL",  cond=jc_navifinal, desc="Position reached"},

   {"CHECK_MSGFAIL", "FAILED", precond=jc_msgidfail, desc="msgid mismatch"},
   {"CHECK_MSGFAIL", "CHECK", precond=true, desc="msgid ok"}
}

function INIT:init()
   if self.fsm.vars.x ~= nil and self.fsm.vars.y ~= nil or
	  self.fsm.vars[1] ~= nil and self.fsm.vars[2] ~= nil then
      local x = self.fsm.vars.x or self.fsm.vars[1]
      local y = self.fsm.vars.y or self.fsm.vars[2]
      -- cartesian goto
	  self.fsm.vars.params = {x = x,
	                          y = y,
	                          ori = self.fsm.vars.ori or self.fsm.vars[3] or math.atan2(y, x)}
   elseif self.fsm.vars.phi ~= nil and self.fsm.vars.dist ~= nil then

	  -- polar goto
	  local phi = self.fsm.vars.phi
	  self.fsm.vars.params = {phi = phi,
	                          dist = self.fsm.vars.dist,
	                          ori = self.fsm.vars.ori or phi }
   else
	  self.fsm.vars.param_fail = true
   end

   self.fsm.vars.drive_mode = navigator:drive_mode()
end

function RELGOTO:init()
   if self.fsm.vars.backwards then
      navigator:msgq_enqueue_copy(navigator.SetDriveModeMessage:new(navigator.AllowBackward))
   end

   if self.fsm.vars.params.x ~= nil then
	  local m = navigator.CartesianGotoMessage:new(self.fsm.vars.params.x,
												   self.fsm.vars.params.y,
												   self.fsm.vars.params.ori)
	  printf("Sending CartesianGotoMessage(%f, %f, %f)", self.fsm.vars.params.x,
														 self.fsm.vars.params.y,
														 self.fsm.vars.params.ori)
	  self.fsm.vars.msgid = navigator:msgq_enqueue_copy(m)
   else
	  local m = navigator.PolarGotoMessage:new(self.fsm.vars.params.phi,
											   self.fsm.vars.params.dist,
											   self.fsm.vars.params.ori)
	  printf("Sending PolarGotoMessage(%f, %f, %f)", self.fsm.vars.params.phi,
													 self.fsm.vars.params.dist,
													 self.fsm.vars.params.ori)
	  self.fsm.vars.msgid = navigator:msgq_enqueue_copy(m)
   end

end

function CHECK:exit()
   if self.fsm.vars.backwards then
      --printf("resetting drive-mode to "..tostring(self.fsm.vars.drive_mode))
      navigator:msgq_enqueue_copy(navigator.SetDriveModeMessage:new(self.fsm.vars.drive_mode))
   end
end

function RELGOTO:reset()
   --printf("relgoto: sending stop");
   --navigator:msgq_enqueue_copy(navigator.StopMessage:new())
end






