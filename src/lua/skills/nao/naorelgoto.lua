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
name = "relgoto"
fsm = SkillHSM:new{name = name, start = "RELGOTO", debug = true}
depends_skills = {"servo"}
depends_interfaces = {
    {v = "naomotion", id = "NaoQi Motion", type = "HumanoidMotionInterface"},
    {v = "navigator", id = "Navigator", type = "NavigatorInterface"}
}

local p = require("predicates.soccer.general")
local np = require("predicates.nao")

documentation = [==[Relative goto skill - tries to keep the ball insight.
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
2. relgoto{x=X, y=Y[, ori=ORI][, margin=MARGIN]}
   Go to the relative cartesian coordinates (X,Y) with the optional final orientation ORI.
3. relgoto{phi=PHI, dist=DIST[, ori=ORI], [margin=MARGIN]}
   Same as 1., but with named arguments.

Parameters:
phi, dist: robot-relative polar coordinates of target point
x, y:      robot-relative cartesian coordinates of target point
ori:       orientation of robot at destination, radian offset from current value
           clock-wise positive
margin:    radius of a circle around the destination point, if the robot is within
           that circle the goto is considered final.

The skill is S_RUNNING as long as the target can still be reached, S_FINAL if the target
has been reached (at least once, the robot might move afterwards for example if it did not
brake fast enough or if another robot crashed into this robot). The skill is S_FAILED if
the navigator started processing another goto message.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("RELGOTO")

function RELGOTO:init()
    if navigator:has_writer() then
        local vm = navigator.MaxVelocityMessage:new(1.0)
        navigator:msgq_enqueue_copy(vm)

        if self.fsm.vars.x ~= nil and self.fsm.vars.y ~= nil or self.fsm.vars[1] ~=
            nil and self.fsm.vars[2] ~= nil then
            -- cartesian goto
            local x = self.fsm.vars.x or self.fsm.vars[1]
            local y = self.fsm.vars.y or self.fsm.vars[2]
            local ori = self.fsm.vars.ori or self.fsm.vars[3] or
                            math.atan2(y, x)
            local m = navigator.CartesianGotoMessage:new(x, y, ori)
            printf("Sending CartesianGotoMessage(%f, %f, %f)", x, y, ori)
            self.fsm.vars.msgid = navigator:msgq_enqueue_copy(m)
        elseif self.fsm.vars.phi ~= nil and self.fsm.vars.dist ~= nil then
            -- polar goto
            local phi, dist = self.fsm.vars.phi, self.fsm.vars.dist
            local ori = self.fsm.vars.ori or phi
            local m = navigator.PolarGotoMessage:new(phi, dist, ori)
            printf("Sending PolarGotoMessage(%f, %f, %f)", phi, dist, ori)
            self.fsm.vars.msgid = navigator:msgq_enqueue_copy(m)
        else
            self.param_fail = true
        end
    end
    self.wait_start = 1
    print("end init");
end

function RELGOTO:loop()
    self.wait_start = self.wait_start + 1

    if p.ball_visible then
        local m = naomotion.YawPitchHeadMessage:new(np.head_yaw_to_center_ball,
                                                    np.head_pitch_to_center_ball,
                                                    0.3)
        naomotion:msgq_enqueue_copy(m)
    end
end

function RELGOTO:reset()
    -- printf("relgoto: sending stop");
    -- navigator:msgq_enqueue_copy(navigator.StopMessage:new())
end

function RELGOTO:jumpcond_paramfail() return self.param_fail end

function RELGOTO:jumpcond_navifail()
    return (self.fsm.vars.msgid == 0 or
               (self.fsm.vars.msgid ~= navigator:msgid() and self.wait_start > 2) or
               not navigator:has_writer() or self.failed)
end

function RELGOTO:jumpcond_navifinal()
    -- printf("msgid: %d/%d  final: %s", self.fsm.vars.msgid, navigator:msgid(), tostring(navigator:is_final()))
    return self.fsm.vars.msgid == navigator:msgid() and navigator:is_final()
end

RELGOTO:add_transition(FAILED, RELGOTO.jumpcond_paramfail,
                       "Invalid/insufficient parameters")
RELGOTO:add_transition(FAILED, RELGOTO.jumpcond_navifail, "Navigator failure")
RELGOTO:add_transition(FINAL, RELGOTO.jumpcond_navifinal, "Position reached")
