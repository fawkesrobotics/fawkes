
----------------------------------------------------------------------------
--  turn.lua - generic turn skill
--
--  Created: Tue Jan 27 17:35:08 2009
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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
name               = "turn"
fsm                = SkillHSM:new{name=name, start="TURN"}
depends_skills     = nil
depends_interfaces = {
   {v = "navigator", type = "NavigatorInterface"}
}

documentation      = [==[Turn on the spot.

Parameters:
angle: angle in rad to turn

]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("TURN")

function TURN:init()
   local angle    = self.fsm.vars.angle or 0
   local velocity = self.fsm.vars.angle or 0

   -- cartesian goto
   local m = navigator.TurnMessage:new(angle, velocity)
   self.fsm.vars.msgid = navigator:msgq_enqueue_copy(m)
end

function TURN:jumpcond_navifail()
   return self.fsm.vars.msgid == 0 or self.fsm.vars.msgid < navigator:msgid()
end

function TURN:jumpcond_navifinal()
   return self.fsm.vars.msgid == navigator:msgid() and navigator:is_final()
end

TURN.nowriter_interfaces = {navigator}

TURN:add_precond_trans(FAILED, JumpState.jumpcond_nowriter, "No navigator writer")
TURN:add_transition(FAILED, TURN.jumpcond_navifail, "Navigator failure")
TURN:add_transition(FINAL, TURN.jumpcond_navifinal, "Turn finished")
