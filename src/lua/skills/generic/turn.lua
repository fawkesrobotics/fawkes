
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

-- Jumpconditions

function jumpcond_navifail(state)
   return state.fsm.vars.msgid == 0 or state.fsm.vars.msgid < navigator:msgid()
end

function jumpcond_navifinal(state)
   return state.fsm.vars.msgid == navigator:msgid() and navigator:is_final()
end

-- States
fsm:define_states{
   export_to=_M,

   {"TURN", JumpState}
}

-- set interfaces to be checked for "no writer"
TURN.nowriter_interfaces = {navigator}

-- Transitions
fsm:add_transitions{
   {"TURN", "FAILED", cond=JumpState.jumpcond_nowriter, desc="No writer for navigator interface", precond_only=true},
   {"TURN", "FAILED", cond=jumpcond_navifail, desc="Navigator failure"},
   {"TURN", "FINAL", cond=jumpcond_navifinal, desc="Turn finished"}
}

function TURN:init()
   local angle    = self.fsm.vars.angle or 0
   local velocity = self.fsm.vars.velocity or 0

   -- cartesian goto
   local m = navigator.TurnMessage:new(angle, velocity)
   self.fsm.vars.msgid = navigator:msgq_enqueue_copy(m)
end


