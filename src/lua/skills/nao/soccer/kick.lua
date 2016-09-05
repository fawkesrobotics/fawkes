
----------------------------------------------------------------------------
--  kick.lua - Skill to kick the ball
--
--  Created: Fri Jan 23 18:17:22 2009
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

-- Crucial skill information
name               = "kick"
fsm                = SkillHSM:new{name=name, start="KICK"}
depends_skills     = nil
depends_interfaces = {
   {v = "naomotion", id = "NaoQi Motion", type = "HumanoidMotionInterface"}
}

documentation      = [==[Kick the ball.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("KICK")
WAIT = WaitState:new{name="WAIT", fsm=fsm, next_state=FINAL,
		     time_sec = 13, labeltime=true}
fsm:add_state(WAIT)

function KICK:jumpcond_noleg()
   return self.fsm.vars.leg ~= "left"
      and self.fsm.vars.leg ~= "right"
end

function KICK:init()
   if self.fsm.vars.leg == "left" then
      naomotion:msgq_enqueue_copy(naomotion.KickMessage:new(naomotion.LEG_LEFT, 1.0))
   elseif self.fsm.vars.leg == "right" then
      naomotion:msgq_enqueue_copy(naomotion.KickMessage:new(naomotion.LEG_RIGHT, 1.0))
   end
end

KICK.nowriter_interfaces = {naomotion}

KICK:add_precond_trans(FAILED, JumpState.jumpcond_nowriter, "No writer for interfaces")
KICK:add_precond_trans(FAILED, KICK.jumpcond_noleg, "No leg defined")
KICK:add_transition(WAIT,   JumpState.jumpcond_true, "Message enqueued")
