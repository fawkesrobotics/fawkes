
----------------------------------------------------------------------------
--  park.lua - Park position skill
--
--  Created: Wed Sep 11 11:41:44 2008 (Cape Town UCT Trip)
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
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
name               = "park"
fsm                = SkillHSM:new{name=name, start="PARK"}
depends_skills     = nil
depends_interfaces = {
   {v = "naomotion", id = "NaoQi Motion", type = "HumanoidMotionInterface"},
   {v = "naostiffness", id = "Nao Joint Stiffness", type = "NaoJointStiffnessInterface"}
}

documentation      = [==[Park the robot.
park(time_sec,servos_off)
park{time_sec=sec,servos_off=true/false}
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("PARK")
fsm:new_jump_state("DONE")
local WAIT = WaitState:new{name="WAIT", fsm=fsm, next_state=DONE, labeltime=true}
fsm:add_state(WAIT)

PARK.nowriter_interfaces = {naomotion}
function PARK:init()
   self.fsm.vars.time_sec = tonumber(self.fsm.vars.time_sec) or
			    tonumber(self.fsm.vars[1]) or 3.0
   naomotion:msgq_enqueue_copy(naomotion.ParkMessage:new())
end

function PARK:jumpcond_enqueued() return true, self.fsm.vars.time_sec end

function DONE:init()
   if self.fsm.vars.servos_off~=nil and self.fsm.vars.servos_off then
      naostiffness:msgq_enqueue_copy(naostiffness.SetBodyStiffnessMessage:new(0, 0))
   end
end

PARK:add_transition(FAILED, JumpState.jumpcond_nowriter, "No writer for naomotion interface")
PARK:add_transition(WAIT, PARK.jumpcond_enqueued, "Message enqueued")
DONE:add_transition(FINAL, true, "Park done")
