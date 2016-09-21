
----------------------------------------------------------------------------
--  getup.lua - Get up skill
--
--  Created: Wed Sep 11 11:32:01 2008 (Cape Town UCT Trip)
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
name               = "getup"
fsm                = SkillHSM:new{name=name, start="GETUP"}
depends_skills     = nil
depends_interfaces = {
   {v = "naomotion", id = "NaoQi Motion", type = "HumanoidMotionInterface"}
}

documentation      = [==[Get the robot up to standing position.
getup(time_sec)
getup{time_sec=sec}
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("GETUP")
local WAIT = WaitState:new{name="WAIT", fsm=fsm, next_state=FINAL, labeltime=true}
fsm:add_state(WAIT)

function GETUP:init()
   self.fsm.vars.time_sec = tonumber(self.fsm.vars.time_sec) or
			    tonumber(self.fsm.vars[1]) or 3.0
   naomotion:msgq_enqueue_copy(naomotion.GetUpMessage:new())
end

function GETUP:jumpcond_enqueued() return true, self.fsm.vars.time_sec end

GETUP.nowriter_interfaces = {naomotion}

GETUP:add_precond_trans(FAILED, JumpState.jumpcond_nowriter, "No writer for naomotion interface")
GETUP:add_transition(WAIT, GETUP.jumpcond_enqueued, "Message enqueued")
