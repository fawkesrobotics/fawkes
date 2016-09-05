
----------------------------------------------------------------------------
--  stop.lua - Stop any motion, immediately!
--
--  Created: Mon Mar 23 15:38:42 2009
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
name               = "stop"
fsm                = SkillHSM:new{name=name, start="STOP"}
depends_skills     = nil
depends_interfaces = {
   {v = "naomotion", type = "HumanoidMotionInterface", id = "NaoQi Motion"}
}

documentation      = [==[Stop any motion.
This stops currently running motion patterns and interpolated angle movements,
it especially stops walking motion and servo skill motion.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("STOP")

function STOP:init()
   naomotion:msgq_enqueue_copy(naomotion.StopMessage:new())
end

STOP.nowriter_interfaces = {naomotion}

STOP:add_precond_trans(FAILED, JumpState.jumpcond_nowriter, "No writer for interface naomotion")
STOP:add_transition(FINAL, true, "Message enqueued")
