
----------------------------------------------------------------------------
--  transrot.lua - generic trans/rot via MotorInterface
--
--  Created: Fri Jan 30 15:41:47 2009
--  Copyright  2008-2009  Tim Niemueller
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
name               = "transrot"
fsm                = SkillHSM:new{name=name, start="TRANSROT", debug=false}
depends_skills     = nil
depends_interfaces = {
   {v = "motor", type = "MotorInterface"}
}

documentation      = [==[Trans/rot via MotorInterface.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:define_states{
   export_to=_M,
   closure={motor=motor},

   {"TRANSROT",   JumpState},
   {"WAIT_ABORT", JumpState},
   {"WAIT_TIME",  JumpState}
}

-- Transitions
fsm:add_transitions{
   {"TRANSROT", "FAILED", cond="not motor:has_writer()", desc="No writer for motor", precond_only=true},
   {"TRANSROT", "FINAL", cond="vars.vx == 0.0 and vars.vy == 0.0 and vars.omega == 0.0", desc="Stop"},
   {"TRANSROT", "WAIT_ABORT", cond="vars.time_sec == nil", desc="Keep going"},
   {"TRANSROT", "WAIT_TIME", cond="vars.time_sec ~= nil", desc="Wait Time"},
   {"WAIT_TIME", "FINAL", timeout=1, desc="Time over"},
}

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end

function TRANSROT:init()
   self.fsm.vars.vx    = self.fsm.vars.vx or 0.0
   self.fsm.vars.vy    = self.fsm.vars.vy or 0.0
   self.fsm.vars.omega = self.fsm.vars.omega or 0.0
   send_transrot(self.fsm.vars.vx, self.fsm.vars.vy, self.fsm.vars.omega)
end

function WAIT_ABORT:exit()
   send_transrot(0, 0, 0)
end

function WAIT_TIME:init()
   self.time_sec = self.fsm.vars.time_sec
end

function WAIT_TIME:exit()
   send_transrot(0, 0, 0)
end
