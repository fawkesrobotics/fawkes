
----------------------------------------------------------------------------
--  spin.lua - spin on the spot
--
--  Copyright  2014  Tim Niemueller [www.niemueller.de]
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
name               = "spin"
fsm                = SkillHSM:new{name=name, start="SPIN", debug=true}
depends_skills     = nil
depends_interfaces = {
    {v = "motor", type = "MotorInterface", id="Motor" }
}

documentation      = [==[Spin on the spot, e.g. for odometry testing.
@param omega angular speed. Warning, this is not checked, choose a value
appropriate for your robot. If not set defaults to 1.0.
]==]

-- Initialize as skill module
skillenv.skill_module(_M )

function send_transrot(vx, vy, omega)
   local oc  = motor:controller()
   local ocn = motor:controller_thread_name()
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new())
   motor:msgq_enqueue_copy(motor.TransRotMessage:new(vx, vy, omega))
   motor:msgq_enqueue_copy(motor.AcquireControlMessage:new(oc, ocn))
end


fsm:define_states{ export_to=_M,
   closure={motor=motor},
   {"SPIN", JumpState},
}

fsm:add_transitions{
   {"SPIN", "FAILED", precond="not motor:has_writer()"},
}

function SPIN:init()
   self.fsm.vars.omega = self.fsm.vars.omega or 1.0
end

function SPIN:loop()
   send_transrot(0, 0, self.fsm.vars.omega)
end

function SPIN:exit()
   send_transrot(0, 0, 0)
end

