
----------------------------------------------------------------------------
--  fake_relgoto.lua - Imitate a relgoto skill with chance of error
--
--  Created: Thu Feb 28 20:25:46 2019 +0100
--  Copyright  2018-2019  Tim Niemueller [www.niemueller.org]
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
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {}
depends_interfaces = {
}

documentation      = [==[Pretend to move to a given place.

@param x  Relative X coordinate (base_link frame)
@param y  Relative Y coordinate (base_link frame)
@param ori  Relative orientation (base_link frame)
@param error_prob Probability of an error in [0..1].
@param duration Time this should take
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

require("fawkes.modinit")

function chance_of_error(self)
	 local error_prob = self.fsm.vars.error_prob or 0.0
	 return math.random() < error_prob
end

function check_params(self)
   return self.fsm.vars.x == nil or self.fsm.vars.y == nil
end

fsm:define_states{ export_to=_M,
  {"INIT",    JumpState},
  {"WAIT",    JumpState},
  {"CHANCE",  JumpState}
}

fsm:add_transitions{
	 {"INIT", "FAILED", cond=check_params, desc="Invalid coordinates"},
	 {"INIT", "WAIT", cond=true},
	 {"WAIT", "CHANCE", timeout=10, desc="Wait for duration"},
	 {"CHANCE", "FAILED", cond=chance_of_error, desc="Error occured"},
	 {"CHANCE", "FINAL", cond=true}
}

function INIT:init()
	 WAIT.timeout_time = self.fsm.vars.duration or 1.0
	 WAIT.transitions[1].description = "Timeout (".. WAIT.timeout_time .." sec)"
end

function WAIT:init()
end
