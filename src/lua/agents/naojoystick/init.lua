
----------------------------------------------------------------------------
--  init.lua - Nao joystick agent
--
--  Created: Sat Aug 13 17:30:58 2011
--  Copyright  2011  Tim Niemueller [www.niemueller.de]
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
module(..., agentenv.module_init)

-- Crucial agent information
name               = "naojoystick"
fsm                = AgentHSM:new{name=name, debug=true, start="WAIT_START"}
depends_skills     = {}
depends_interfaces = {
   { v="joystick", type="JoystickInterface", id="Joystick" }
}

documentation      = [==[Agent to control Nao via joystick.]==]

-- Initialize as agent module
agentenv.agent_module(...)

local Skill = AgentSkillExecJumpState
local preds = require("predicates.nao")

-- Setup FSM
fsm:define_states{ export_to=_M,
   closure={joystick=joystick, JoystickInterface=JoystickInterface, preds=preds},
   {"WAIT_START", JumpState},
   {"CONTROL", JumpState},
   {"STOP_KICK_LEFT", Skill, skills={{"stop"}},
      final_to="KICK_LEFT", fail_to="CONTROL"},
   {"KICK_LEFT", Skill, skills={{"kick", leg="left"}},
      final_to="CONTROL", fail_to="CONTROL"},
   {"STOP", Skill}
}

fsm:add_transitions{
   {"WAIT_START", "CONTROL", "preds.short_button", desc="Button pressed"},
   {"CONTROL", "STOP_KICK_LEFT",
    "joystick:pressed_buttons() == JoystickInterface.BUTTON_1"},
   {"CONTROL", "WAIT_START", "preds.short_button", desc="Button pressed"},
}
