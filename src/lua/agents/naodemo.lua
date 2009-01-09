
----------------------------------------------------------------------------
--  naodemo.lua - Simple luaagent demo for Nao using HSM/JumpState
--
--  Created: Fri Jan 02 14:42:31 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
--
--  $Id$
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
module(..., agentenv.module_init)

-- Crucial skill information
name               = "naodemo"
fsm                = AgentHSM:new{name=name, debug=true, start="WAIT_FOR_BUTTON",
				  exit_state="FINAL", fail_state="FAILED"}
depends_interfaces = {
   { v = "chestbutton", id = "Nao ChestButton", type = "SwitchInterface" }
}

documentation      = [==[Simple demo agent for Nao.]==]

-- Initialize as agent module
agentenv.agent_module(...)

-- Constants
local MOVE_POSITIONS = { {x = 0.3, y = 0.0},
			 {x = -0.3, y = 0.3},
			 {x = 0, y = -0.3, ori=3.93},
			 {x = 0.000001, y = 0, ori = -2.36 } }


local RETRACT_ARM_SKILL = {{"servo", {l_shoulder_pitch = 0.9, l_shoulder_roll = 0.15,
				      l_elbow_yaw = -1.2, l_elbow_roll = -1.1,
				      r_shoulder_pitch = 0.9, r_shoulder_roll = -0.15,
				      r_elbow_yaw = 1.2, r_elbow_roll = 1.1,
				      tolerance = 0.3} }}

-- States
fsm:new_jump_state("FINAL")
fsm:new_jump_state("FAILED")

fsm:new_jump_state("WAIT_FOR_BUTTON")
fsm:new_jump_state("MOVE")
fsm:new_skill_state("RETRACT_ARM_1", RETRACT_ARM_SKILL, MOVE, FAILED)
fsm:new_skill_state("HELLO_AUDIENCE",
		    {{"say", {text="Hello audience! "..
			      "I'm Nao, a humanoid robot for playing soccer autonomously. " ..
			      "Currently I'm learning how to walk."}},
		     {"beckon"}}, RETRACT_ARM_1, FAILED)
fsm:new_skill_state("GETUP", {{"servo", {enable=true}},
			      {"getup"}}, HELLO_AUDIENCE, FAILED)
fsm:new_skill_state("TURNOFF", {{"servo", {enable=false}}}, FINAL, FAILED)
fsm:new_skill_state("GOTOXYO", {{"relgoto"}}, MOVE, FAILED)
fsm:new_skill_state("PARK", {{"park"}}, TURNOFF, FAILED)
fsm:new_skill_state("RETRACT_ARM_2", RETRACT_ARM_SKILL, PARK, FAILED)
fsm:new_skill_state("GOODBYE_BECKON", {{"beckon", {arm="left"}}}, RETRACT_ARM_2, FAILED)
fsm:new_skill_state("GOODBYE_SAY", {{"say", {text="Good bye. I hope you enjoyed the show."}}},
		    GOODBYE_BECKON, FAILED)

-- Code
function jumpcond_chestbut_short(state)
   return chestbutton:short_activations() > 0
end

for _,s in ipairs({WAIT_FOR_BUTTON, FINAL, FAILED}) do
   s:add_transition(GETUP, jumpcond_chestbut_short,
		    "ChestButton short activation detected")
end

function MOVE:init()
   if self.current_move <= #MOVE_POSITIONS then
      self.dotattr.label = "\\N " .. self.current_move .. "/" .. #MOVE_POSITIONS
   end
end

function MOVE:jumpcond_gotoxyo()
   if self.current_move <= #MOVE_POSITIONS then
      local nextpos = MOVE_POSITIONS[self.current_move]
      self.current_move = self.current_move + 1
      return true, {x = nextpos.x, y = nextpos.y, ori = nextpos.ori}
   else
      return false
   end
end

function MOVE:reset()
   self.current_move = 1
   self.dotattr.label = "\\N 0/" .. #MOVE_POSITIONS
end

-- This is only needed because we allow to start over from FINAL/FAILURE
function FINAL:exit()
   self.fsm:reset_trace()
   MOVE:reset()
end
FAILED.exit = FINAL.exit

MOVE:add_transition(GOTOXYO, MOVE.jumpcond_gotoxyo, "Move positions left")
MOVE:add_transition(GOODBYE_SAY,
		    function (self) return self.current_move > #MOVE_POSITIONS end,
		    "All move positions done")

-- Cosmetics for graph output
WAIT_FOR_BUTTON:get_transitions(GETUP).dotattr = { labeloffsetx = 50 }
GETUP:get_transitions(FAILED).dotattr = { labelrotate = 90 }
GETUP:get_transitions(HELLO_AUDIENCE).dotattr = { labeloffsety = 15 }
RETRACT_ARM_1:get_transitions(FAILED).dotattr = { labelrotate = 90, labeloffsetx = -10 }
RETRACT_ARM_1:get_transitions(MOVE).dotattr = { labeloffsetx = 20, labeloffsety = -10 }
GOTOXYO:get_transitions(FAILED).dotattr = { labelrotate = 65, labeloffsety = 20 }
GOTOXYO:get_transitions(MOVE).dotattr = { labelrotate=16, labeloffsetx = -60, labeloffsety = -50 }
GOODBYE_SAY:get_transitions(FAILED).dotattr = { labelrotate = 90, labeloffsetx = -10, labeloffsety = -60 }
GOODBYE_SAY:get_transitions(GOODBYE_BECKON).dotattr = { labeloffsetx = 20, labeloffsety = -10 }
GOODBYE_BECKON:get_transitions(FAILED).dotattr = { labelrotate = 90, labeloffsetx = 25, labeloffsety = -100 }
GOODBYE_BECKON:get_transitions(RETRACT_ARM_2).dotattr = { labeloffsetx = 30}
RETRACT_ARM_2:get_transitions(FAILED).dotattr = { labelrotate = -56, labeloffsetx = 30, labeloffsety = -20 }
RETRACT_ARM_2:get_transitions(PARK).dotattr = { labeloffsetx = 30 }
PARK:get_transitions(FAILED).dotattr = { labelrotate = -70, labeloffsety = 60 }
PARK:get_transitions(TURNOFF).dotattr = { labeloffsetx = 5, labeloffsety = -25 }
TURNOFF:get_transitions(FAILED).dotattr = { labelrotate = -36, labeloffsetx = -30, labeloffsety = 40 }
TURNOFF:get_transitions(FINAL).dotattr = { labelrotate=25, labeloffsetx = -30, labeloffsety = -35 }
FINAL:get_transitions(GETUP).dotattr = { labelrotate=90, labeloffsetx = 20, labeloffsety = -195 }
HELLO_AUDIENCE:get_transitions(FAILED).dotattr = { labelrotate  = 90, labeloffsetx = 40, labeloffsety = 60 }
HELLO_AUDIENCE:get_transitions(RETRACT_ARM_1).dotattr = { labeloffsetx = 40 }
MOVE:get_transitions(GOTOXYO).dotattr = { labelrotate = 30, labeloffsetx = -60, labeloffsety = -20 }
MOVE:get_transitions(GOODBYE_SAY).dotattr = { labelrotate = 20, labeloffsetx = -165, labeloffsety = -75 }
FAILED:get_transitions(GETUP).dotattr = { labelrotate=90, labeloffsetx = 50, labeloffsety = -200}
