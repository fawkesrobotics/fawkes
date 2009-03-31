
----------------------------------------------------------------------------
--  hsmtest.lua - Simple test for HSM/JumpState
--
--  Created: Wed Dec 31 15:16:36 2008
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
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
module(..., skillenv.module_init)

-- Crucial skill information
name               = "hsmtest"
fsm                = SkillHSM:new{name=name, start="INITIAL"}

documentation      = [==[Simple test skill for HSMs.
hsmtest()
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("INITIAL")
fsm:new_jump_state("FINAL")
fsm:new_jump_state("LOOP")
WS2 = WaitState:new{name="WS2", fsm=fsm, next_state="SUBFSM", time_sec=2, labeltime=true}
WS1 = WaitState:new{name="WS1", fsm=fsm, next_state=LOOP, time_sec=1, labeltime=true}
fsm:add_state(WS2)
fsm:add_state(WS1)

local subfsm = SkillHSM:new{name="TestSubFSM", start="SUBINIT",
			     export_states_to_parent=false}
SUBINIT = WaitState:new{name="SUBINIT", fsm=subfsm, next_state="SUB_1", time_sec=2, labeltime=true}
subfsm:add_state(SUBINIT)
SUB_1 = subfsm:new_jump_state("SUB_1")
SUB_1:add_transition(subfsm.states["FINAL"], "math.random() >= 0.5")
SUB_1:add_transition(subfsm.states["FAILED"], true, "Unconditional")

SUBFSM = SubFSMJumpState:new{name="SUBFSM", fsm=fsm, subfsm=subfsm,
			     exit_to="FINAL", fail_to="FAILED"}
fsm:add_state(SUBFSM)

function LOOP:reset()
   self.loopcount = 0
   self.loops     = 1
   self.dotattr.label = "\\N 0/" .. self.loops
end
function LOOP:exit()
   self.loopcount = self.loopcount + 1
end
function LOOP:init()
   self.dotattr.label = "\\N " .. tostring(self.loopcount) .. "/" .. self.loops
end

local it1 = INITIAL:add_transition(WS1, true, "Start FSM")
local lt1 = LOOP:add_transition(WS1, "self.loopcount <  self.loops", "Keep looping")
local lt2 = LOOP:add_transition(WS2, "self.loopcount >= self.loops", "Loops done")


-- Cosmetics
--WS1:get_transition(LOOP).dotattr = { labelangle = 15, labeldistance = 4, labeloffsetx = -10, labeloffsety=-15, labelrotate=-20 }
--WS2:get_transition(FINAL).dotattr = { labelangle = -50 }
--lt1.dotattr = { labelangle = 35, labeldistance = 3, labeloffsetx = -15, labeloffsety = 30, labelrotate=-45 }
--lt2.dotattr = { labelangle = 10, labeldistance = 3 }
--it1.dotattr = { labelangle = -50 }
