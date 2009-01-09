
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
fsm                = SkillHSM:new{name=name, start="INITIAL",
				  no_default_states=true, exit_state="FINAL"}

documentation      = [==[Simple test skill for HSMs.
hsmtest()
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("INITIAL")
fsm:new_jump_state("FINAL")
fsm:new_jump_state("LOOP")
WS2 = WaitState:new{name="WS2", fsm=fsm, next_state=FINAL, time_sec=2, labeltime=true}
WS1 = WaitState:new{name="WS1", fsm=fsm, next_state=LOOP, time_sec=1, labeltime=true}
fsm:add_state(WS2)
fsm:add_state(WS1)

function LOOP:reset()
   self.loopcount = 0
   self.loops     = 5
   self.dotattr.label = "\\N 0/" .. self.loops
end
function LOOP:jumpcond_loop()      return self.loopcount < 5            end
function LOOP:jumpcond_abortloop() return self.loopcount >= 5           end
function LOOP:exit()               self.loopcount = self.loopcount + 1  end
function LOOP:init()
   self.dotattr.label = "\\N " .. tostring(self.loopcount) .. "/" .. self.loops
end

local it1 = INITIAL:add_transition(WS1, function () return true end, "Start FSM")
local lt1 = LOOP:add_transition(WS1, LOOP.jumpcond_loop, "Keep looping")
local lt2 = LOOP:add_transition(WS2, LOOP.jumpcond_abortloop, "Loops done")

-- Cosmetics
WS1:get_transitions(LOOP).dotattr = { labelangle = 15, labeldistance = 4, labeloffsetx = -10 }
WS2:get_transitions(FINAL).dotattr = { labelangle = -50 }
lt1.dotattr = { labelangle = 35, labeldistance = 3, labeloffsetx = -25, labeloffsety = -25 }
lt2.dotattr = { labelangle = 10, labeldistance = 3 }
it1.dotattr = { labelangle = -50 }
