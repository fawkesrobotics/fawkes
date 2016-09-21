
----------------------------------------------------------------------------
--  standup.lua - Standup skill
--
--  Created: Mon Jan 19 19:36:13 2009
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
name               = "standup"
fsm                = SkillHSM:new{name=name, start="TURNON"}
depends_skills     = {"servo", "getup"}
depends_interfaces = {
   {v = "naomotion", id = "NaoQi Motion", type = "HumanoidMotionInterface"}
}

documentation      = [==[Make the robot standup.]==]

local np = require("predicates.nao")

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("STANDUP")
fsm:new_jump_state("FROM_BACK")
fsm:new_jump_state("FROM_FRONT")
fsm:new_jump_state("TURNON", servo, STANDUP, FAILED, {enable=true})
fsm:new_jump_state("GETUP", getup, FINAL, FAILED)
fsm:new_wait_state("WAIT", FINAL, {time_sec = 12.5, labeltime=true})

function TURNON:jumpcond_notlying()
   return self.fsm.vars.from_pos ~= "front"
      and self.fsm.vars.from_pos ~= "back"
      and not np.lying
end

TURNON:add_transition(GETUP, TURNON.jumpcond_notlying, "Not lying")

function STANDUP:init()
   self.fsm.vars.from_pos = self.fsm.vars.from_pos or ""
end

function STANDUP:jumpcond_back()
   return self.fsm.vars.from_pos == "back" or np.lying_on_back
end

function STANDUP:jumpcond_front()
   return self.fsm.vars.from_pos == "front" or np.lying_on_front
end

function FROM_FRONT:init()
   naomotion:msgq_enqueue_copy(naomotion.StandupMessage:new(naomotion.STANDUP_FRONT))
end

function FROM_BACK:init()
   naomotion:msgq_enqueue_copy(naomotion.StandupMessage:new(naomotion.STANDUP_BACK))
end


STANDUP.nowriter_interfaces = {naomotion}

STANDUP:add_transition(FAILED, JumpState.jumpcond_nowriter, "No writer for interfaces")
STANDUP:add_transition(FROM_FRONT, STANDUP.jumpcond_front, "Lying on front")
STANDUP:add_transition(FROM_BACK, STANDUP.jumpcond_back, "Lying on back")

FROM_FRONT:add_transition(WAIT, true, "Message enqueued")
FROM_BACK:add_transition(WAIT, true, "Message enqueued")

-- Cosmetics for graph output
STANDUP:get_transitions(FROM_FRONT).dotattr = { labeloffsety = 10, labelrotate = -10 }
STANDUP:get_transitions(FROM_BACK).dotattr = {labelrotate = -4 }
STANDUP:get_transitions(FAILED).dotattr = {labelrotate = 12, labeloffsety = -5 }
TURNON:get_transitions(FAILED).dotattr = {labeloffsety = -5 }
TURNON:get_transitions(STANDUP).dotattr = {labeloffsety = 18, labelrotate = -14,
					   labelangle=100, labeldistance=1.5}
FROM_FRONT:get_transitions(WAIT).dotattr = { labeloffsety = -5, labelrotate = 10 }
FROM_BACK:get_transitions(WAIT).dotattr = { labeloffsety = -5 }
GETUP:get_transitions(FINAL).dotattr = { labeloffsety = -5 }

