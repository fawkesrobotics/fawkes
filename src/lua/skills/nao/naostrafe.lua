
----------------------------------------------------------------------------
--  naostrafe.lua - nao strafe skill
--
--  Created: 
--  Copyright  2009  Patrick Podbregar
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

local p = require("predicates.soccer.general")
local np = require("predicates.nao")

-- Crucial skill information
name               = "naostrafe"
fsm                = SkillHSM:new{name=name, start="STRAFE"}
depends_skills     = nil
depends_interfaces = {
   {v = "naomotion", id = "NaoQi Motion", type = "HumanoidMotionInterface"}
}

documentation      = [==[Straves and tries to keep the ball in sight
parameters: distance (> 0.0 = left ; < 0.0 = right)
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- states
fsm:new_jump_state("STRAFE")

local motion_started = false

function STRAFE:init()
   local distance = self.fsm.vars.distance or self.fsm.vars[1]
   local samples = self.fsm.vars.samples or self.fsm.vars[2] or 0
   local m = naomotion.WalkSidewaysMessage:new(distance,0)
   self.fsm.vars.msgid = naomotion:msgq_enqueue_copy(m)
end

function STRAFE:loop()
   if p.ball_visible then
     --[[local m = naomotion.YawPitchHeadMessage:new(
                  np.head_yaw_to_center_ball,
                  np.head_pitch_to_center_ball,
                  0.03)
     naomotion:msgq_enqueue_copy(m)--]]
   end
   if(not motion_started) then
      if(naomotion:is_moving()) then
         motion_started = true
      end
   end
end

function STRAFE:jumpcond_motionfail()
   return self.fsm.vars.msgid == 0 or self.fsm.vars.msgid < naomotion:msgid()
end

function STRAFE:jumpcond_motionfinal()
   return motion_started and not naomotion:is_moving()
end

function FINAL:init()
   motion_started = false
end

STRAFE.nowriter_interfaces = {naomotion}

STRAFE:add_transition(FAILED, JumpState.jumpcond_nowriter, "no naomotion writer")
STRAFE:add_transition(FAILED, STRAFE.jumpcond_motionfail, "naomotion failure")
STRAFE:add_transition(FINAL, STRAFE.jumpcond_motionfinal, "strafe final")

