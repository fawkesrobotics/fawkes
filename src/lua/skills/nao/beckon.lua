
----------------------------------------------------------------------------
--  beckon.lua - Beckoning skill for the Nao
--
--  Created: Thu Sep 11 16:45:14 2008 (UCT Visit South Africa)
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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
name               = "beckon"
fsm                = SkillHSM:new{name = name, start="GOTO_START"}
depends_skills     = {"servo"}
depends_interfaces = nil

documentation = [==[Beckon with the Nao.

This skill allows to beckon with the arms of the Nao.

Parameters:
arm  string   either "left", "right", or "both", defaults to "left"
num  number   number of times to beckon, defaults to 3
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Skill code
require("fawkes.fsm")

fsm:new_jump_state("BECKON")
fsm:new_jump_state("GOTO_START", servo, BECKON, FAILED)
fsm:new_jump_state("MOVE_DOWN", servo, BECKON, FAILED)
fsm:new_jump_state("MOVE_UP", servo, MOVE_DOWN, FAILED)

function GOTO_START:init()
   self.fsm.vars.num = self.fsm.vars.num or 3
   local arm = self.fsm.vars.arm or "left"

   self.args = self.args or {}
   self.args.time_sec = 1.0
   if arm == "left" or arm == "both" then
      self.fsm.vars.left = true
      self.args.l_shoulder_pitch = -0.9
      self.args.l_shoulder_roll  = 1.5
      self.args.l_elbow_yaw      = 0
      self.args.l_elbow_roll     = -0.5
   end
   if arm == "right" or arm == "both" then
      self.fsm.vars.right = true
      self.args.r_shoulder_pitch = -0.9
      self.args.r_shoulder_roll  = -1.5
      self.args.r_elbow_yaw      = 0
      self.args.r_elbow_roll     = 0.5
   end
end

function BECKON:init()
   self.fsm.vars.current_num = self.fsm.vars.current_num + 1
   if self.fsm.vars.current_num <= self.fsm.vars.num then
      self.dotattr.label = "\\N (" .. self.fsm.vars.current_num .. "/" .. self.fsm.vars.num .. ")";
      self.fsm:mark_changed()
   end
end

function BECKON:reset()
   self.fsm.vars.current_num = 0
end

BECKON:add_transition(MOVE_UP, "vars.current_num <= vars.num", "Movements left")
BECKON:add_transition(FINAL, "vars.current_num > vars.num", "All arm movements executed")

function MOVE_UP:init()
   self.args = { time_sec = 0.4, tolerance = 0.3 }
   if self.fsm.vars.left then
      self.args.l_elbow_roll = -1.5	 
   end
   if self.fsm.vars.right then
      self.args.r_elbow_roll =  1.5	 
   end
end

function MOVE_DOWN:init()
   self.args = { time_sec = 0.4, tolerance = 0.3 }
   if self.fsm.vars.left then
      self.args.l_elbow_roll = -0.5
   end
   if self.fsm.vars.right then
      self.args.r_elbow_roll =  0.5
   end
end
