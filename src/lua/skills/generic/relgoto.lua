
----------------------------------------------------------------------------
--  relgoto.lua
--
--  Created: Thu Aug 14 14:32:47 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--             2014  Tobias Neumann
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
name               = "relgoto"
fsm                = SkillHSM:new{name=name, start="CHECK_INPUT", debug=false}
depends_skills     = nil
depends_interfaces = {
   {v = "navigator", type="NavigatorInterface", id="Navigator"}
}

documentation      = [==[Move to base_link coords using navigator. Fail on navigator error only.
@param rel_x The target x coordinate
@param rel_y dito y
@param rel_ori dito orientation]==]

-- Initialize as skill module
skillenv.skill_module(_M)

function can_navigate()
   return navigator:has_writer()
end

--function cannot_navigate()
--   return not can_navigate()
--end

function target_reached()
   if navigator:msgid() == fsm.vars.goto_msgid then
      if navigator:is_final() and navigator:error_code() ~= 0 then
         return false
      end
      return navigator:is_final()
   end
   return false
end

function navi_failure()
   if navigator:msgid() == fsm.vars.goto_msgid then
      return navigator:is_final() and navigator:error_code() ~= 0
   end
   return false
end

function not_our_msgid()
   return navigator:msgid() ~= fsm.vars.goto_msgid and os.time() > fsm.vars.msgid_timeout
end

fsm:define_states{ export_to=_M,
   closure={can_navigate=can_navigate},
   {"CHECK_INPUT", JumpState},
   {"MOVING", JumpState}
}

fsm:add_transitions{
   { "CHECK_INPUT", "FAILED", cond="not can_navigate()", desc="Navigator not running" },
   { "MOVING", "FAILED", cond=navi_failure, desc="Navigator returned error" },
   { "MOVING", "MOVING", cond=not_our_msgid, desc="msgid workaround" },
   { "CHECK_INPUT", "MOVING", cond=can_navigate },
   { "MOVING", "FINAL", cond=target_reached },
}

function CHECK_INPUT:init()
      self.fsm.vars.x   = self.fsm.vars.x   or self.fsm.vars.rel_x or 0
      self.fsm.vars.y   = self.fsm.vars.y   or self.fsm.vars.rel_y or 0
      self.fsm.vars.ori = self.fsm.vars.ori or self.fsm.vars.rel_ori or math.nan
end

function MOVING:init()
   self.fsm.vars.msgid_timeout = os.time() + 1
   if not math.isnan( self.fsm.vars.ori ) then
      local msg_ori = navigator.SetOrientationModeMessage:new( navigator.OrientAtTarget )
      fsm.vars.ori_msgid = navigator:msgq_enqueue(msg_ori)
   end

   local msg = navigator.CartesianGotoMessage:new(
      self.fsm.vars.x,
      self.fsm.vars.y,
      self.fsm.vars.ori)
   fsm.vars.goto_msgid = navigator:msgq_enqueue(msg)
end


