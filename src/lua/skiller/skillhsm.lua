
----------------------------------------------------------------------------
--  skillhsm.lua - Hybrid State Machine for skills, closely related to FSM
--
--  Created: Mon Dec 22 11:53:39 2008
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
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

require("fawkes.modinit")

--- Hybrid State Machine for skills.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)
local hsmmod = require("fawkes.hsm")
local jsmod = require("fawkes.fsm.jumpstate")
local sjsmod = require("skiller.skill_jumpstate")
local subfsmjsmod = require("fawkes.fsm.subfsmjumpstate")

local HSM         = hsmmod.HSM
JumpState         = jsmod.JumpState
SkillJumpState    = sjsmod.SkillJumpState
SubFSMJumpState   = subfsmjsmod.SubFSMJumpState

SkillHSM = {}

--- Constructor.
-- @param o object pre-initializer, must be a table.
function SkillHSM:new(o)
   local f = HSM:new(o)
   setmetatable(o, self)
   setmetatable(self, HSM)
   self.__index = self

   if not f.no_default_states then
      f.exit_state = "FINAL"
      f.fail_state = "FAILED"
      f:define_states{ export_to = o.export_to,
                       {"FINAL", JumpState}, {"FAILED", JumpState}}
      f.state_changed = true
   end

   return f
end
