
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

   f:clear_states()

   return f
end


--- Clear all states.
-- Removes all states. If no_default_states is not set a FINAL and FAILED state
-- are added.
function SkillHSM:clear_states()
   self.states = {}
   if not self.no_default_states then
      self.exit_state = "FINAL"
      self.fail_state = "FAILED"

      local es = SkillJumpState:new{name = "FINAL", fsm = self}
      local fs = SkillJumpState:new{name = "FAILED", fsm = self}

      self.states["FINAL"] = es
      self.states["FAILED"] = fs

      if self.export_states_to_parent then
	 local e = getfenv(2)
	 if e == _M then
	    e = getfenv(3)
	 end
	 e["FINAL"] = es
	 e["FAILED"] = fs
      end
   end
   self.state_changed = true
end
