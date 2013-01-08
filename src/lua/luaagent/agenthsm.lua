
----------------------------------------------------------------------------
--  agenthsm.lua - Hybrid State Machine for agents, closely related to FSM
--
--  Created: Fri Jan 02 17:30:28 2009
--  Copyright  2008-2010  Tim Niemueller [http://www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
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

--- Hybrid State Machine for agents.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

local hsmmod = require("fawkes.hsm")
local HSM    = hsmmod.HSM

AgentHSM = {}

--- Create new AgentHSM.
function AgentHSM:new(o)
   local f = HSM:new(o)
   setmetatable(f, self)
   setmetatable(self, HSM)
   self.__index = self

   f:clear_states()
   f.exit_state = "FINAL"
   f.fail_state = "FAILED"

   return f
end


--- Simple state generation not supported for AgentHSM.
-- Throws an error. Only jump states can be created for AgentHSMs.
function AgentHSM:new_state()
   error("Only jump states can be created for an AgentHSM")
end
