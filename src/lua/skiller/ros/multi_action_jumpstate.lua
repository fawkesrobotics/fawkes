
------------------------------------------------------------------------
--  multi_action_jumpstate.lua - Execute multiple ROS actions
--
--  Created: Fri Sep 10 11:43:10 2010
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--
------------------------------------------------------------------------

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

--- Jump states to build Hybrid State Machines (HSM) for skills.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

require("fawkes.fsm.multi_subfsm_jumpstate")
require("fawkes.hsm")
require("actionlib")
require("skiller.ros.action_jumpstate")

-- Convenience shortcuts
local MultiSubFSMJumpState = fawkes.fsm.multi_subfsm_jumpstate.MultiSubFSMJumpState
local ActionJumpState = skiller.ros.action_jumpstate.ActionJumpState

MultiActionJumpState = {}

function MultiActionJumpState:new(o)
   assert(o.action_clients, "Action clients not specified")

   MultiActionJumpState.setup_subfsms(o)
   o.exit_to = o.exit_to or "FINAL"
   o.fail_to = o.fail_to or "FAILED"

   local o = MultiSubFSMJumpState:new(o)

   setmetatable(o, self)
   setmetatable(self, MultiSubFSMJumpState)
   self.__index = self

   return o
end

function MultiActionJumpState:setup_subfsms()
   self.subfsms = {}
   for _, ac in ipairs(self.action_clients) do
      local fsm = fawkes.hsm.HSM:new{name=self.name .. ":" .. ac.name,
				     start="RUN_ACTION",
				     exit_state="FINAL", fail_state="FAILED"}
      fsm.action_client = ac
      fsm:define_states{"FINAL", "FAILED", {"RUN_ACTION", ActionJumpState, action_client=ac}}
      --fsm:add_transitions{{"RUN_ACTION", "FINAL", "fsm.disabled", precond_only=true}}
      table.insert(self.subfsms, fsm)
   end
end


function MultiActionJumpState:disable_action(action_client)
   for _,s in ipairs(self.subfsms) do
      if s.action_client == action_client then
	 s.disabled = true
      end
   end
end


function MultiActionJumpState:enable_action(action_client)
   for _,s in ipairs(self.subfsms) do
      if s.action_client == action_client then
	 s.disabled = false
      end
   end
end
