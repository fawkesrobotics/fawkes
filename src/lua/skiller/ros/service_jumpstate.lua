
------------------------------------------------------------------------
--  service_jumpstate.lua - HSM state to execute ROS services
--
--  Created: Tue Sep 14 16:42:45 2010
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

require("fawkes.fsm.subfsmjumpstate")
require("fawkes.hsm")
require("roslua")

-- Convenience shortcuts
local SubFSMJumpState = fawkes.fsm.subfsmjumpstate.SubFSMJumpState

ServiceJumpState = {}

function ServiceJumpState:new(o)
   assert(o.service_client, "Service client not specified")

   ServiceJumpState.setup_subfsm(o)
   o.exit_to = o.exit_to or "FINAL"
   o.fail_to = o.fail_to or "FAILED"

   local o = SubFSMJumpState:new(o)

   setmetatable(o, self)
   setmetatable(self, SubFSMJumpState)
   self.__index = self

   return o
end

--- Set parameters of message.
-- Given a table of input values this function iterates over the messages
-- fields and sets the message value fields appropriately. If the input 
local function set_params(msg, input)
   for _, f in ipairs(msg.spec.fields) do
      if f.is_builtin then
	 msg.values[f.name] = input[f.name]
      else
	 assert(type(input[f.name]) == "table", "Input value for " .. f.name .. " is not a table")
	 set_params(msg.values[f.name], input[f.name])
      end
   end
end

local function CALL_init(self)
   self.fsm.service_client:concexec_start(self.fsm.params or self.fsm.vars)
end

function ServiceJumpState:setup_subfsm()
   self.subfsm = fawkes.hsm.HSM:new{name=self.name .. ":" .. self.service_client.service,
				    start="CALL",
				    exit_state="FINAL", fail_state="FAILED"}
   self.subfsm.service_client = self.service_client
   self.subfsm.graph_collapse = true

   self.subfsm:set_debug(self.fsm.debug)
   self.subfsm:define_states{ export_to = self, closure={service_client=self.service_client},
      "FAILED", "FINAL", "CALL" }

   self.subfsm:add_transitions{
      {"CALL", "FINAL", "service_client:concexec_succeeded()"},
      {"CALL", "FAILED", "service_client:concexec_failed()"}
   }
   self.CALL.init = CALL_init
end

function ServiceJumpState:do_init()
   -- Note that this also already calls init() and checks the regular
   -- non-precondition transitions!
   local rv = { SubFSMJumpState.do_init(self) }
   if next(rv) then return unpack(rv) end

   if self.params then
      self.subfsm.params = self.params
   end
end

function ServiceJumpState:do_exit()
   -- It's not running if a precondition already causes a state change
   if self.service_client:concexec_running() then
      if self.service_client:concexec_succeeded() then
         self.fsm.vars.result = self.service_client:concexec_result()
      else
         self.service_client:concexec_abort()
      end
      if self.subfsm.current.name == self.subfsm.fail_state then
         self.subfsm.error = self.service_client.concexec_error or ""
         print_warn("ServiceJumpState[%s %s] error: %s",
                    self.name, self.service_client.service, self.subfsm.error)
      end

   end

   SubFSMJumpState.do_exit(self)
end
