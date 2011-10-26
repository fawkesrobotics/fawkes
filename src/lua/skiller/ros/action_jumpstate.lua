
------------------------------------------------------------------------
--  action_jumpstate.lua - HSM state to execute ROS actions
--
--  Created: Wed Aug 25 10:28:33 2010
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
require("actionlib")

-- Convenience shortcuts
local SubFSMJumpState = fawkes.fsm.subfsmjumpstate.SubFSMJumpState

ActionJumpState = {}

function ActionJumpState:new(o)
   assert(o.action_client, "Action client not specified")

   ActionJumpState.setup_subfsm(o)
   o.exit_to = o.exit_to or "FINAL"
   o.fail_to = o.fail_to or "FAILED"

   local o = SubFSMJumpState:new(o)

   setmetatable(o, self)
   setmetatable(self, SubFSMJumpState)
   self.__index = self

   return o
end

function ActionJumpState:goal_handle()
   return self.subfsm.vars.goal
end

function ActionJumpState:goal_sent()
   return (self.subfsm.vars.goal ~= nil)
end

function action_trans(from, to, state_to)
   local state_to = state_to or to
   return {from, state_to, "vars.goal and vars.goal.state == vars.goal." .. to, desc="["..to.."]"}
end


--- Set parameters of message.
-- Given a table of input values this function iterates over the messages
-- fields and sets the message value fields appropriately. If the input 
local function set_params(msg, input)
   assert(msg, "ActionJumpState: message is nil")
   for _, f in ipairs(msg.spec.fields) do
      if f.is_array then
	 local a = {}
	 if f.is_builtin then
	    for i, v in ipairs(input[f.name]) do
	       a[i] = v
	    end
	 else
	    for i, v in ipairs(input[f.name]) do
	       a[i] = f.spec:instantiate()
	       set_params(a[i], v)
	    end
	 end
	 msg.values[f.name] = a
      else
	 if f.is_builtin then
	    msg.values[f.name] = input[f.name]
	 else
	    assert(type(input[f.name]) == "table",
		   "Input value for " .. f.name .. " is not a table")
	    set_params(msg.values[f.name], input[f.name])
	 end
      end
   end
end

local function WAIT_GOAL_ACK_init(self)
   local goal = self.fsm.action_client.actspec.goal_spec:instantiate()
   set_params(goal, self.fsm.vars)
   self.fsm.vars.goal = self.fsm.action_client:send_goal(goal)
end

function ActionJumpState:setup_subfsm()
   self.subfsm = fawkes.hsm.HSM:new{name=self.name .. ":" .. self.action_client.name,
				    start="WAIT_SERVER",
				    exit_state="FINAL", fail_state="FAILED"}
   self.subfsm.action_client = self.action_client
   self.subfsm.graph_collapse = true
   if self.collapse ~= nil then self.subfsm.graph_collapse = self.collapse end

   self.subfsm:set_debug(self.fsm.debug)
   self.subfsm:define_states{
      export_to = self,
      "FAILED", "FINAL", "WAIT_SERVER", "WAIT_GOAL_ACK", "PENDING", "ACTIVE", "WAIT_CANCEL_ACK",
      "RECALLING", "PREEMPTING", "WAIT_RESULT"
   }

   self.subfsm:add_transitions{
      {"WAIT_SERVER", "WAIT_GOAL_ACK", "fsm.action_client:has_server()",
       timeout={10, "FAILED", error="no action server"}, desc="Connected"},
      action_trans("WAIT_GOAL_ACK", "PENDING"),
      action_trans("WAIT_GOAL_ACK", "ACTIVE"),
      action_trans("WAIT_GOAL_ACK", "WAIT_CANCEL_ACK"),
      action_trans("WAIT_GOAL_ACK", "REJECTED", "FAILED"),
      action_trans("WAIT_GOAL_ACK", "RECALLING"),
      action_trans("WAIT_GOAL_ACK", "PREEMPTING"),
      action_trans("WAIT_GOAL_ACK", "WAIT_RESULT"),
      action_trans("WAIT_GOAL_ACK", "ABORTED", "FAILED"),
      action_trans("WAIT_GOAL_ACK", "SUCCEEDED", "FINAL"),
      {"WAIT_GOAL_ACK", "FAILED", timeout={10, error="goal not acknowledged"}},
      action_trans("PENDING", "ACTIVE"),
      action_trans("PENDING", "RECALLING"),
      action_trans("PENDING", "PREEMPTING"),
      action_trans("PENDING", "REJECTED", "FAILED"),
      action_trans("PENDING", "WAIT_CANCEL_ACK"),
      action_trans("PENDING", "WAIT_RESULT"),
      action_trans("PENDING", "SUCCEEDED", "FINAL"),
      action_trans("ACTIVE", "WAIT_CANCEL_ACK"),
      action_trans("ACTIVE", "PREEMPTING"),
      action_trans("ACTIVE", "RECALLING"),
      action_trans("ACTIVE", "WAIT_RESULT"),
      action_trans("ACTIVE", "SUCCEEDED", "FINAL"),
      action_trans("ACTIVE", "ABORTED", "FAILED"),
      action_trans("WAIT_CANCEL_ACK", "RECALLING"),
      action_trans("WAIT_CANCEL_ACK", "PREEMPTING"),
      action_trans("WAIT_CANCEL_ACK", "RECALLED", "FAILED"),
      action_trans("WAIT_CANCEL_ACK", "PREEMPTED", "FAILED"),
      action_trans("RECALLING", "PREEMPTING"),
      action_trans("RECALLING", "RECALLED", "FAILED"),
      action_trans("PREEMPTING", "PREEMPTED", "FAILED"),
      action_trans("PREEMPTING", "ABORTED", "FAILED"),
      action_trans("WAIT_RESULT", "SUCCEEDED", "FINAL"),
      {"WAIT_RESULT", "FAILED", timeout=10}
   }
   self.WAIT_GOAL_ACK.init = WAIT_GOAL_ACK_init
end

function ActionJumpState:do_exit()
   if self.subfsm.current.name == self.subfsm.fail_state and
      self.subfsm.vars.goal
   then
      self.subfsm.error = self.subfsm.vars.goal.status_text or ""
      print_warn("ActionJumpState[%s %s] error: %s",
		 self.name, self.action_client.name, self.subfsm.error)
   end

   SubFSMJumpState.do_exit(self)

   if self.subfsm.current.name ~= self.subfsm.exit_state
      and self.subfsm.current.name ~= self.subfsm.fail_state
      and self.fsm.vars.goal
   then
      self.fsm.vars.goal:cancel()
   end
end
