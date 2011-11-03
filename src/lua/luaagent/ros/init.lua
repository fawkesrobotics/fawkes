
----------------------------------------------------------------------------
--  init.lua - Lua Agent ROS dependent bits
--
--  Created: Thu Sep  2 15:09:49 2010 (at Intel Research, Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--             2010  Carnegie Mellon University
--             2010  Intel Labs Pittsburgh
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

module("luaagent.ros", package.seeall)

require("roslua")
require("actionlib")
require("skiller.ros.graph")
require("skiller.skillstati")
require("luaagent.skillqueue")

local actc_skiller
--local pub_status
local goal_handle
nodemon=nil

function init()
   roslua.init_node{node_name="luaagent"}

   local ok, nodemonmod = pcall(require, "nodemon")
   if ok then
      nodemon = nodemonmod.NodeStatePublisher:new("luaagent", "luaagent")
   else
      print_warn("Node monitoring disabled (module nodemon not found):\n%s",
                 nodemonmod)
   end

   skiller.ros.graph.init()
   require("luaagent.agentenv")
   luaagent.agentenv.write_graph = skiller.ros.graph.publish
   luaagent.agentenv.handle_error = agentenv_handle_error


   --pub_status = roslua.publisher("/luaagent/status", "skiller/SkillStatus")
   actc_skiller = actionlib.action_client("skiller/exec", "skiller/ExecSkill")
   printf("Waiting for Skiller")
   luaagent.skillqueue.SkillQueue.execute = SkillQueue_execute_ros
   luaagent.skillqueue.SkillQueue.status  = SkillQueue_status_ros
   luaagent.skillqueue.SkillQueue.stop    = SkillQueue_stop_ros
   luaagent.skillqueue.SkillQueue.error   = SkillQueue_error_ros

   roslua.add_spinner(deferred_init)
end


function deferred_init()
   if actc_skiller:has_server() then
      roslua.remove_spinner(deferred_init)
      printf("Skiller connected")

      local ok, err = pcall(agentenv.init, AGENT)

      if not ok then
         if luaagent.ros.nodemon then
            luaagent.ros.nodemon:set_fatal("init_fail",
                                           "Agent initialization failed: " .. err)
         end
         print_error("Initialization of agent '%s' failed: %s", AGENT, err)
         roslua.exit()
      else
         roslua.add_spinner(luaagent.agentenv.execute)
         if nodemon then nodemon:set_running() end
         print_warn("Agent '%s' initialization complete.", AGENT)
      end
   end
   if nodemon then nodemon:ping() end
end

function publish_status(fsm)
   local m = pub_status.msgspec:instantiate()
   m.values.errmsg = fsm.error
   m.values.status = pub_status.msgspec.constants.S_RUNNING.value
   pub_status:publish(m)
end

--- Error handling function.
-- This function can be overridden. It is called if the agent's execute fails.
-- The default implementation just raises an error.
-- @param err error message
function agentenv_handle_error(err)
   if luaagent.ros.nodemon then
      luaagent.ros.nodemon:set_fatal("exec_fail",
                                     "Agent execution failed: " .. err)
   end
   print_error("Agent execution failed: %s", err)
   error(err)
end


function SkillQueue_execute_ros(self)
   self.skillstring = self:skill_string()

   if goal_handle then goal_handle:cancel() end

   printf("Sending goal '%s'", self.skillstring)

   local goal = actc_skiller.actspec.goal_spec:instantiate()
   goal.values.skillstring = self.skillstring
   goal_handle = actc_skiller:send_goal(goal)
end


function SkillQueue_status_ros(self)
   if self.skillstring == "" then
      return skiller.skillstati.S_INACTIVE
   else
      if goal_handle:canceled() or goal_handle:failed() then
         --[[
         -- We only care about errors, e.g. not simply that an action failed,
         -- but the skill string was wrong or similar, this is already reported
         -- by the skiller, hence do not report again
         if goal_handle and goal_handle:failed() and nodemon then
            local s = string.format("Skill execution of %s failed: %s",
                                    self.skillstring,
                                    goal_handle.result.values.errmsg or "?")
            nodemon:set_error("skill_exec_failed", s)
            nodemon:set_recovering("autorecover", "Can continue next time")
         end
         --]]
	 return skiller.skillstati.S_FAILED
      elseif goal_handle:succeeded() then
	 return skiller.skillstati.S_FINAL
      else
	 return skiller.skillstati.S_RUNNING
      end
   end
end


function SkillQueue_stop_ros(self)
   if goal_handle then
      goal_handle:cancel()
   end
   goal_handle = nil
end


function SkillQueue_error_ros(self)
   if goal_handle and goal_handle.result then
      return goal_handle.result.values.errmsg or ""
   else
      return ""
   end
end
