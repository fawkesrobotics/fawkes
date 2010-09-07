
----------------------------------------------------------------------------
--  init.lua - Skiller ROS dependent bits
--
--  Created: Tue Aug 24 13:29:28 2010 (at Intel Research, Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
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

module("skiller.ros", package.seeall)

require("roslua")
require("actionlib")
require("skiller.ros.graph")
local skillenv

function init()
   roslua.init_node{master_uri=ROS_MASTER_URI, node_name="/skiller"}
   skiller_as = actionlib.action_server("/skiller/exec", "skiller/ExecSkill",
					goal_cb, spin_cb, cancel_cb)
   skiller.ros.graph.init()
   skillenv = require("skiller.skillenv")
end


function goal_cb(goal_handle, action_server)
   printf("Starting goal %s", goal_handle.goal_id)
   action_server:cancel_goals_before(goal_handle.goalmsg.values.header.values.stamp)
   goal_handle.vars.skillstring = goal_handle.goalmsg.values.goal.values.skillstring
   local ok, sksf = pcall(loadstring, goal_handle.vars.skillstring)
   if ok then
      skillenv.reset_all()
      local sandbox = skillenv.gensandbox()
      setfenv(sksf, sandbox)
      goal_handle.vars.sksf = sksf
      printf("Accepting goal %s", goal_handle.goal_id)
      goal_handle:accept()
   else
      printf("Rejecting goal %s", goal_handle.goal_id)
      goal_handle:reject(sksf)
   end
end


function spin_cb(goal_handle, action_server)
   skillenv.reset_status()
   local ok, errmsg = pcall(goal_handle.vars.sksf)
   if not ok then
      print_error("Execution of %s failed: %s", goal_handle.vars.skillstring, errmsg)
      local result = action_server.actspec.result_spec:instantiate()
      result.values.errmsg = errmsg
      goal_handle:abort(result, errmsg)
   else
      local running, final, failed = skillenv.get_status()

      if failed > 0 then
	 local result = action_server.actspec.result_spec:instantiate()
	 result.values.errmsg = skillenv.get_error()
	 print_warn("Skill execution of '%s' failed (%s)",
		    goal_handle.vars.skillstring, result.values.errmsg)
	 goal_handle:abort(result, result.values.errmsg)
      elseif final > 0 and running == 0 then
	 print_info("Skill execution of '%s' succeeded", goal_handle.vars.skillstring)
	 local result = action_server.actspec.result_spec:instantiate()
	 goal_handle:finish(result)
      elseif running > 0 then
	 -- nothing to do
      end
   end

   local active_skill = skiller.skillenv.get_active_skills()
   local fsm
   if active_skill then
      fsm = skiller.skillenv.get_skill_fsm(active_skill)
   end
   skiller.ros.graph.publish(fsm)
end


function cancel_cb(goal_handle, action_server)
   print_warn("Goal %s (%s) cancelled", goal_handle.goal_id, goal_handle.vars.skillstring)
   skillenv.reset_all()
   skiller.ros.graph.publish()
end
