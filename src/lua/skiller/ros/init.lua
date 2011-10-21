
----------------------------------------------------------------------------
--  init.lua - Skiller ROS dependent bits
--
--  Created: Tue Aug 24 13:29:28 2010 (at Intel Labs Pittsburgh)
--  Copyright  2010-2011  Tim Niemueller [www.niemueller.de]
--             2011       SRI International
--             2010-2011  Carnegie Mellon University
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

module("skiller.ros", package.seeall)

require("roslua")
require("actionlib")
require("skiller.ros.graph")
require("fawkes.logprint")
local skillenv
local nodemon

-- You can set this from the outside, it must be a function which takes
-- the same arguments as printf (from fawkes.logprint), i.e. a format string
-- and a suitable number of additional arguments for the format entities, as
-- for string.format().
print_fail  = fawkes.logprint.print_warn
print_final = fawkes.logprint.print_info

function init()
   roslua.init_node{node_name="skiller"}

   local ok, nodemonmod = pcall(require, "nodemon")
   if ok then
      nodemon = nodemonmod.NodeStatePublisher:new("skiller", "skiller")
   else
      print_warn("Node monitoring disabled (module nodemon not found):\n%s",
		 nodemonmod)
   end

   skiller_as = actionlib.action_server("~exec", "skiller/ExecSkill",
					goal_cb, spin_cb, cancel_cb)
   skiller.ros.graph.init()
   skillenv = require("skiller.skillenv")

   if nodemon then nodemon:set_running() end
end


function goal_cb(goal_handle, action_server)
   local stamp = goal_handle.goalmsg.values.header.values.stamp
   action_server:cancel_goals_before(stamp)
   goal_handle.vars.skillstring =
      goal_handle.goalmsg.values.goal.values.skillstring
   printf("Starting goal %s (%s)", goal_handle.goal_id,
          goal_handle.vars.skillstring)
   local sksf, err = loadstring(goal_handle.vars.skillstring)
   if sksf then
      skillenv.reset_all()
      local sandbox = skillenv.gensandbox()
      setfenv(sksf, sandbox)
      goal_handle.vars.sksf = sksf
      printf("Accepting goal %s", goal_handle.goal_id)
      goal_handle:accept(goal_handle.vars.skillstring)
      if nodemon then nodemon:set_running() end
   else
      local errstr = string.format("%s|%s", goal_handle.vars.skillstring, err)
      print_error("lua_error_skillstring: " .. errstr)
      if nodemon then
	 nodemon:set_error("lua_error_skillstring", errstr)
	 nodemon:set_recovering("autorecover", "ready for new skill calls")
	 nodemon:set_running()
      end
      goal_handle:reject(errstr)
   end
end


function spin_cb(goal_handle, action_server)
   skillenv.reset_status()
   local ok, errmsg = xpcall(goal_handle.vars.sksf, debug.traceback)
   if not ok then
      local errstr = string.format("%s|%s", goal_handle.vars.skillstring, errmsg)
      print_error("lua_error_skill: " .. errstr)

      local result = action_server.actspec.result_spec:instantiate()
      result.values.errmsg = errstr
      if nodemon then
	 nodemon:set_error("lua_error_skill", errstr)
	 nodemon:set_recovering("autorecover: ready for new skill calls")
	 nodemon:set_running()
      end
      goal_handle:abort(result, errstr)
   else
      local running, final, failed = skillenv.get_status()

      if failed > 0 then
	 local result = action_server.actspec.result_spec:instantiate()
	 local errstr_machine, errstr_human = skillenv.get_error()

	 result.values.errmsg = errstr_machine
	 print_fail("%s", errstr_human)
	 if nodemon then
	    nodemon:set_error("lua_error_skill", errstr_machine)
	    nodemon:set_recovering("autorecover", "ready for new skill calls")
	    nodemon:set_running()
	 end
	 goal_handle:abort(result, errstr)
      elseif final > 0 and running == 0 then
	 print_final("exec_final: %s", goal_handle.vars.skillstring)
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
   print_warn("Goal %s (%s) cancelled",
	      goal_handle.goal_id, goal_handle.vars.skillstring)
   skillenv.reset_all()
   skiller.ros.graph.publish()
   if nodemon then nodemon:set_running() end
end

