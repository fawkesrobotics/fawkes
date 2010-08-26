
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

local pub_graph
local sksf

function init()
   roslua.init_node{master_uri=ROS_MASTER_URI, node_name="/skiller"}
   skiller_as = actionlib.action_server("skiller", "skiller/ExecSkill")
   roslua.add_spinner(spin)
   pub_graph = roslua.publisher("/skiller/graph", "skiller/Graph")
end


function start(skillstring)
   sksf = loadstring(skillstring)
   local sandbox = skillenv.gensandbox()
   setfenv(sksf, sandbox)
end

function spin()
   skillenv.reset_status()
   if sksf then
      sksf()
      local active_skill = skillenv.get_active_skills()
      if active_skill then
	 local fsm = skillenv.get_skill_fsm(active_skill)
	 if fsm and fsm:changed() then
	    local graph = fsm:graph()
	    local m = roslua.get_msgspec("skiller/Graph"):instantiate()
	    m.values.name = active_skill
	    m.values.dotgraph = graph
	    pub_graph:publish(m)
	 end
      end
   end
end
