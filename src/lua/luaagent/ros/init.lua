
----------------------------------------------------------------------------
--  init.lua - Lua Agent ROS dependent bits
--
--  Created: Thu Sep  2 15:09:49 2010 (at Intel Research, Pittsburgh)
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

module("luaagent.ros", package.seeall)

require("roslua")
require("actionlib")
require("skiller.ros.graph")
require("luaagent.agentenv")


function init()
   roslua.init_node{master_uri=ROS_MASTER_URI, node_name="/luaagent"}
   skiller.ros.graph.init()
   luaagent.agentenv.write_graph = skiller.ros.graph.publish
   roslua.add_spinner(luaagent.agentenv.execute)
end
