
----------------------------------------------------------------------------
--  graph_services.lua - ROS services to manipulate graph
--
--  Created: Fri Aug 27 09:42:56 2010 (at Intel Research, Pittsburgh)
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

module("skiller.ros.graph", package.seeall)

require("roslua")
require("fawkes.fsm.grapher")

local pub_graph
local srv_color
local srv_direction
local msgspec_graph

function init()
   pub_graph = roslua.publisher("~graph", "skiller/Graph")
   pub_graph.latching = true

   srv_direction = roslua.service("~graph/set_direction",
				  "skiller/SetGraphDirection",
				  set_direction)
   srv_color = roslua.service("~graph/set_colored",
			      "skiller/SetGraphColored",
			      set_colored)
   msgspec_graph = roslua.get_msgspec("skiller/Graph")
end


function set_colored(colored)
   local use_color = false
   if colored == 1 then use_color = true end
   fawkes.fsm.grapher.set_colored(use_color)
   printf("Setting graph colored to %s", tostring(use_color))
end

function set_direction(direction)
   if direction == msgspec_graph.constants.GRAPH_DIR_LEFT_RIGHT.value then
      fawkes.fsm.grapher.set_rankdir("LR")
   elseif direction == msgspec_graph.constants.GRAPH_DIR_RIGHT_LEFT.value then
      fawkes.fsm.grapher.set_rankdir("RL")
   elseif direction == msgspec_graph.constants.GRAPH_DIR_TOP_BOTTOM.value then
      fawkes.fsm.grapher.set_rankdir("TB")
   elseif direction == msgspec_graph.constants.GRAPH_DIR_BOTTOM_TOP.value then
      fawkes.fsm.grapher.set_rankdir("BT")
   end
   printf("Setting graph rankdir to %s", fawkes.fsm.grapher.get_rankdir())
end

local function rankdir_to_graphdir()
   local rankdir = fawkes.fsm.grapher.get_rankdir()
   if rankdir == "TB" then
      return msgspec_graph.constants.GRAPH_DIR_TOP_BOTTOM.value
   elseif rankdir == "BT" then
      return msgspec_graph.constants.GRAPH_DIR_BOTTOM_TOP.value
   elseif rankdir == "RL" then
      return msgspec_graph.constants.GRAPH_DIR_RIGHT_LEFT.value
   else
      return msgspec_graph.constants.GRAPH_DIR_LEFT_RIGHT.value
   end
end

function publish(fsm)
   local do_publish = (fsm == nil)

   local m = roslua.get_msgspec("skiller/Graph"):instantiate()
   m.values.stamp = roslua.Time.now()

   if fsm then
      if (fsm:changed() or fawkes.fsm.grapher.get_params_changed()) then
	 if fsm.dotattr then
	    fsm.dotattr.fontname = "Neo Sans Intel"
	 else
	    fsm.dotattr = { fontname = "Neo Sans Intel"}
	 end
	 local graph = fsm:graph()
	 m.values.name      = active_skill
	 m.values.dotgraph  = graph
	 m.values.colored   = fawkes.fsm.grapher.get_colored()
	 m.values.direction = rankdir_to_graphdir()
	 do_publish = true
      end
   else
      m.values.name      = ""
      m.values.dotgraph  = ""
      m.values.colored   = fawkes.fsm.grapher.get_colored()
      m.values.direction = rankdir_to_graphdir()
   end

   if do_publish then pub_graph:publish(m) end
end
