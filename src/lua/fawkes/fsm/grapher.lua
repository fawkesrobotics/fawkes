
------------------------------------------------------------------------
--  grapher.lua - FSM Grapher (via graphviz)
--
--  Created: Tue Dec 23 00:02:34 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
--  $Id$
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

--- This module provides a state to create a Hybrid State Machine (HSM)
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)
local fsmmod = require("fawkes.fsm")
assert(fsmmod, "fsmmod is nil")

local gv_load_success, gv_load_error = pcall(require, "gv")
local gv
if gv_load_success then
   gv = _G.gv
else
   print_warn("FSM graphing disabled, Graphviz could not be loaded: %s",
	      gv_load_error)
end

function write(fsm, filename)
   assert(fsm, "Grapher requires valid FSM")

   if not gv_load_success then return end

   local g = gv.digraph(fsm.name)

   --gv.setv(g, "rankdir", "LR")

   local pn = gv.protonode(g);
   local pe = gv.protoedge(g);

   gv.setv(pn, "penwidth", "4.0")
   --gv.setv(pn, "mindist", "0.0")
   gv.setv(pe, "penwidth", "2.0")
   --gv.setv(pe, "constraint", "false")

   for name, state in pairs(fsm.states) do
      --print("*** Adding state " .. name)
      local n = gv.node(g, name)

      if state.dotattr then
	 for k,v in pairs(state.dotattr) do
	    gv.setv(n, k, v)
	 end
      end
      -- print("*** Checking exit state " .. fs.name)
      if name == fsm.exit_state or name == fsm.fail_state then
	 gv.setv(n, "shape", "doublecircle")
	 gv.setv(n, "penwidth", "1.0")
      end
      if fsm.current and name == fsm.current.name then
	 gv.setv(n, "color", "red")
      elseif not fsm.current and name == fsm.start then
	 gv.setv(n, "style", "bold")
      elseif fsm.tracing and fsm:traced_state(state) then
	 gv.setv(n, "color", "blue")
      end
   end

   for name, state in pairs(fsm.states) do
      if (state.transitions) then
	 for _, tr in ipairs(state.transitions) do
	    --print("*** Adding transition " .. name .. " -> " .. tr.state.name .. "(" .. tr.description .. ")")
	    local e = gv.edge(g, name, tr.state.name)
	    if tr.dotattr then
	       for k,v in pairs(tr.dotattr) do
		  gv.setv(e, k, v)
	       end
	    end
	    if tr.description then
	       gv.setv(e, "label", tostring(tr.description))
	    end
	    if fsm.fail_state and tr.state.name == fsm.fail_state then
	       gv.setv(e, "style", "dotted")
	    end
	    if fsm.exit_state and name == fsm.exit_state or
	       fsm.fail_state and name == fsm.fail_state then
	       gv.setv(e, "color", "red3")
	       gv.setv(e, "style", "dashed")
	    end
	    if fsm.tracing then
	       local traced, traces = fsm:traced_trans(tr)
	       if traced then
		  local sorted_indexes = {}
		  for i,_ in pairs(traces) do
		     table.insert(sorted_indexes, i)
		  end
		  table.sort(sorted_indexes)
		  if next(sorted_indexes) then
		     local s = tostring(sorted_indexes[1])
		     for i=2,#sorted_indexes do
			s = s .. "," .. tostring(sorted_indexes[i])
		     end
		     gv.setv(e, "taillabel", s)
		     gv.setv(e, "labelfontcolor", "blue")
		  end
		  gv.setv(e, "color", "blue")
	       end
	    end
	 end
      end
   end

   --print("+++ Writing to file " .. filename)
   gv.write(g, filename)
end


function dotgraph(fsm)
   if not gv_load_success then return "" end
   local tfname = os.tmpname()
   write(fsm, tfname)
   local rv = ""
   local tf = io.open(tfname)
   for line in tf:lines() do
      rv = rv .. line .. "\n"
   end
   tf:close()
   os.remove(tfname)

   return rv
end
