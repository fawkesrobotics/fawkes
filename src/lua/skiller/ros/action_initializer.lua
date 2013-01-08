
------------------------------------------------------------------------
--  action_initializer.lua - Action dependency initializer
--
--  Created: Wed Aug 25 18:11:59 2010 (Intel Research Pittsburgh)
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

--- Action dependency initializer function.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

require("actionlib")

local action_clients = {}

function init_actions(module, table)
   local deps = module.depends_actions
   if not deps then return end

   assert(type(deps) == "table", "Type of dependencies not table")

   for _,t in ipairs(deps) do
      assert(type(t) == "table", "Non-table element in action dependencies")
      assert(t.v, "Action dependency does not have a variable name (v) field")
      assert(t.name, "Action dependency does not have a name field")
      assert(t.type, "Action dependency does not have a type field")

      local id = t.name .. "::" .. t.type
      if not action_clients[id] then
	 local flags
	 if t.reduced then
	    flags = {no_feedback=true, no_cancel=true}
	    print_debug("ActionClient %s::%s ignores feedback and cancelling", t.name, t.type)
	 end
	 action_clients[id] = actionlib.action_client(t.name, t.type, flags)
      end
      table[t.v] = action_clients[id]
   end
end
