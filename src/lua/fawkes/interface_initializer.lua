
------------------------------------------------------------------------
--  interface_initializer.lua - Fawkes interface dependency initializer
--
--  Created: Sun Mar 08 12:28:44 2009 (Wesel am Niederrhein)
--  Copyright  2009-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
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

---Dependency initializer functions.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)


function init_interfaces(module, table)
   local name = module.name
   local dependencies = module.depends_interfaces
   if not dependencies then return end

   assert(type(dependencies) == "table", "Type of dependencies not table")
   assert(interfaces and type(interfaces) == "table", "Interfaces not initialized")
   for _,t in ipairs(dependencies) do
      assert(type(t) == "table", "Non-table element in interface dependencies")
      assert(t.v, "Interface dependency does not have a variable name (v) field")
      assert(t.type, "Interface dependency does not have a type field")
      if t.id then
	 local uid = t.type .. "::" .. t.id
	 if interfaces.reading_by_uid[uid] then
	    table[t.v] = interfaces.reading_by_uid[uid]
	 elseif interfaces.writing_by_uid[uid] then
	    table[t.v] = interfaces.writing_by_uid[uid]
	 else
	    error("No interface available with the UID " .. uid ..
		  ", required by ".. name)
	 end
      else
	 if interfaces.reading[t.v] then
	    table[t.v] = interfaces.reading[t.v]
	 elseif interfaces.writing[t.v] then
	    table[t.v] = interfaces.writing[t.v]
	 else
	    error("No interface available with the variable name " .. t.v ..
		  ", required by ".. name)
	 end
      end
   end
end
