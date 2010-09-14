
------------------------------------------------------------------------
--  service_initializer.lua - Service dependency initializer
--
--  Created: Tue Sep 14 17:39:37 2010 (Intel Research Pittsburgh)
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

function init_services(module, table)
   local deps = module.depends_services
   if not deps then return end

   assert(type(deps) == "table", "Type of service dependencies not table")

   for _,t in ipairs(deps) do
      assert(type(t) == "table", "Non-table element in service dependencies")
      assert(t.v, "Service dependency does not have a variable name (v) field")
      assert(t.name, "Service dependency does not have a name field")
      assert(t.type, "Service dependency does not have a type field")

      -- we do not cache those, roslua does already
      printf("Registering service client %s::%s as %s", t.name, t.type, t.v)
      table[t.v] = roslua.service_client(t.name, t.type, t.persistent or false)
   end
end
