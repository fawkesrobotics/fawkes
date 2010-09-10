
------------------------------------------------------------------------
--  topic_initializer.lua - Topic dependency initializer
--
--  Created: Wed Sep 08 18:32:47 2010 (Intel Research Pittsburgh)
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

function init_topics(module, table)
   local deps = module.depends_topics
   if not deps then return end

   assert(type(deps) == "table", "Type of topic dependencies not table")

   for _,t in ipairs(deps) do
      assert(type(t) == "table", "Non-table element in topic dependencies")
      assert(t.v, "Topic dependency does not have a variable name (v) field")
      assert(t.name, "Topic dependency does not have a name field")
      assert(t.type, "Topic dependency does not have a type field")

      -- we do not cache those, roslua does already
      if t.publisher then
	 table[t.v] = roslua.publisher(t.name, t.type)
      else
	 printf("Registering subscriber %s::%s for %s", t.name, t.type, t.v)
	 table[t.v] = roslua.subscriber(t.name, t.type)
      end
      table[t.v].latching = t.latching
   end
end
