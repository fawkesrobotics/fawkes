
------------------------------------------------------------------------
--  depinit.lua - Dependency initializer functions
--
--  Created: Sun Mar 08 12:28:44 2009 (Wesel am Niederrhein)
--  Copyright  2009-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Research Pittsburgh
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

local module_initializers = {}

--- Add a module initializer.
-- Module initializers are called as part of the skill_module() call in skill
-- modules. They are called after basic initializations have been run. They
-- can be used for example to initialize, check, and assert dependencies.
-- @param di dependency initializer, must be a function which takes two
-- arguments. The module m, and a table to which fields should be added (the
-- index metatable). The initializer should not set values directly on the
-- module.
function add_module_initializer(di)
   table.insert(module_initializers, di)
end


--- Initialize module with registered initializers.
-- @param module module table to initialize, considered read-only
-- @param table add anythin you want to export to a module to this table
function init_module(module, table)
   for _, mi in ipairs(module_initializers) do
      mi(module, table)
   end
end
