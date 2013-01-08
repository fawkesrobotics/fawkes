
------------------------------------------------------------------------
--  predlib.lua - Predicate Library Module
--
--  Created: Sat Mar 07 13:28:41 2009 (Millingen am Niederrhein)
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

--- Predicate library base module.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)
local depinit = require("fawkes.depinit")

local predlibs = {}

local function create_newindex_func(predicates)
   return function (module, key, value)

	     local valtype = type(value)

	     if key:match("depends_.*") then
		rawset(module, key, value)
	     elseif key == "predparams" or key == "setup"
		    or valtype == "table" or valtype == "table"
		    or valtype == "string" or valtype == "number"
		    or valtype == "boolean" then

		    -- constants, params or setup function
		   rawset(module, key, value)
	     else
		predname = key
		predfunc = value

		assert(type(predname) == "string" and #predname > 0, "Invalid predicate name, not a string")
		assert(type(predfunc) == "function", "Invalid predicate function for " .. predname)
		assert(not predicates[predname], "Predicate " .. predname .. " has already been defined")


		-- print("Adding t[(" .. type(predname) .. ") " .. tostring(predname) .. "] = (" .. type(predfunc) .. ") " .. tostring(predfunc) .. " from " .. module._NAME)
		predicates[predname] = predfunc
	     end
	  end
end


local function create_index_func(predicates, metatable)
   return function (module, predname)
	     if metatable.dependencies[predname] or predname:match("depends_.*") then
		return metatable.dependencies[predname]
	     end
	     assert(predicates[predname],
		    "Predicate " .. tostring(predname) .. " is not available in module " .. module._NAME)

	     local p, store_value = predicates[predname]()
	     if store_value ~= nil then
		rawset(module, predname, storevalue)
	     else
		rawset(module, predname, p)
	     end
	     return p
	  end
end

local function create_setup_func(module)
   return function (args)
	     if not module.predparams then
		module.predparams = {}
	     end
	     for k,v in pairs(args) do
		module.predparams[k] = v
	     end
	  end
end

function get_predfunc(predlib, predname)
   local m = getmetatable(predlib)
   assert(m, "No meta table found, not a predicate library?")
   assert(m.predicates and type(m.predicates) == "table",
	  "Predicates table not found or not a table, not a predicate library?")

   return m.predicates[predname]
end

function reset()
   for name, pl in pairs(predlibs) do
      -- print("Working on module " .. name)
      local m = pl.module
      for pn,_ in pairs(pl.predicates) do
	 -- print("Resetting predicate " .. pn .. " in module " .. name)
	 rawset(m, pn, nil)
      end
      for n, i in pairs(pl.interfaces) do
	 rawset(m, n, i)
      end
   end
end

function module_init(module)
   assert(module._NAME, "predlib.module_init() can only be called for modules")
   assert(predlibs[module._NAME] == nil, "predlib " .. module._NAME .. " has already been registered")

   --fawkes.modinit.module_init(module)

   local predicates = {}
   local metatable  = {}

   metatable.predicates   = predicates
   metatable.dependencies = {}
   metatable.__index      = create_index_func(predicates, metatable)
   metatable.__newindex   = create_newindex_func(predicates, metatable),

   setmetatable(module, metatable)

   if not rawget(module, "setup") then
      module.setup = create_setup_func(module)
   end

   predlibs[module._NAME] = {module=module, predicates=predicates, interfaces = {}}
end

-- Initialize a predicate library.
-- @param m table or name of the module to initialize
function setup(module_name)
   local m = module_name
   if type(module_name) == "string" then m = require(module_name) end

   assert(m.name and type(m.name) == "string", "Skill name not set or not a string")
   local mt = getmetatable(m)
   assert(mt, "Predicate library not properly initialized using predlib,module_init")

   depinit.init_module(m, mt.dependencies)
end
