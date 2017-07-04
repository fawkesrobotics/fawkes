
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

local interfaces_reading = {}
local interfaces_writing = {}

local interfaces_writing_stash = {}
local interfaces_writing_preloaded = {}

local blackboard = _G.blackboard

function finalize_prepare()
	 interfaces_writing_stash = interfaces_writing
	 interfaces_writing = {}
	 return interfaces_writing_stash
end

function finalize_cancel()
	 interfaces_writing = interfaces_writing_stash
	 interfaces_writing_stash = {}
end

function finalize()
	 for _,v in pairs(interfaces_reading) do
			blackboard:close(v)
	 end
	 interfaces_reading = {}
	 for _,v in pairs(interfaces_writing) do
			blackboard:close(v)
	 end
	 interfaces_writing = {}
	 for _,v in pairs(interfaces_writing_stash) do
			tolua.releaseownership(v)
	 end
	 interfaces_writing_stash = {}
end

function read()
	 for _,v in pairs(interfaces_reading) do
			v:read()
	 end
end

function write()
	 for k,v in pairs(interfaces_writing) do
			if v:changed() then
				 v:write()
			end
	 end	 
end

function preload(interfaces_writing_preload)
	 for k,v in pairs(interfaces_writing_preload) do
			print_debug("Preloading writing interface " .. tostring(k) .. "::" .. tostring(v))
			interfaces_writing[k] = v
			interfaces_writing_preloaded[k] = v
	 end
end

--- Renive preloaded interfaces
-- This can be used to prevent closing preloaded interfaces.
function preloaded_remove_without_closing()
	 for k,_ in pairs(interfaces_writing_preloaded) do
			interfaces_writing[k] = nil
	 end
end

function init_interfaces(module, table)
   local name = module.name
   local dependencies = module.depends_interfaces
   if not dependencies then return end

   assert(type(dependencies) == "table", "Type of dependencies not table")
   if #dependencies == 0 then
      for k,v in pairs(dependencies) do
         assert(type(v) == "table", "depends_interfaces must be a table of tables")
      end
   end
   for _,t in ipairs(dependencies) do
      assert(type(t) == "table", "Non-table element in interface dependencies")
      assert(t.v, "Interface dependency of " .. name .. " does not have a variable name (v) field")
      assert(t.type, "Interface dependency '" .. t.v .. "' of " .. name .. " does not have a type field")
      assert(t.id, "Interface dependency '" .. t.v .. "' of " .. name .. " does not have an id field")

			local uid = t.type .. "::" .. t.id
			
			if t.writing then
				 if not interfaces_writing[uid] then
						print_debug("Opening interface %s for writing", uid)
						interfaces_writing[uid] = blackboard:open_for_writing(t.type, t.id)
				 end
				 assert(interfaces_writing[uid], "Failed to open interface " .. uid .. " for writing")

				 table[t.v] = interfaces_writing[uid]
			else
				 if not interfaces_reading[uid] then
						print_debug("Opening interface %s for reading", uid)
						interfaces_reading[uid] = blackboard:open_for_reading(t.type, t.id)
				 end
				 assert(interfaces_reading[uid], "Failed to open interface " .. uid .. " for reading")

				 table[t.v] = interfaces_reading[uid]
			end
	 end
end
