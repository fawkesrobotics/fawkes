
------------------------------------------------------------------------
--  tableext.lua - table package extensions
--
--  Created: Tue Nov 02 17:06:26 2010 (at Intel Labs Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--             2010  Carnegie Mellon University
--             2010  Intel Labs Pittsburgh
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

local _G = _G;
local table = table;
local type = type
local setmetatable = setmetatable
local getmetatable = getmetatable
local assert = assert
local pairs = pairs

module("fawkes.tableext");

--- Deep copy table.
-- This function creates a deep copied version of the table. This means that
-- a new table is created, and all values are copied. Elements which are a
-- table by themselves are recursively copied, causing real copies to be
-- created instead of just copying references. Optionally, the metatable of
-- the copied table can be assigned to the new table. Note however, that the
-- metatable itself is not deep copied (which would in most cases be harmful).
-- If you want to deepcopy it you can use:
-- newtable = table.deepcopy(t)
-- setmetatable(newtable, table.deepcopy(getmetatable(t)))
-- @param t table to copy
-- @param copy_metatable true to assign the metatable, false not to. Defaults
-- to false.
-- @return new table with a deep copy of t
function table.deepcopy(t, assign_metatable)
   assert(type(t) == "table", "Argument must be of type table")
   local assign_metatable = assign_metatable or false

   local result = {}
   for k,v in pairs(t) do
      if type(v) == "table" then
	 result[k] = table.deepcopy(v)
      else
	 result[k] = v
      end
   end

   if assign_metatable then setmetatable(result, getmetatable(t)) end

   return result
end
