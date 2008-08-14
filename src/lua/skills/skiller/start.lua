
----------------------------------------------------------------------------
--  start.lua - skiller Lua start code
--              executed when exec thread is running, but before skills are
--              executed. Only run if initialization was successful.
--
--  Created: Thu Mar 13 11:24:40 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
--  $Id$
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

-- Can be used to debug component path
--[[
print(package.path);
print(package.cpath);
--]]

--[[ lists whole config
v = config:iterator()
while ( v:next() ) do
   if ( v:is_float() ) then
      print(v:path(), "[float]", v:get_float())
   elseif ( v:is_uint() ) then
      print(v:path(), "[uint]", v:get_uint())
   elseif ( v:is_int() ) then
      print(v:path(), "[int]", v:get_int())
   elseif ( v:is_bool() ) then
      print(v:path(), "[bool]", v:get_bool())
   elseif ( v:is_string() ) then
      print(v:path(), "[string]", v:get_string())
   end
end
--]]

require("fawkes.logprint")
require("skills.skiller.skillenv")

fawkes.logprint.init(logger)
skills.skiller.skillenv.init(SKILLSPACE)

-- Interfaces are not yet available at this point!
require("skills." .. SKILLSPACE)

logger:log_debug("Lua startup completed")
