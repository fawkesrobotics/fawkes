
----------------------------------------------------------------------------
--  start.lua - skiller Lua start code
--              executed when exec thread is running, but before skills are
--              executed. Only run if initialization was successful.
--
--  Created: Thu Mar 13 11:24:40 2008
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

-- lists whole config
--[[
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
-- prints all interfaces
--[[
for k,v in pairs(interfaces) do
   for k2,v2 in pairs(v) do
      print(k, k2)
   end
end
--]]

require("fawkes.logprint")
fawkes.logprint.init(logger)

require("fawkes.depinit")
require("fawkes.mathext")
local ifinitmod = require("fawkes.interface_initializer")

skillenv = require("skiller.skillenv")

if interfaces_writing_preload then
	 ifinitmod.preload(interfaces_writing_preload)
	 interfaces_writing_preload = nil
end

fawkes.depinit.add_module_initializer(ifinitmod.init_interfaces)
skillenv.add_finalize_callback("interface_initializer", ifinitmod.finalize)
skillenv.add_preloop_callback("fawkes_interfaces_read", ifinitmod.read)
skillenv.add_loop_callback("fawkes_interfaces_write", ifinitmod.write)

if config:exists("/skiller/features/ros/enable")
   and config:get_bool("/skiller/features/ros/enable")
then
   logger:log_debug("ROS feature enabled, checking for roslua availability")
   local ros_available = require("skiller.ros.available")
   if ros_available() then
      logger:log_debug("Starting internal ROS node (roslua)")
      local uri = os.getenv("ROS_MASTER_URI")
      if uri then
				 ROS_MASTER_URI = uri
      else
				 error("ROS_MASTER_URI environment variable not defined")
      end

      dofile(LUADIR .. "/skiller/ros/start.lua")

      _G.HAVE_ROS = true

      logger:log_debug("ROS startup complete")
   else
      logger:log_error("ROS feature requested but roslua not available")
      error("ROS feature requested but roslua not available")
   end
end

require("skiller.fawkes")
skiller.fawkes.init()

local ok, errmsg = xpcall(function() skillenv.init(SKILLSPACE, skiller.fawkes.loop) end, debug.traceback)
if not ok then error(errmsg) end

logger:log_debug("Lua startup completed")
