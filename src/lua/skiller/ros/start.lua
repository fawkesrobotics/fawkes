
----------------------------------------------------------------------------
--  start.lua - Skiller startup script -- ROS version
--
--  Created: Tue Aug  3 15:29:28 2010 (at Intel Research, Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
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

local roslua_logging = require("roslua.logging")
local logprint = require("fawkes.logprint")
require("fawkes.depinit")

require("actionlib")
require("fawkes.mathext")
require("skiller.ros")
local actjsmod   = require("skiller.ros.action_jumpstate")
local srvjsmod   = require("skiller.ros.service_jumpstate")
local actinitmod = require("skiller.ros.action_initializer")
local topinitmod = require("skiller.ros.topic_initializer")
local srvinitmod = require("skiller.ros.service_initializer")

function ros_log_fawkes(level, time, msg)
   if level == roslua.logging.DEBUG then
      logprint.print_debug("%s", msg)
   elseif level == roslua.logging.WARN then
      logprint.print_warn("%s", msg)
   elseif level == roslua.logging.ERROR then
      logprint.print_error("%s", msg)
   elseif level == roslua.logging.FATAL then
      logprint.print_error("%s", msg)
   end
   logprint.print_info("%s", msg)
end

roslua_logging.remove_stdout_logger()
roslua_logging.add_logger(ros_log_fawkes)

skiller.ros.init{no_signal_handler = true,
		 no_action_server  = true}

_G.skillenv = require("skiller.skillenv")
skillenv.add_export("ActionJumpState", actjsmod.ActionJumpState)
skillenv.add_export("ServiceJumpState", srvjsmod.ServiceJumpState)
fawkes.depinit.add_module_initializer(actinitmod.init_actions)
fawkes.depinit.add_module_initializer(topinitmod.init_topics)
fawkes.depinit.add_module_initializer(srvinitmod.init_services)
--skillenv.init(SKILLSPACE)

skillenv.add_loop_callback("roslua", roslua.spin)
skillenv.add_finalize_callback("roslua", roslua.finalize)
