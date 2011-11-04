
----------------------------------------------------------------------------
--  start.lua - Lua Agent startup script -- ROS version
--
--  Created: Thu Sep  2 15:07:16 2010 (at Intel Research, Pittsburgh)
--  Copyright  2010  Tim Niemueller [www.niemueller.de]
--             2010  Carnegie Mellon University
--             2010  Intel Labs Pittsburgh
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

require("roslua.logging")
require("fawkes.logprint")
roslua.logging.register_print_funcs(fawkes.logprint)

require("fawkes.mathext")
require("fawkes.depinit")
require("luaagent.ros")
require("skiller.ros.topic_initializer")
require("skiller.ros.service_initializer")
local srvjsmod = require("skiller.ros.service_jumpstate")

luaagent.ros.init()

agentenv = require("luaagent.agentenv")
agentenv.add_export("ServiceJumpState", srvjsmod.ServiceJumpState)
fawkes.depinit.add_module_initializer(skiller.ros.topic_initializer.init_topics)
fawkes.depinit.add_module_initializer(skiller.ros.service_initializer.init_services)
