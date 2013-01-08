
----------------------------------------------------------------------------
--  start.lua - LuaAgent Lua start code
--
--  Created: Fri Jan 02 14:47:07 2008
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

require("fawkes.logprint")
fawkes.logprint.init(logger)

require("fawkes.mathext")
require("fawkes.depinit")

require("fawkes.interface_initializer")

agentenv = require("luaagent.agentenv")
fawkes.depinit.add_module_initializer(fawkes.interface_initializer.init_interfaces)
agentenv.init(AGENT)

logger:log_debug("Lua startup completed")
