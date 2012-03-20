
----------------------------------------------------------------------------
--  init.lua - Continuous agent demo
--
--  Created: Thu May 26 15:53:18 2011
--  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

-- Initialize module
module(..., agentenv.module_init)

-- Crucial skill information
name               = "contdemo"

documentation      = [==[Continuous demo agent.]==]

-- Initialize as agent module
agentenv.agent_module(...)

local socket = require("socket")

function execute()
   while true do
      read_interfaces()
      printf("LOOP")
      socket.sleep(5.0)
      write_interfaces()
   end
end
