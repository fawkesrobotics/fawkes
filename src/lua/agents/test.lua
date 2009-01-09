
----------------------------------------------------------------------------
--  test.lua - Simple test agent
--
--  Created: Fri Jan 02 17:42:15 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
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

-- Initialize module
module(..., agentenv.module_init)

-- Crucial skill information
name               = "test"
fsm                = AgentHSM:new{name=name, start="INITIAL", exit_state = "FINAL"}

documentation      = "Simple test agent to demonstrate agent/skiller integration."

-- Initialize as agent module
agentenv.agent_module(...)

-- States
fsm:new_jump_state("INITIAL")
fsm:new_jump_state("FINAL")
fsm:new_skill_state("SAY", {{"say", {text="Agent test successful"}}},
		    FINAL, FINAL)

-- Code
INITIAL:add_transition(SAY, function (state) return true end, "Always true")
