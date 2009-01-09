
----------------------------------------------------------------------------
--  agentenv.lua - Agent environment functions
--
--  Created: Fri Jan 02 14:44:24 2008
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

require("fawkes.modinit")
module(..., fawkes.modinit.register_all)
require("fawkes.logprint")
local skillstati = require("skiller.skillstati")
local hsmmod = require("luaagent.agenthsm")
local skqmod = require("luaagent.skillqueue")
local fsmjsmod = require("fawkes.fsm.jumpstate")
local fsmstmod = require("fawkes.fsm.state")
local skillenv = require("skiller.skillenv")

local agent = nil

--- Initialize agent module.
-- Exports some basic features like AgentHSM, SkillQueue, JumpState etc.
-- into the given module.
-- @param m Module to initialize
function module_init(m)
   fawkes.modinit.module_init(m)
   m.AgentHSM   = hsmmod.AgentHSM
   m.SkillQueue = skqmod.SkillQueue
   m.State      = fsmstmod.State
   m.JumpState  = fsmjsmod.JumpState

   for k,v in pairs(skillstati) do
      if string.match(k, "^S_([%a_]+)$") then m[k] = v end
   end
end

--- Initialize agent module.
-- Initializes the agent module by providing and assuring the required
-- interfaces etc.
-- @param module_name name of the agent module.
function agent_module(module_name)
   local m = require(module_name)
   assert(m, "Could not load agent module")

   local orig_depends_skills = m.depends_skills
   m.depends_skills = nil
   skillenv.skill_module(module_name)
   m.depends_skills = orig_depends_skills
end

--- Init agentenv.
-- Loads the given agent. If it has a init() routine this is called,
-- otherwise if the agent as a FSM this is reset.
function init(agentname)
   agent = require("agents." .. agentname)
   assert(agent, "Agent " .. agentname .. " could not be loaded")

   if agent.init then
      agent.init()
   elseif agent.fsm then
      agent.fsm:reset()
      skillenv.write_fsm_graph(agent.fsm, interfaces.writing.agdbg)
   end
end


--- Write graph of current agent.
-- If the current agent supplies a FSM its graph is written to the
-- agent debug interface (interfaces.writing.agdbg).
function write_graph()
   if agent.fsm then
      skillenv.write_fsm_graph(agent.fsm, interfaces.writing.agdbg)
   end
end


--- Execute current agent.
-- Calls the agent's execute routine. If this does not exist and the
-- agent has an FSM the FSM is run via FSM:loop(), otherwise an error
-- is thrown.
function execute()
   if agent.execute then
      agent.execute()
   elseif agent.fsm then
      agent.fsm:loop()
   else
      error("Agent has neither execute() function nor FSM")
   end

   write_graph()
end
