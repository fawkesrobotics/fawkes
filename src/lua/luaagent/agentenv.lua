
----------------------------------------------------------------------------
--  agentenv.lua - Agent environment functions
--
--  Created: Fri Jan 02 14:44:24 2008
--  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
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
local agjsmod  = require("luaagent.jumpstates")
local fsmjsmod = require("fawkes.fsm.jumpstate")
local shsmmod  = require("fawkes.fsm.subfsmjumpstate")
local fsmstmod = require("fawkes.fsm.state")
local skillenv = require("skiller.skillenv")
local predlib  = require("fawkes.predlib")
local grapher  = require("fawkes.fsm.grapher")

local agent = nil

local graphing_enabled = true

local module_exports = {
   AgentHSM                = hsmmod.AgentHSM,
   AgentSkillExecJumpState = agjsmod.AgentSkillExecJumpState,
   SkillQueue              = skqmod.SkillQueue,
   JumpState               = fsmjsmod.JumpState,
   SubFSMJumpState         = shsmmod.SubFSMJumpState,
   config                  = config
}

add_module_initializer = skillenv.add_module_initializer

--- Add an export for module initialization.
-- All exports are exported to modules when they are initialized.
-- @param key key of the export, i.e. the name with which the value will be
-- available in the skill module
-- @param value the value of the exported entry
function add_export(key, value)
   module_exports[key] = value
end

--- Initialize agent module.
-- Exports some basic features like AgentHSM, SkillQueue, JumpState etc.
-- into the given module.
-- @param m Module to initialize
function module_init(m)
   fawkes.modinit.module_init(m)

   for k, v in pairs(module_exports) do
      m[k] = v
   end

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
   local ok
   agent = require("agents." .. agentname)

   assert(agent, "Agent " .. agentname .. " could not be loaded")

   if agent.init then
      agent.init()
   elseif agent.fsm then
      agent.fsm:reset()
      if interfaces then
	 skillenv.write_fsm_graph(agent.fsm, interfaces.writing.agdbg)
      end
   end
end

--- Set graph rankdir.
-- @param graphdir rank dir
-- @see fawkes.fsm.grapher.set_rankdir()
function set_graphdir(graphdir)
   if agent.fsm then
      if skillenv.update_grapher_config(interfaces.writing.agdbg, graphdir) then
	 agent.fsm:mark_changed()
      end
   end
end

--- Set if the graph should be colored.
-- @param colored true to enable colored graph output, false otherwise
function set_graph_colored(colored)
   if agent.fsm then
      if skillenv.update_grapher_config(interfaces.writing.agdbg, nil, colored) then
	 agent.fsm:mark_changed()
      end
   end
end

--- Write graph of current agent.
-- If the current agent supplies a FSM its graph is written to the
-- agent debug interface (interfaces.writing.agdbg).
function write_graph(fsm)
   if fsm then
      skillenv.write_fsm_graph(agent.fsm, interfaces.writing.agdbg)
   end
end


--- Error handling function.
-- This function can be overridden. It is called if the agent's execute fails.
-- The default implementation just raises an error.
-- @param err error message
function handle_error(err)
   error(err)
end


--- Execute current agent.
-- Calls the agent's execute routine. If this does not exist and the
-- agent has an FSM the FSM is run via FSM:loop(), otherwise an error
-- is thrown.
function execute()
   if agent.execute then
      local ok, err = pcall(agent.execute)
      if not ok then
         handle_error(err)
      end
   elseif agent.fsm then
      local ok, err = pcall(agent.fsm.loop, agent.fsm)
      if not ok then
         handle_error(err)
      end      
   else
      handle_error("Agent has neither execute() function nor FSM")
   end

   predlib.reset()

   if graphing_enabled then
      write_graph(agent.fsm)
   end
   if status_publisher then
      status_publisher(agent.fsm)
   end
end
