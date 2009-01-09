
----------------------------------------------------------------------------
--  agenthsm.lua - Hybrid State Machine for agents, closely related to FSM
--
--  Created: Fri Jan 02 17:30:28 2009
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

require("fawkes.modinit")

--- Hybrid State Machine for skills.
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)
local fsmmod = require("fawkes.fsm")
local ajsmod = require("luaagent.jumpstates")

local FSM               = fsmmod.FSM
AgentSkillExecJumpState = ajsmod.AgentSkillExecJumpState

--- @class AgentHSM
-- Hybrid state machine specifically for agents. Similar to plain FSM, the major
-- addition is the new_skill_state() method to easily add an
-- AgentSkillExecJumpState.
-- @author Tim Niemueller
AgentHSM = {current = nil,
	    debug   = false,
	    export_states_to_parent = true,
	    set_debug      = FSM.set_debug,
	    loop           = FSM.loop,
	    trans          = FSM.trans,
	    reset          = FSM.reset,
	    changed        = FSM.changed,
	    mark_changed   = FSM.mark_changed,
	    graph          = FSM.graph,
	    traced         = FSM.traced,
	    traced_state   = FSM.traced_state,
	    traced_trans   = FSM.traced_trans,
	    reset_trace    = FSM.reset_trace,
	    add_state      = FSM.add_state,
	    new_jump_state = FSM.new_jump_state,
	    get_start_state = FSM.get_start_state
	  }



--- Create new AgentHSM.
function AgentHSM:new(o)
   local f = FSM:new(o)
   setmetatable(o, self)
   self.__index = self

   return o
end


--- Simple state generation not supported for AgentHSM.
-- Throws an error. Only jump states can be created for AgentHSMs.
function AgentHSM:new_state()
   error("Only jump states can be created for a SkillHSM")
end


--- Add new AgentSkillExecJumpState.
-- @param name name of state
-- @param skills skills to add to the states skill queue
-- @param final_state state to trans to upon successful skill execution
-- @param failure_state state to trans to on failure
function AgentHSM:new_skill_state(name, skills, final_state, failure_state)
   assert(name, "AgentHSM:new_skill_state: no name given")
   assert(name, "AgentHSM:new_skill_state: no skills given")

   local s = AgentSkillExecJumpState:new{name=name, fsm=self, skills=skills,
					 final_state=final_state,
					 failure_state = failure_state}

   self.states[name] = s
   if self.export_states_to_parent then
      local e = getfenv(2)
      e[name] = s
   end

   return s
end
