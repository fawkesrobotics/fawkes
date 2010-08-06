
----------------------------------------------------------------------------
--  agenthsm.lua - Hybrid State Machine for agents, closely related to FSM
--
--  Created: Fri Jan 02 17:30:28 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
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
local jsmod = require("fawkes.fsm.jumpstate")
local wsmod = require("fawkes.fsm.waitstate")
local ajsmod = require("luaagent.jumpstates")
local subfsmfsmod = require("fawkes.fsm.subfsmjumpstate")

local FSM               = fsmmod.FSM
AgentSkillExecJumpState = ajsmod.AgentSkillExecJumpState
SubFSMJumpState         = subfsmfsmod.SubFSMJumpState
JumpState               = jsmod.JumpState
WaitState               = wsmod.WaitState

--- @class AgentHSM
-- Hybrid state machine specifically for agents. Similar to plain FSM, the major
-- addition is the add_transitions() function.
-- @author Tim Niemueller
AgentHSM = {current = nil,
	    debug   = false,
	    export_states_to_parent = true,
	    set_debug               = FSM.set_debug,
	    set_error               = FSM.set_error,
	    loop                    = FSM.loop,
	    trans                   = FSM.trans,
	    reset                   = FSM.reset,
	    changed                 = FSM.changed,
	    mark_changed            = FSM.mark_changed,
	    set_changed             = FSM.set_changed,
	    graph                   = FSM.graph,
	    traced                  = FSM.traced,
	    traced_state            = FSM.traced_state,
	    traced_trans            = FSM.traced_trans,
	    reset_trace             = FSM.reset_trace,
	    add_state               = FSM.add_state,
	    remove_state            = FSM.remove_state,
	    clear_states            = FSM.clear_states,
	    new_jump_state          = FSM.new_jump_state,
	    get_start_state         = FSM.get_start_state,
	    add_default_transition  = FSM.add_default_transition,
	    apply_deftrans          = FSM.apply_deftrans
	 }


--- Create new AgentHSM.
function AgentHSM:new(o)
   local f = FSM:new(o)
   setmetatable(f, self)
   self.__index = self

   f:clear_states()
   f.exit_state = "FINAL"
   f.fail_state = "FAILED"

   return f
end


--- Simple state generation not supported for AgentHSM.
-- Throws an error. Only jump states can be created for AgentHSMs.
function AgentHSM:new_state()
   error("Only jump states can be created for an AgentHSM")
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


function AgentHSM:add_transitions(trans)
   for _,t in ipairs(trans) do
      if t[2] then -- Normal from -> to transition
	 assert(t[1], "Must have an originating state")
	 assert(not (t[3] and t.cond), "Only one of cond field and third index may be set as condition")

	 local from = t[1]
	 local to   = t[2]
	 local cond = t[3] or t.cond
	 local s    = nil

	 assert(type(from) == "string", "From states must be given by name, not as objects")
	 assert(type(to) == "string", "To states must be given by name, not as objects")

	 -- If we only get a time as timeout assume jump to normal to state
	 if t.timeout and type(t.timeout) == "number" then
	    t.timeout = { t.timeout, to }
	 end
	 if t.timeout then
	    local timeout_to = t.timeout.to or t.timeout[2]
	    if not self.states[timeout_to] then
	       self.states[timeout_to] = JumpState:new{name=timeout_to, fsm=self,
						       closure=trans.closure}
	       self:apply_deftrans(self.states[timeout_to])
	    end
	 end

	 local s
	 if t.skills or t.fsm then  -- This state calls skills

	    if self.states[from] then
	       assert(not self.states[from].skills and not self.states[from].subfsm,
		      self.name .. ": " .. from .. " -> " .. to .. ":Skill exec " ..
		      "state cannot be generated for existing skill or sub-FSM state")
	       if self.debug then
		  printf("%s: Erasing state %s and re-creating as sub-skill state",
			 self.name, from)
	       end
	       self.states[from] = nil
	    end

	    if t.skills then

	       if not t.fail_to then
		  t.fail_to = to
	       end

	       if self.debug then
		  printf("%s: %s -> %s/%s for skill execution of %d skill(s)",
			 self.name, from, to, t.fail_to, #t.skills)
	       end

	       if not self.states[t.fail_to] then
		  self.states[t.fail_to] = JumpState:new{name=t.fail_to, fsm=self,
							 closure=trans.closure}
		  self:apply_deftrans(self.states[t.fail_to])
	       end

	       s = AgentSkillExecJumpState:new{name=from, fsm=self, closure=trans.closure,
					       skills=t.skills, final_state=to,
					       failure_state=t.fail_to,
					       dotattr=t.from_dotattr,
					       final_dotattr=t.final_dotattr,
					       failure_dotattr=t.failure_dotattr,
					       timeout=t.timeout}
	    elseif t.fsm then -- This state executes a sub-FSM

	       if self.debug then
		  printf("%s: %s -> %s/%s for FSM execution of %s",
			 self.name, from, to, tostring(t.fail_to), t.fsm.name)
	       end

	       s = SubFSMJumpState:new{name=from, fsm=self, closure=trans.closure,
				       subfsm=t.fsm, exit_to=to, timeout=t.timeout}
	    end
	 elseif t.wait_sec ~= nil then -- Wait state
	    if self.states[from] then
	       assert(not self.states[from].skills and not self.states[from].subfsm,
		      self.name .. ": " .. from .. " -> " .. to .. ": Wait state " ..
		      "state cannot be generated for existing skill or sub-FSM state")
	       if self.debug then
		  printf("%s: Erasing state %s and re-creating as sub-skill state",
			 self.name, from)
	       end
	       self.states[from] = nil
	    end
	    s = WaitState:new{name=from, fsm=self, next_state=to,
			      time_sec=t.wait_sec, labeltime=t.labeltime,
			      closure=trans.closure}

	 else              -- Simple state
	    printf("Creating simple state %s", from)
	    if not self.states[from] then
	       s = JumpState:new{name=from, fsm=self, closure=trans.closure,
				 dotattr=trans.from_dotattr, timeout=t.timeout}
	    else
	       if t.from_dotattr then
		  local fsd = self.states[from].dotattr
		  for k,v in pairs(t.from_dotattr) do fsd[k] = v end
	       end
	    end
	 end

	 if s then
	    self.states[from] = s
	    self:apply_deftrans(s)
	 end
	 assert(self.states[from], "From state not created, serious bug")
	 s = self.states[from]

	 if not self.states[to] then
	    if self.debug then
	       printf("%s: Creating blanko to state %s (from %s)", self.name, to, from)
	    end
	    self.states[to] = JumpState:new{name=to, fsm=self,
					    closure=trans.closure}
	    self:apply_deftrans(self.states[to])
	 else
	    if t.to_dotattr then
	       local tsd = self.states[to].dotattr
	       for k,v in pairs(t.to_dotattr) do tsd[k] = v end
	    end
	 end

	 if cond then -- We have an (extra) condition, add it
	    if self.debug then
	       printf("%s: %s -> %s, adding custom jump condition %s (desc: %s)",
		      self.name, from, to, tostring(cond), tostring(t.desc))
	    end
	    local tr = s:add_transition(self.states[to].name, cond, t.desc)
	    if t.trans_dotattr then tr.dotattr = t.trans_dotattr end
	    if t.precond then
	       if self.debug then
		  printf("%s: Making transition %s -> %s a precondition",
			 self.name, from, to)
	       end
	       s:add_precondition(tr)
	    end
	 else
	    assert(not t.precond, "Precondition can only be set with specific condition")
	 end

	 if t.init then
	    s.init = t.init
	 end

      else  -- default transition
	 local to = t[1]
	 assert(to, "Default transition must have a target state as first argument")
	 assert(t.cond, "Default transition must have a jump condition")

	 if self.debug then
	    printf("%s: Adding default transition to %s (cond: %s, desc: %s)",
		   self.name, to, tostring(t.cond), tostring(t.desc))
	 end

	 if not self.states[to] then
	    self.states[to] = JumpState:new{name=to, fsm=self,
					    closure=trans.closure}
	    self:apply_deftrans(self.states[to])
	 end

	 self:add_default_transition(self.states[to].name, t.cond, t.desc)
      end
   end

   if self.export_states_to_parent then
      local e = getfenv(2)
      for name,s in pairs(self.states) do
	 e[name] = s
      end
   end
end
