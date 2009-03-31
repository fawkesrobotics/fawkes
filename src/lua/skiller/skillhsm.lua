
----------------------------------------------------------------------------
--  skillhsm.lua - Hybrid State Machine for skills, closely related to FSM
--
--  Created: Mon Dec 22 11:53:39 2008
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
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
local jsmod = require("fawkes.fsm.jumpstate")
local sjsmod = require("skiller.skill_jumpstates")
local subfsmjsmod = require("fawkes.fsm.subfsmjumpstate")

local FSM         = fsmmod.FSM
JumpState         = jsmod.JumpState
SkillJumpState    = sjsmod.SkillJumpState
SubFSMJumpState   = subfsmjsmod.SubFSMJumpState


--- @class SkillHSM
-- Hybrid state machine for skills.
-- @author Tim Niemueller
SkillHSM = {current                 = nil,
	    debug                   = false,
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
	    get_start_state         = FSM.get_start_state,
	    add_default_transition  = FSM.add_default_transition,
	    apply_deftrans          = FSM.apply_deftrans,
	    new_wait_state          = FSM.new_wait_state
	   }

--- Constructor.
-- @param o object pre-initializer, must be a table.
function SkillHSM:new(o)
   local f = FSM:new(o)
   setmetatable(o, self)
   self.__index = self

   f:clear_states()

   return f
end

--- Clear all states.
-- Removes all states. If no_default_states is not set a FINAL and FAILED state
-- are added.
function SkillHSM:clear_states()
   self.states = {}
   if not self.no_default_states then
      self.exit_state = "FINAL"
      self.fail_state = "FAILED"

      local es = SkillJumpState:new{name = "FINAL", fsm = self}
      local fs = SkillJumpState:new{name = "FAILED", fsm = self}

      self.states["FINAL"] = es
      self.states["FAILED"] = fs

      if self.export_states_to_parent then
	 local e = getfenv(2)
	 if e == _M then
	    e = getfenv(3)
	 end
	 e["FINAL"] = es
	 e["FAILED"] = fs
      end
   end
   self.state_changed = true
end

--- Simple state generation not supported for SkillHSM.
-- Throws an error. Only jump states can be created for SkillHSMs.
function SkillHSM:new_state()
   error("Only jump states can be created for a SkillHSM")
end


--- Create a new jump states for a skill HSM.
-- @see FSM:new_jump_state
-- @param name name of the state and the variable in the environment of the caller
-- that holds this state
-- @param subskill single or multiple subskills. If subskill is nil then a
-- SkillJumpState with no sub-skills is instantiated. If subskill is a table then
-- a SkillJumpState with this table as the list of sub-skills is instantiated. If
-- subskill is not nil and not a table it is assumed that it is the module of a
-- single skill and a SkillJumpState for single-skill execution is instantiated
-- with final_goto and failure_goto as the outcome states.
-- @param final_goto The state the SubSkillJumpState jumps to on successful
-- execution
-- @param failure_goto The state the SubSkillJumpState jumps to on failed
-- execution
function SkillHSM:new_jump_state(name, subskill, final_state, failure_state, args)
   local s
   if subskill ~= nil then
      assert(type(subskill) == "table", "subskill(s) must be a table")
      if subskill.execute and subskill.reset then
	 --printf("Creating SubSkillJumpState 1 name: " .. name)
	 s = SkillJumpState:new{name          = name,
				fsm           = self,
				skill         = subskill,
				final_state   = final_state,
				failure_state = failure_state,
				args          = args}
      else
	 --printf("Creating SkillJumpState 2 name: " .. name)
	 s = SkillJumpState:new{name = name, fsm = self,
				subskills     = subskill,
				final_state   = final_state,
				failure_state = failure_state}
      end
   else
      --printf("Creating SkillJumpState 3 name: " .. name)
      s = SkillJumpState:new{name = name, fsm = self}
   end

   self.states[name] = s
   self:apply_deftrans(s)

   if self.export_states_to_parent then
      local e = getfenv(2)
      e[name] = s
   end

   return s
end


--- Add many transitions and auto-create states.
-- This method is a convenience methods to add many transitions with one call and
-- generate all required and non-existing states, ideally allowing to specify the
-- whole HSM in one go.
-- @param trans a table consisting of transitions to add. Each entry is another
-- table, consisting of the following.
-- Up to three indexed values.
-- First is from state, second is to state. Third is an optional jump condition,
-- which can be passed as appropriate for add_transition(). If only the first
-- is given, it is the target state for a default transition.
-- The following named elements can exist:
-- - cond jump condition, required for default transition
-- - skill skill to execute in the source state, on success transition to target
-- state is executed. fail_to can be set to the target state in case the skill
-- fails. Otherwise "FAILED" is implied.
-- - skills skills to execute in the source state, on success transition to target
-- state is executed. fail_to can be set to the target state in case the skill
-- fails. Otherwise "FAILED" is implied.
-- - subskills sub-skills to reset when entering and leaving the source state. No
-- skill is executed. The skill_status value of the state is evaluated for a
-- transition to the target state if the status is S_FINAL. fail_to can be set
-- to the target state in case skill_status is S_FAILED. Otherwise "FAILED" is
-- implied. The loop field must be set to a loop function that executes the states.
-- - args can be set to arguments passed for skills
-- - loop can be set to a function executed when the state is active
-- - init init function for the state
-- - precond if true, the transition is marked as a precondition
-- - desc description of the transition
-- - cond jump condition that must fire to execute the transition
--
-- Additionally the trans table may contain the element closure, which must be
-- an associative table. Each element will be assigned to the function environment
-- of jump conditions that are passed as string. In particular you need to add
-- predicate libraries to the closure.
function SkillHSM:add_transitions(trans)
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

	 if t.skill or t.skills or t.subskills then -- Sub-skill(s)
	    local fail_to = t.fail_to or "FAILED"

	    if not self.states[fail_to] then
	       self.states[fail_to] = SkillJumpState:new{name=fail_to, fsm=self, closure=trans.closure}
	       self:apply_deftrans(self.states[fail_to])
	    end

	    if self.states[from] then
	       local f = self.states[from]
	       assert(not f.skill and not next(f.subskills),
		      self.name .. ": " .. from .. " -> " .. to .. ": " ..
		      "The originating state for a sub-skill execution may " ..
		      "only exist if it has no sub-skills assigned")
	       if self.debug then
		  printf("%s: Erasing state %s and re-creating as sub-skill state",
			 self.name, from)
	       end
	       self.states[from] = nil
	    end

	    if t.skill then     -- subskill
	       if self.debug then
		  printf("%s: %s -> %s/%s for single sub-skill %s",
			 self.name, from, to, fail_to, t.skill.name)
	       end

	       s = SkillJumpState:new{name=from, fsm=self, skill=t.skill,
				      final_state=to, failure_state=fail_to,
				      args=t.args, closure=trans.closure}

	    elseif t.skills then     -- execute multiple subskills
	       if self.debug then
		  printf("%s: %s -> %s/%s for multi sub-skill execution %s",
			 self.name, from, to, fail_to, t.skill.name)
	       end

	       s = SkillJumpState:new{name=from, fsm=self, skills=t.skills,
				      final_state=to, failure_state=fail_to,
				      args=t.args, closure=trans.closure}

	    elseif t.subskills then -- multiple non-execute subskills, must have a loop to call them
	       local fail_to = t.fail_to and t.fail_to.name or "FAILED"

	       assert(t.loop, "A state with multiple non-exec subskills must have a loop function")

	       if self.debug then
		  printf("%s: %s -> %s/%s for multi sub-skills %s",
			 self.name, from, to, fail_to,
			 string.join(", ", t.subskills))
	       end

	       s = SkillJumpState:new{name=from, fsm=self, subskills=t.subskills,
				      final_state=to, failure_state=fail_to,
				      closure=trans.closure}
	    end
	    -- Set loop function if supplied
	    if t.loop then s.loop = t.loop end

	 else -- simple jump state without any sub-skills
	    if self.debug then
	       printf("%s: Creating blanko from state %s (to %s)", self.name, from, to)
	    end
	    if not self.states[from] then
	       s = SkillJumpState:new{name=from, fsm=self, closure=trans.closure}
	    elseif self.debug then
	       printf("%s: From state %s -> %s already exists", self.name, from, to)
	    end
	 end

	 if not self.states[to] then
	    if self.debug then
	       printf("%s: Creating blanko to state %s (from %s)", self.name, to, from)
	    end
	    self.states[to] = SkillJumpState:new{name=to, fsm=self, closure=trans.closure}
	    self:apply_deftrans(self.states[to])
	 end

	 if s then
	    self.states[from] = s
	    self:apply_deftrans(s)
	 end

	 assert(self.states[from], "From state not created, serious bug")
	 s = self.states[from]

	 if cond then -- We have an (extra) condition, add it
	    if self.debug then
	       printf("%s: %s -> %s, adding custom jump condition %s (desc: %s)",
		      self.name, from, to, tostring(cond), tostring(t.desc))
	    end
	    if self.debug then
	       for _,at in ipairs(s.transitions) do
		  printf("BEFORE %s transition to %s\n", s.name, at.state)
	       end
	    end
	    local t = s:add_transition(self.states[to].name, cond, t.desc)
	    if self.debug then
	       for _,at in ipairs(s.transitions) do
		  printf("AFTER %s transition to %s\n", s.name, at.state)
	       end
	    end
	    if t.precond then
	       if self.debug then
		  printf("%s: Making transition %s -> %s a precondition",
			 self.name, from, to)
	       end
	       s:add_precondition(t)
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
	    self.states[to] = SkillJumpState:new{name=to, fsm=self, closure=trans.closure}
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
