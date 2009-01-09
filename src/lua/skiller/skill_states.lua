
------------------------------------------------------------------------
--  skill_states.lua - FSM states specifically for skills
--
--  Created: Tue Oct 07 17:38:58 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
--  $Id: fsm.lua 1311 2008-07-31 11:44:00Z tim $
--
------------------------------------------------------------------------

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
local fsmmod = require("fawkes.fsm");

--- This module provides a generic finite state machine (FSM).
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

local skillstati = require("skiller.skillstati")
local State = fsmmod.State

--- @class SkillState
-- FSM state specifically designed for skills.
-- The SkillState can be used in the place of a regular State, they can also be
-- intermixed. Like a regular State, transitions are done by returning a follow
-- state from within init() or loop(). As a specialty for a SkillState you can
-- add a number of sub-skills. These skills are automatically reset during
-- init() and exit().
-- @author Tim Niemueller
SkillState = { do_loop         = State.do_loop,
	       loop            = State.loop,
	       init            = State.init,
	       exit            = State.exit,
	       reset           = State.reset,
	       add_transition  = State.add_transition,
	       get_transitions = State.get_transitions,
	       last_transition = State.last_transition,
	    }

--- Create new skill state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SkillState:new(o)
   assert(o, "SkillState requires a table as argument")
   assert(o.name, "SkillState requires a name")
   assert(o.fsm, "SkillState " .. o.name .. " requires a FSM")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.subskills = o.subskills or {}
   o.transitions = o.transitions or {}
   o.dotattr = o.dotattr or {}

   return o
end

--- Execute init routines.
-- This resets any subskills that have been added for this state and then executes
-- the state's init() routine. Do not overwrite do_init(), rather implement init().
-- @param ... Any parameters, passed to init()
function SkillState:do_init(...)
   for _, s in ipairs(self.subskills) do
      s.reset()
   end
   return self:init(...)
end


--- Execute exit routine.
-- This resets any subskills that have been added for this state and then executes
-- the state's exit() routine. Do not overwrite do_exit(), rather implement exit().
function SkillState:do_exit()
   for _, s in ipairs(self.subskills) do
      s.reset()
   end
   self:exit()
end


--- Add a subskill to this state.
-- Skills which are added as subskills are automatically reset during init and
-- exit.
-- @param subskill subskill to add
function SkillState:add_subskill(subskill)
   table.insert(self.subskills, subskill)
end


--- @class SubSkillState
-- FSM state to run a particular skill.
-- This special skill state is meant for running a single sub-skill and executing
-- transitions depending on the skill's outcome. Any arguments that are passed to
-- a SubSkillState during init() are given verbatim to the skill's execute()
-- function.
-- @author Tim Niemueller
SubSkillState = {get_transitions = State.get_transitions,
		 last_transition = State.last_transition,
	         reset           = State.reset}

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SubSkillState:new(o)
   assert(o, "SubSkillState requires a table as argument")
   assert(o.name, "SubSkillState requires a name")
   assert(o.fsm, "SubSkillState " .. o.name .. " requires a FSM")
   assert(o.skill, "SubSkillState requires a skill to execute")
   assert(o.final_state, "SubSkillState requires state to go to on success")
   assert(o.failure_state, "SubSkillState requires state to go to on failure")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.transitions = o.transitions or {}
   o.dotattr = o.dotattr or {}
   assert(type(o.transitions) == "table", "Transitions for " .. o.name .. " not a table")

   o:add_transitions(self.final_state, self.failure_state)

   return o
end

--- Execute init routines.
-- This resets any subskills that have been added for this state and then executes
-- the state's init() routine. Do not overwrite do_init(), rather implement init().
-- @param ... Any parameters, passed to init()
function SubSkillState:do_init(...)
   self.args = { ... }
   self.skill.reset()
end


--- Execute exit routine.
-- This resets any subskills that have been added for this state and then executes
-- the state's exit() routine. Do not overwrite do_exit(), rather implement exit().
function SubSkillState:do_exit()
   self.skill.reset()
end


--- Execute skill.
function SubSkillState:do_loop()
   local skill_status = self.skill.execute(self.args)
   if skill_status == skillstati.S_FINAL then
      self.last_trans = self:get_transitions(self.final_state)
      return self.final_state
   elseif skill_status == skillstati.S_FAILURE then
      self.last_trans = self:get_transitions(self.failure_state)
      return self.failure_state
   end
end


--- Create a new skill state.
-- This creates a new skill state for regular FSMs. Augment your FSM with
-- this new_state method.
-- @param name name of the state and the variable in the environment of the caller
-- that holds this state
-- @param subskill single or multiple subskills. If subskill is nil then a
-- SkillJumpState with no sub-skills is instantiated. If subskill is a table then
-- a SkillJumpState with this table as the list of sub-skills is instantiated. If
-- subskill is not nil and not a table it is assumed that it is the module of a
-- single skill and a SubSkillJumpState is instantiated with final_goto and
-- failure_goto as the outcome states.
-- @param final_goto The state the SubSkillJumpState jumps to on successful
-- execution
-- @param failure_goto The state the SubSkillJumpState jumps to on failed
-- execution
--@return new state
function new_state(self, name, subskill, final_state, failure_state)
   local s
   if subskill ~= nil then
      assert(type(subskill) == "table", "subskill(s) must be a table")
      if subskill.execute and subskill.reset then
	 --printf("Creating SubSkillState 1 name: " .. name)
	 s = SubSkillState:new{name          = name,
			       fsm           = self,
			       skill         = subskill,
			       final_state   = final_state,
			       failure_state = failure_state,
			       args          = args}
      else
	 --printf("Creating SkillState 1 name: " .. name)
	 s = SkillState:new{name = name, fsm = self, subskills = subskill}
      end
   else
      --printf("Creating SkillState 2 name: " .. name)
      s = SkillState:new{name = name, fsm = self}
   end

   self.states[name] = s

   if self.export_states_to_parent then
      local e = getfenv(2)
      e[name] = s
   end

   return s
end
