
------------------------------------------------------------------------
--  sfsm.lua - Lua Finite State Machines (FSM) for Skills
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
local fsm = require("fawkes.fsm");

--- This module provides a generic finite state machine (FSM).
-- @author Tim Niemueller
module(..., fawkes.modinit.module_init)

FSM = fsm.FSM

--- @class State
-- State for an FSM.
SkillState = {}

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SkillState:new(o)
   assert(o, "SkillState requires a table as argument")
   assert(o.name, "SkillState requires a name")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.subskills = {}

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


function SkillState:add_subskill(subskill)
   table.insert(self.subskills, subskill)
end

--- Init function.
-- The init function is called when you enter a state. It may take an arbitrary
-- number of arguments, which may be passed from other states.
function SkillState:init()
end


--- Loop function.
-- This function is called in every loop. You can for example check conditions
-- for state changes.
-- @return return nil to stay in this state, or another state to change the state
-- to this state. Additionally you can return any number of arguments (only if
-- the returned state is not nil). These arguments are passed to the init()
-- function of the state we transit to.
function SkillState:loop()
end


--- Exit function.
-- This function is called by the FSM just before the state is left. If any special
-- operations are necessary (like retracting an arm) for a safe transition out of
-- this state add them here.
function SkillState:exit()
end


--- @class State
-- State for an FSM.
SubSkillState = {}

--- Create new state.
-- @param o table with initializations for the object.
-- @return Initialized FSM state
function SubSkillState:new(o)
   assert(o, "SkillState requires a table as argument")
   assert(o.name, "SkillState requires a name")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.subskills = {}

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


function SkillState:add_subskill(subskill)
   table.insert(self.subskills, subskill)
end

--- Init function.
-- The init function is called when you enter a state. It may take an arbitrary
-- number of arguments, which may be passed from other states.
function SkillState:init()
end


--- Loop function.
-- This function is called in every loop. You can for example check conditions
-- for state changes.
-- @return return nil to stay in this state, or another state to change the state
-- to this state. Additionally you can return any number of arguments (only if
-- the returned state is not nil). These arguments are passed to the init()
-- function of the state we transit to.
function SkillState:loop()
end


--- Exit function.
-- This function is called by the FSM just before the state is left. If any special
-- operations are necessary (like retracting an arm) for a safe transition out of
-- this state add them here.
function SkillState:exit()
end


--- Convenience method to create a new state.
-- Creates a new state in the environment of the caller of this function with the
-- given name.
-- @param name name of the state and the variable in the environment of the caller
-- that holds this state
-- @param init_table all values from this table will be added to the newly created
-- state.
function new_skill_state(name, subskill, final_goto, failure_goto)
   local e = getfenv(2)
   local s
   if subskill ~= nil then
      s = SubSkillState:new{name = name, skill = subskill,
			    final = final_goto, failure = failure_goto}
   else
      s = SkillState:new{name = name}
   end
   e[name] = s
end
