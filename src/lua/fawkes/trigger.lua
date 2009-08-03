
------------------------------------------------------------------------
--  fsm.lua - Lua Finite State Machines (FSM)
--
--  Created: Fri Jun 13 11:25:36 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

--- This module provides a generic trigger facility.
-- @author Tim Niemueller
module(..., fawkes.modinit.register_all)

TriggerManager = {}

function TriggerManager:new(o)
   assert(o, "State requires a table as argument")
   assert(o.name, "State requires a name")
   assert(not getmetatable(o), "Meta table already set for object")
   setmetatable(o, self)
   self.__index = self

   o.triggers = {}

   return o
end


function TriggerManager:add_trigger(event, func)
   if not self.triggers then
      self.triggers = {}
   end
   if not self.triggers[event] then
      self.triggers[event] = {}
   end
   table.insert(self.triggers[event], func)
end


function TriggerManager:remove_trigger(event, func)
   if self.triggers[event] then
      for i, f in ipairs(self.triggers[event]) do
	 if f == func then
	    table.remove(self.triggers[event], i)
	 end
      end
   end    
end

function TriggerManager:trigger(event, ...)
   if self.triggers[event] then
      local event_time = os.time()
      for _, f in ipairs(self.triggers[event]) do
	 f(event, event_time, ...)
      end
   -- else: no triggers registered for this event
   end
end
