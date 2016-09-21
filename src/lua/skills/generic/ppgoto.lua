
----------------------------------------------------------------------------
--  ppgoto.lua - generic pathplan goto
--
--  Created: Tue Jun 16 10:34:23 2009
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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
module(..., skillenv.module_init)

-- Crucial skill information
name               = "ppgoto"
fsm                = SkillHSM:new{name=name, start="PPGOTO", debug=true}
depends_skills     = nil
depends_interfaces = {
   {v = "ppnavi", id = "Pathplan", type = "NavigatorInterface"}
}

documentation      = [==[Pathplan goto skill.
This skill takes you to a place using a pathplan facility. The path planning
itself is not implemented in the skill, rather it uses the NavigatorInterface
to instruct the appropriate component.

There are several forms to call this skill:
1. ppgoto{x=X, y=Y[, ori=ORI]}
   This will goto the position giving in the global cartesian coordinates,
   optionally with the given orientation. The path planner will use the plan
   nodes to go as close to the desired position as possible and will issue a
   relative goto to reach the final position from there.
2. ppgoto{place=PLACE}
   Go to the given place.
2. ppgoto{place=PLACE, ori=ORI}
   Go to the given place and attain the given orientation.
   This will override any orientation that might be set for the node.
3. ppgoto{stop=true}
   Stop the current pathplan goto.

Parameters:
x, y:      global world cartesian coordinates of target point
ori:       orientation of robot at destination, radian offset from forward
           clock-wise positive
place:     name of a place
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Jumpconditions
function jumpcond_paramfail(state)
   return state.fsm.vars.param_fail
end

function jumpcond_navifail(state)
   return (state.fsm.vars.msgid == 0
     or (state.fsm.vars.msgid ~= ppnavi:msgid() and state.wait_start > 20)
     or not ppnavi:has_writer()
     or state.failed)
end

function jumpcond_navifinal(state)
   --printf("msgid: %d/%d  final: %s", state.fsm.vars.msgid, ppnavi:msgid(), tostring(ppnavi:is_final()))
   return state.fsm.vars.msgid == ppnavi:msgid() and ppnavi:is_final()
end

-- States
fsm:define_states{
   export_to=_M,
   closure={ppnavi=ppnavi},

   {"PPGOTO", JumpState},
   {"CHECK_ERROR", JumpState}
}

-- Transitions
fsm:add_transitions{
   {"PPGOTO", "FAILED", cond_and_precond="not ppnavi:has_writer()", desc="No writer for interface"},
   {"PPGOTO", "FAILED", cond=jumpcond_paramfail, desc="Invalid/insufficient parameters"},
   {"PPGOTO", "FAILED", cond=jumpcond_navifail, desc="Navigator failure"},
   {"PPGOTO", "CHECK_ERROR", cond=jumpcond_navifinal, desc="Navigator final"},

   {"CHECK_ERROR", "FINAL", cond="ppnavi:error_code() == ppnavi.ERROR_NONE", desc="Position reached"},
   {"CHECK_ERROR", "FAILED", cond=true, desc="Navigator error"}
}

function PPGOTO:init()
   if self.fsm.vars.x ~= nil and self.fsm.vars.y ~= nil then
      -- cartesian goto
      local x = self.fsm.vars.x or 0 --self.fsm.vars[1]
      local y = self.fsm.vars.y or 0 --self.fsm.vars[2]
      local ori = self.fsm.vars.ori or math.nan
      local m = ppnavi.CartesianGotoMessage:new(x, y, ori)
      printf("Sending CartesianGotoMessage(%f, %f, %f)", x, y, ori)
      self.fsm.vars.msgid = ppnavi:msgq_enqueue_copy(m)
   elseif self.fsm.vars.place ~= nil then
      -- place goto
      local place = self.fsm.vars.place
      if self.fsm.vars.ori ~= nil then
	 local ori = self.fsm.vars.ori
	 local m = ppnavi.PlaceWithOriGotoMessage:new(place, ori)
	 printf("Sending PlaceWithOriGotoMessage(%s, %f)", place, ori)
	 self.fsm.vars.msgid = ppnavi:msgq_enqueue_copy(m)
      else
	 local m = ppnavi.PlaceGotoMessage:new(place)
	 printf("Sending PlaceGotoMessage(%s)", place)
	 self.fsm.vars.msgid = ppnavi:msgq_enqueue_copy(m)
      end
   elseif self.fsm.vars.stop ~= nil then
      local m = ppnavi.StopMessage:new()
      printf("Sending StopGotoMessage")
      self.fsm.vars.msgid = ppnavi:msgq_enqueue_copy(m)
   else
      self.fsm.vars.param_fail = true
   end
   self.wait_start = 1
end

function PPGOTO:loop()
   self.wait_start = self.wait_start + 1
end

function PPGOTO:reset()
   if ppnavi:has_writer() and not ppnavi:is_final() then
      printf("ppgoto: sending stop");
      ppnavi:msgq_enqueue_copy(ppnavi.StopMessage:new())
   end
end



