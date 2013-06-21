
----------------------------------------------------------------------------
--  jaco.lua - Skill to control the Kinova Jaco arm via the kinova plugin
--
--  Created: Thu Jun 20 14:38:51 2013
--  Copyright  2013  Bahram Maleki-Fard
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
name               = "jaco"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = nil
depends_interfaces = {
   {v = "jacoarm", type = "JacoInterface"}
}

documentation      = [==[
]==]

-- Initialize as skill module
skillenv.skill_module(...)


--- Check if arm motion is final.
-- @return true if motion is final, false otherwise
function jc_arm_is_final(state)
   return state.fsm.vars.msgid == jacoarm:msgid() and
          jacoarm:is_final()
end

--- Check if kinova plugin skipped our message
-- @return true if kinova plugin skipped our message, false otherwise
function jc_next_msg(state)
   return  jacoarm:msgid() > state.fsm.vars.msgid
end


-- States
fsm:define_states{
   export_to=_M,
   closure={jacoarm=jacoarm},

   {"INIT", JumpState},
   {"GOTO_HOME", JumpState},
   {"GOTO_RETRACT", JumpState},
   {"STOP", JumpState},
   {"GOTO", JumpState},
   {"GRIPPER", JumpState}
}

-- Transitions
fsm:add_transitions{
   {"INIT", "FAILED", precond_only="not jacoarm:has_writer()", desc="no writer"},
   {"INIT", "GOTO_HOME", precond_only="vars.pos == 'home'", desc="goto home pos"},
   {"INIT", "GOTO_RETRACT", precond_only="vars.pos == 'retract'", desc="goto retract pos"},
   {"INIT", "STOP", precond_only="vars.pos == 'stop'", desc="stop"},
   {"INIT", "GOTO", precond_only="vars.x ~= nil and vars.y ~= nil and vars.z ~= nil", desc="goto parms"},
   {"INIT", "GRIPPER", precond_only="vars.gripper == 'open' or vars.gripper == 'close'", desc="move gripper"},


   {"GOTO_HOME", "GOTO", cond=true, desc="params set"},
   {"GOTO_RETRACT", "GOTO", cond=true, desc="params set"},

   {"GRIPPER", "FINAL", cond=jc_arm_is_final, desc="gripper moved"},
   {"GRIPPER", "FAILED", cond=jc_next_msg, desc="next msg"},

   {"STOP", "FINAL", cond=jc_arm_is_final, desc="stopped"},
   {"STOP", "FAILED", cond=jc_next_msg, desc="next msg"},

   {"GOTO", "FAILED", precond_only="vars.x == nil or vars.y == nil or vars.z == nil", desc="insufficient params"},
   {"GOTO", "FINAL", cond=jc_arm_is_final, desc="goto final"},
   {"GOTO", "FAILED", cond=jc_next_msg, desc="next msg"}
}

function GOTO_HOME:init()
   self.fsm.vars.x = 0.234982
   self.fsm.vars.y = 0.201018
   self.fsm.vars.z = 0.435909
   self.fsm.vars.e1 = -1.518357
   self.fsm.vars.e2 = 0.472125
   self.fsm.vars.e3 = -3.177326

   self.fsm.vars.type = "cart"
end

function GOTO_RETRACT:init()
--[[
   -- possible cartesian coordinates for RETRACT position
   self.fsm.vars.x = 0.136740
   self.fsm.vars.y = 0.021601
   self.fsm.vars.z = 0.317547
   self.fsm.vars.e1 = -0.443940
   self.fsm.vars.e2 = -0.012489
   self.fsm.vars.e3 = -0.302805
--]]

   self.fsm.vars.x = 270.527344
   self.fsm.vars.y = 150.205078
   self.fsm.vars.z = 25.042963
   self.fsm.vars.e1 = 267.451172
   self.fsm.vars.e2 = 5.800781
   self.fsm.vars.e3 = 99.448242

   self.fsm.vars.type = "ang"
end

function GRIPPER:init()
   if self.fsm.vars.gripper == "open" then
      local m = jacoarm.OpenGripperMessage:new()
      self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
   elseif self.fsm.vars.gripper =="close" then
      local m = jacoarm.CloseGripperMessage:new()
      self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
   end
end

function STOP:init()
   local m = jacoarm.StopMessage:new()
   self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
end

function GOTO:init()
   local x, y, z = self.fsm.vars.x, self.fsm.vars.y, self.fsm.vars.z

   jacoarm:read()
   local e1             = self.fsm.vars.e1      or jacoarm:euler1()
   local e2             = self.fsm.vars.e2      or jacoarm:euler2()
   local e3             = self.fsm.vars.e3      or jacoarm:euler3()
   --printf("goto: "..x.."  "..y.."  "..z.."  "..e1.."  "..e2.."  "..e3)

   local m
   if self.fsm.vars.type == "ang" then
      m = jacoarm.AngularGotoMessage:new(x, y, z, e1, e2, e3)
   else
      m = jacoarm.CartesianGotoMessage:new(x, y, z, e1, e2, e3)
   end
   self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
end