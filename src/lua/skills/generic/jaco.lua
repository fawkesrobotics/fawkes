
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
   jacoarm:read()
   return state.fsm.vars.msgid == jacoarm:msgid() and
          jacoarm:is_final()
end

--- Check if kinova plugin skipped our message
-- @return true if kinova plugin skipped our message, false otherwise
function jc_next_msg(state)
   jacoarm:read()
   return  jacoarm:msgid() > state.fsm.vars.msgid
end


-- States
fsm:define_states{
   export_to=_M,
   closure={jacoarm=jacoarm},

   {"INIT", JumpState},
   {"MODE_READY", JumpState},
   {"MODE_RETRACT", JumpState},
   {"GOTO_HOME", JumpState},
   {"GOTO_RETRACT", JumpState},
   {"STOP", JumpState},
   {"GOTO", JumpState},
   {"GRIPPER", JumpState}
}

-- Transitions
fsm:add_transitions{
   {"INIT", "FAILED", precond_only="not jacoarm:has_writer()", desc="no writer"},
   {"INIT", "MODE_READY", precond_only="vars.mode == 'init'", desc="initialize arm"},
   {"INIT", "MODE_RETRACT", precond_only="vars.mode == 'retract'", desc="initialize arm"},
   {"INIT", "GOTO_HOME", precond_only="vars.pos == 'home'", desc="goto home pos"},
   {"INIT", "GOTO_RETRACT", precond_only="vars.pos == 'retract'", desc="goto retract pos"},
   {"INIT", "STOP", precond_only="vars.pos == 'stop'", desc="stop"},
   {"INIT", "GOTO", precond_only="vars.x ~= nil and vars.y ~= nil and vars.z ~= nil", desc="goto parms"},
   {"INIT", "GRIPPER", precond_only="vars.gripper == 'open' or vars.gripper == 'close'", desc="move gripper"},

   {"MODE_READY", "FINAL", cond=jc_arm_is_final, desc="gripper moved"},
   {"MODE_READY", "FAILED", cond=jc_next_msg, desc="next msg"},

   {"MODE_RETRACT", "FINAL", cond=jc_arm_is_final, desc="gripper moved"},
   {"MODE_RETRACT", "FAILED", cond=jc_next_msg, desc="next msg"},

   {"GOTO_HOME", "GOTO", cond=true, desc="params set"},
   {"GOTO_RETRACT", "GOTO", cond=true, desc="params set"},

   {"GRIPPER", "FINAL", cond=jc_arm_is_final, desc="gripper moved"},
   {"GRIPPER", "FAILED", cond=jc_next_msg, desc="next msg"},
   {"GRIPPER", "FAILED", cond="self.error", desc="bad error!!"},

   {"STOP", "FINAL", cond=jc_arm_is_final, desc="stopped"},
   {"STOP", "FAILED", cond=jc_next_msg, desc="next msg"},

   {"GOTO", "FAILED", precond_only="vars.x == nil or vars.y == nil or vars.z == nil", desc="insufficient params"},
   {"GOTO", "FINAL", cond=jc_arm_is_final, desc="goto final"},
   {"GOTO", "FAILED", cond=jc_next_msg, desc="next msg"}
}
function MODE_READY:init()
   local m = jacoarm.CalibrateMessage:new()
   self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
end

function MODE_RETRACT:init()
   local m = jacoarm.RetractMessage:new()
   self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
end

function GOTO_HOME:init()
   self.fsm.vars.x = 282.522400
   self.fsm.vars.y = 154.470856
   self.fsm.vars.z = 44.191490
   self.fsm.vars.e1 = 230.081223
   self.fsm.vars.e2 = 83.242500
   self.fsm.vars.e3 = 77.796173

   self.fsm.vars.type = "ang"
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
   self.error = false
   if self.fsm.vars.gripper == "open" then
      local m = jacoarm.OpenGripperMessage:new()
      self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
   elseif self.fsm.vars.gripper =="close" then
      local m = jacoarm.CloseGripperMessage:new()
      self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
   else
     self.error = true
   end
end

function STOP:init()
   local m = jacoarm.StopMessage:new()
   self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
end

function GOTO:init()
   local x, y, z = self.fsm.vars.x, self.fsm.vars.y, self.fsm.vars.z

   jacoarm:read()
   local e1             = self.fsm.vars.e1      or math.pi/2 + math.atan2(y,x)
   local e2             = self.fsm.vars.e2      or math.pi/2
   local e3             = self.fsm.vars.e3      or 0
   --printf("goto: "..x.."  "..y.."  "..z.."  "..e1.."  "..e2.."  "..e3)

   local m
   if self.fsm.vars.type == "ang" then
      m = jacoarm.AngularGotoMessage:new(x, y, z, e1, e2, e3)
   else
      m = jacoarm.CartesianGotoMessage:new(x, y, z, e1, e2, e3)
   end
   self.fsm.vars.msgid = jacoarm:msgq_enqueue_copy(m)
end