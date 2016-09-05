
----------------------------------------------------------------------------
--  jaco_bimanual.lua - Skill for coordinated bimanual manipulation of Jaco arm via jaco plugin
--
--  Created: Tue Sep 30 02:08:22 2014
--  Copyright  2014  Bahram Maleki-Fard
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
name               = "jaco_bimanual"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = nil
depends_interfaces = {
   {v = "jaco_bi", id = "JacoArm Bimanual", type = "JacoBimanualInterface"}
}

documentation      = [==[
]==]

-- Initialize as skill module
skillenv.skill_module(...)

--- Check if arm motion is final.
-- @return true if motion is final, false otherwise
function jc_arm_is_final(state)
   jaco_bi:read()
   return state.fsm.vars.msgid == jaco_bi:msgid() and
          jaco_bi:is_final()
end

--- Check if jaco plugin skipped our message
-- @return true if jaco plugin skipped our message, false otherwise
function jc_next_msg(state)
   jaco_bi:read()
   return  jaco_bi:msgid() > state.fsm.vars.msgid
end

--- Check if there was some error for our message
-- @return true if there was no error, false otherwise
function jc_error_none(state)
   jaco_bi:read()
   return  jaco_bi:error_code() == jaco_bi.ERROR_NONE
end

--- Check if the used interface is withour writer.
-- We use a dedicated method for this check, because we have multiple interfaces.
-- This is a lot easier than considering each in the fsm, using closure etc.
-- @return true if interface has no writer.
function jc_iface_no_writer(state)
   return not jaco_bi:has_writer()
end

function jc_goto(state)
   for _,arm in ipairs(state.fsm.vars.arms) do
      print("checking arm")
      if type(arm)~="table" then
         return false
      end
      if arm.x == nil or arm.y == nil or arm.z == nil then
         return false
      end
   end
   return #state.fsm.vars.arms==2
end

function jc_gripper(state)
   for _,arm in ipairs(state.fsm.vars.arms) do
      if type(arm)~="table" or arm.gripper==nil then
         return false
      end
   end
   return #state.fsm.vars.arms==2
end

function jc_params(state)
   return type(state.fsm.vars.params)=="string"
end

function jc_constrain(state)
   return type(state.fsm.vars.constrain)=="boolean"
end

-- States
fsm:define_states{
   export_to=_M,

   {"INIT", JumpState},
   {"READY", JumpState},
   {"GOTO", JumpState},
   {"GRIPPER", JumpState},
   {"PARAMS", JumpState},
   {"CONSTRAIN", JumpState},

   {"CHECK_FINAL", JumpState},
   {"CHECK_ERROR", JumpState}
}

-- Transitions
fsm:add_transitions{
   {"INIT", "READY", cond=true, desc="initialized"},

   {"READY", "FAILED", precond=jc_iface_no_writer, desc="no writer"},
   {"READY", "GOTO", precond=jc_goto, desc="move arms"},
   {"READY", "GRIPPER", precond=jc_gripper, desc="move gripper"},
   {"READY", "PARAMS", precond=jc_params, desc="set planner params"},
   {"READY", "CONSTRAIN", precond=jc_constrain, desc="set planning constraint"},
   {"READY", "FAILED", precond=true, desc="insufficient params"},

   {"GRIPPER", "FAILED", cond="self.error", desc="bad error!!"},

   {"GOTO", "CHECK_FINAL", cond=true, desc="msg sent"},
   {"GRIPPER", "CHECK_FINAL", cond=true, desc="msg sent"},
   {"PARAMS", "CHECK_FINAL", cond=true, desc="msg sent"},
   {"CONSTRAIN", "CHECK_FINAL", cond=true, desc="msg sent"},

   {"CHECK_FINAL", "CHECK_ERROR", cond=jc_arm_is_final, desc="arm final"},
   {"CHECK_FINAL", "FAILED", cond=jc_next_msg, desc="next msg"},

   {"CHECK_ERROR", "FINAL", cond=jc_error_none, desc="no error"},
   {"CHECK_ERROR", "FAILED", cond=true, desc="have an error"}
}

function INIT:init()
   self.fsm.vars.arms = {self.fsm.vars.left, self.fsm.vars.right}
end

function GRIPPER:init()
   self.error = false
   local f = {}

   for i,arm in ipairs(self.fsm.vars.arms) do
      f[i]={}
      if type(arm.gripper) == "table" and
         arm.gripper.f1 ~= nil and
         arm.gripper.f2 ~= nil and
         arm.gripper.f3 ~= nil then
         f[i].f1 = arm.gripper.f1
         f[i].f2 = arm.gripper.f2
         f[i].f3 = arm.gripper.f3
      elseif type(arm.gripper) == "number" then
         f[i].f1 = arm.gripper
         f[i].f2 = arm.gripper
         f[i].f3 = arm.gripper
      elseif arm.gripper == "open" then
         f[i].f1, f[i].f2, f[i].f3 = 0.25, 0.25, 0.25
      elseif arm.gripper =="close" then
         f[i].f1, f[i].f2, f[i].f3 = 52.0, 52.0, 52.0
      else
        self.error = true
      end
   end

   if not self.error then
      local m = jaco_bi.MoveGripperMessage:new(f[1].f1, f[1].f2, f[1].f3,
                                               f[2].f1, f[2].f2, f[2].f3)
      self.fsm.vars.msgid = jaco_bi:msgq_enqueue_copy(m)
   end
end


function GOTO:init()
   local pos = {}
   jaco_bi:read()

   for i,arm in ipairs(self.fsm.vars.arms) do
      local x, y, z = arm.x, arm.y, arm.z

      local e1             = arm.e1 or math.pi/2 + math.atan2(y,x)
      local e2             = arm.e2 or math.pi/2
      local e3             = arm.e3 or 0
      --printf("goto: "..x.."  "..y.."  "..z.."  "..e1.."  "..e2.."  "..e3)
      pos[i] = {x=x, y=y, z=z, e1=e1, e2=e2, e3=e3}
   end

   local m = jaco_bi.CartesianGotoMessage:new(pos[1].x, pos[1].y, pos[1].z, pos[1].e1, pos[1].e2, pos[1].e3,
                                              pos[2].x, pos[2].y, pos[2].z, pos[2].e1, pos[2].e2, pos[2].e3)
   self.fsm.vars.msgid = jaco_bi:msgq_enqueue_copy(m)
end

function PARAMS:init()
   local m = jaco_bi.SetPlannerParamsMessage:new( self.fsm.vars.params )
   self.fsm.vars.msgid = jaco_bi:msgq_enqueue_copy(m)
end

function CONSTRAIN:init()
   local m = jaco_bi.SetConstrainedMessage:new( self.fsm.vars.constrain )
   self.fsm.vars.msgid = jaco_bi:msgq_enqueue_copy(m)
end
