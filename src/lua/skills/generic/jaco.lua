
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

module(..., skillenv.module_init)

function use_skill(skill_name, iface_name)
   local M = {}

   -- Initialize module
   skillenv.module_init(M)

   -- Crucial skill information
   M.name               = skill_name
   M.fsm                = SkillHSM:new{name=skill_name, start="INIT", debug=true}
   M.depends_skills     = nil
   M.depends_interfaces = {
      --{v = iface_name, type = "JacoInterface"},
      {v = "jacoarm", id = "JacoArm", type = "JacoInterface"},
      {v = "jacoarm_left", id = "JacoArm Left", type = "JacoInterface"},
      {v = "jacoarm_right", id = "JacoArm Right", type = "JacoInterface"}
   }

   M.documentation      = [==[Kinova Jaco arm skill.

   This skill is used to control a Kinova Jaco robot arm.

   ====================
   Creating skills:

   The file is structured as a skill-generator, meaning you can use it to
   create separate skills for multiple Jaco arms, whit this in "init.lu":

     local jaco_gen = require("skills.generic.jaco")
     --jaco_gen.use_skill("skill_name", "iface_to_use")
     jaco_gen.use_skill("jaco", "jacoarm")
     jaco_gen.use_skill("jaco_left", "jacoarm_left")
     jaco_gen.use_skill("jaco_right", "jacoarm_right")

   Interfaces are currently restricted to those above in "M.depends_interfaces".


   ====================
   Using the skill:
   (assume we created a skill named "jaco").

   jaco{mode="init" | "retract"}
      Brings the arm into the desired mode (similar to pressing the
         yellow button on the joystick).

   jaco{pos="home" | "retract" | "stop"}
      Brings the arm to predefined positions (with path-planning etc).
      "home" is the position at INIT mode,
      "retract" is the position at RETRACT mode,
      "stop" stops the movement.

   jaco{gripper="open" | "close" | F_ALL | {F1, F2, F3} }
      F_ALL: numeric value [0,..,60].
      {F1, F2, F3}: table with values [0,..,60].
      Moving the griper. Either use predefined modes, or set the joint
         values directly. You can use one value for all fingers (F_ALL),
         or set independent ones ({F1, F2, F3}).

   jaco{params=PARAMS}
      PARAMS: string.
      Set planner parameters string that can be added to OpenRAVE's
         BaseManipulation module. See OpenRAVE doc for deeper information.
         Use empty string to clear the custom parameters.

   jaco{x=X, y=Y, z=Z [,e1=E1] [,e2=E2] [,e3=E3] [,type="ang"]}
      X, Y, Z: positions in 3D (currently based on base-frame).
      E1, E2, E3: rotations of the hand (optional).
      This sends a GotoMessage to the jaco plugin, with the given target
         position. If a rotation value is not set, its default will be used.
      This can also be used to move joint angles. Simply consider the tuple
         (X,Y,Z,E1,E2,E3) as values for the 6 joints, and add the argument
         type="ang".


   ====================
   For backwards compatibility:

   In order to not break some existing skills, it is still possible to
   call the "jaco" skill (after it has been properly generated) with
   an additional parameter that defines which arm (left or right) to use:

   jaco{arm="left" | "right", ...}
      This explicitly tells the skill to use the left or right arm.

    Use this additional argument for every command you want to use. It
    was introduced when this skill was not a "skill generator" yet, but
    a single skill where we still wanted to be able to decide which arm
    to use. Now that we use this "generator" to have truly separate skills
    for each arm, this method should not be used anymore.

   ]==]

   -- Initialize as skill module
   skillenv.skill_module(M)

   -- Set interface to use --
   local iface = M[iface_name]

   --- Check if arm motion is final.
   -- @return true if motion is final, false otherwise
   local function jc_arm_is_final(state)
      iface:read()
      return state.fsm.vars.msgid == iface:msgid() and
             iface:is_final()
   end

   --- Check if kinova plugin skipped our message
   -- @return true if kinova plugin skipped our message, false otherwise
   local function jc_next_msg(state)
      iface:read()
      return  iface:msgid() > state.fsm.vars.msgid
   end

   --- Check if there was some error for our message
   -- @return true if there was no error, false otherwise
   local function jc_error_none(state)
      iface:read()
      return  iface:error_code() == iface.ERROR_NONE
   end

   --- Check if the used interface is withour writer.
   -- We use a dedicated method for this check, because we have multiple interfaces.
   -- This is a lot easier than considering each in the fsm, using closure etc.
   -- @return true if interface has no writer.
   local function jc_iface_no_writer(state)
      return not iface:has_writer()
   end

   -- States
   M.fsm:define_states{
      export_to=M,

      {"INIT", JumpState},
      {"READY", JumpState},
      {"MODE_READY", JumpState},
      {"MODE_RETRACT", JumpState},
      {"GOTO_HOME", JumpState},
      {"GOTO_RETRACT", JumpState},
      {"STOP", JumpState},
      {"GOTO", JumpState},
      {"GRIPPER", JumpState},
      {"PARAMS", JumpState},

      {"CHECK_FINAL", JumpState},
      {"CHECK_ERROR", JumpState}
   }

   -- Transitions
   M.fsm:add_transitions{
      {"INIT", "READY", cond=true, desc="initialized"},

      {"READY", "FAILED", precond=jc_iface_no_writer, desc="no writer"},
      {"READY", "MODE_READY", precond="vars.mode == 'init'", desc="initialize arm"},
      {"READY", "MODE_RETRACT", precond="vars.mode == 'retract'", desc="initialize arm"},
      {"READY", "GOTO_HOME", precond="vars.pos == 'home'", desc="goto home pos"},
      {"READY", "GOTO_RETRACT", precond="vars.pos == 'retract'", desc="goto retract pos"},
      {"READY", "STOP", precond="vars.pos == 'stop'", desc="stop"},
      {"READY", "GOTO", precond="vars.x ~= nil and vars.y ~= nil and vars.z ~= nil", desc="goto parms"},
      {"READY", "GRIPPER", precond="vars.gripper", desc="move gripper"},
      {"READY", "PARAMS", precond="vars.params", desc="set planner params"},
      {"READY", "FAILED", precond=true, desc="bad params"},

      {"GOTO_HOME", "GOTO", cond=true, desc="params set"},
      {"GOTO_RETRACT", "GOTO", cond=true, desc="params set"},

      {"GOTO", "FAILED", precond="vars.x == nil or vars.y == nil or vars.z == nil", desc="insufficient params"},
      {"GRIPPER", "FAILED", cond="self.error", desc="bad error!!"},

      {"MODE_READY", "CHECK_FINAL", cond=true, desc="msg sent"},
      {"MODE_RETRACT", "CHECK_FINAL", cond=true, desc="msg sent"},
      {"STOP", "CHECK_FINAL", cond=true, desc="msg sent"},
      {"GOTO", "CHECK_FINAL", cond=true, desc="msg sent"},
      {"GRIPPER", "CHECK_FINAL", cond=true, desc="msg sent"},
      {"PARAMS", "CHECK_FINAL", cond=true, desc="msg sent"},

      {"CHECK_FINAL", "FINAL", precond="vars.no_wait", desc="skip final checking"},
      {"CHECK_FINAL", "CHECK_ERROR", cond=jc_arm_is_final, desc="arm final"},
      {"CHECK_FINAL", "FAILED", cond=jc_next_msg, desc="next msg"},

      {"CHECK_ERROR", "FINAL", cond=jc_error_none, desc="no error"},
      {"CHECK_ERROR", "FAILED", cond=true, desc="have an error"}
   }

   function M.INIT.init(self)
      if self.fsm.vars.arm == nil then
         return
      elseif self.fsm.vars.arm == 'left' then
         --printf("use left arm")
         iface = M.jacoarm_left
      elseif self.fsm.vars.arm == 'right' then
          --printf("use right arm")
         iface = M.jacoarm_right
      end
   end

   function M.MODE_READY.init(self)
      local m = iface.CalibrateMessage:new()
      self.fsm.vars.msgid = iface:msgq_enqueue_copy(m)
   end

   function M.MODE_RETRACT.init(self)
      local m = iface.RetractMessage:new()
      self.fsm.vars.msgid = iface:msgq_enqueue_copy(m)
   end

   function M.GOTO_HOME.init(self)
      self.fsm.vars.x = 282.522400
      self.fsm.vars.y = 154.470856
      self.fsm.vars.z = 44.191490
      self.fsm.vars.e1 = 230.081223
      self.fsm.vars.e2 = 83.242500
      self.fsm.vars.e3 = 77.796173

      self.fsm.vars.type = "ang"
   end

   function M.GOTO_RETRACT.init(self)
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

   function M.GRIPPER.init(self)
      self.error = false
      local f1, f2, f3
      if type(self.fsm.vars.gripper) == "table" and
         self.fsm.vars.gripper.f1 ~= nil and
         self.fsm.vars.gripper.f2 ~= nil and
         self.fsm.vars.gripper.f3 ~= nil then
         f1 = self.fsm.vars.gripper.f1
         f2 = self.fsm.vars.gripper.f2
         f3 = self.fsm.vars.gripper.f3
      elseif type(self.fsm.vars.gripper) == "number" then
         f1 = self.fsm.vars.gripper
         f2 = self.fsm.vars.gripper
         f3 = self.fsm.vars.gripper
      elseif self.fsm.vars.gripper == "open" then
         f1, f2, f3 = 0.25, 0.25, 0.25
      elseif self.fsm.vars.gripper =="close" then
         f1, f2, f3 = 52.0, 52.0, 52.0
      else
        self.error = true
      end

      if not self.error then
         local m = iface.MoveGripperMessage:new(f1, f2, f3)
         self.fsm.vars.msgid = iface:msgq_enqueue_copy(m)
      end
   end

   function M.STOP.init(self)
      local m = iface.StopMessage:new()
      self.fsm.vars.msgid = iface:msgq_enqueue_copy(m)
   end

   function M.GOTO.init(self)
      local x, y, z = self.fsm.vars.x, self.fsm.vars.y, self.fsm.vars.z

      iface:read()
      local e1             = self.fsm.vars.e1      or math.pi/2 + math.atan2(y,x)
      local e2             = self.fsm.vars.e2      or math.pi/2
      local e3             = self.fsm.vars.e3      or 0
      --printf("goto: "..x.."  "..y.."  "..z.."  "..e1.."  "..e2.."  "..e3)

      local m
      if self.fsm.vars.type == "ang" then
         m = iface.AngularGotoMessage:new(x, y, z, e1, e2, e3)
      else
         m = iface.CartesianGotoMessage:new(x, y, z, e1, e2, e3)
      end
      self.fsm.vars.msgid = iface:msgq_enqueue_copy(m)
   end

   function M.PARAMS.init(self)
      local m = iface.SetPlannerParamsMessage:new( self.fsm.vars.params )
      self.fsm.vars.msgid = iface:msgq_enqueue_copy(m)
   end

   _G[skill_name] = M
   package.loaded[skill_name] = M

   skillenv.use_skill(skill_name)
end
