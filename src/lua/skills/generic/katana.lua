
----------------------------------------------------------------------------
--  katana.lua - Skill to control the Katana arm via the katana plugin
--
--  Created: Wed Jun 10 13:58:58 2009
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

-- Initialize module
module(..., skillenv.module_init)

-- Crucial skill information
name               = "katana"
fsm                = SkillHSM:new{name=name, start="DECIDE_MODE"}
depends_skills     = nil
depends_interfaces = {
   {v = "katanaarm", type = "KatanaInterface"}
}

documentation      = [==[Katana skill.
Possible call modes:

katana{calibrate=true}
Calibrate the arm.

katana{park=true}
Park the arm in a safe position. Warning, make sure that the motion to
the parking position is possible and safe.

katana{enable=true/false}
Enable or disable motors. Warning, make sure the arm is in a safe place
before turning off the motors.

katana{stop=true}
Immediately stop any running motion.

katana{gripper="open"/"close"}
Open or close the gripper.

katana{x=..., y=..., z=..., phi=..., theta=..., psi=...[, frame=...] [, rot_frame=...]}
Move robot to the specified position.
The (x,y,z) is the position of the tool attached to the arm, relative
to the arms base. The (phi, theta, psi) angles are the Euler angles of
the tool.
"frame" refers to the tf frame which is origin of the target coordinates..
This is usually the robot's coordinate system (frame), as values shared in interfaces
should be in that coordinate system. This parameter is optional.
"rot_frame" refers to the tf frame which is origin of the tool rotation.
This is an optional value and rarely set. Default is robot's coordinate system,
as it is the easiest to reproduce a rotation from there on.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

--- Check if arm motion is final.
-- @return true if motion is final, false otherwise
function jc_arm_is_final(state)
   return state.fsm.vars.msgid == katanaarm:msgid() and
          katanaarm:is_final()
end

--- Check if katana plugin skipped our message
-- @return true if katana plugin skipped our message, false otherwise
function jc_next_msg(state)
   return  katanaarm:msgid() > state.fsm.vars.msgid
end

--- Set planner parameters
-- @return msgid of sent SetPlannerParamsMessage
function set_planner_params(params, straight)
   local s = straight or false
   return katanaarm:msgq_enqueue_copy(katanaarm.SetPlannerParamsMessage:new(params, s))
end


-- States
fsm:define_states{
   export_to=_M,
   closure={katanaarm=katanaarm},
   {"DECIDE_MODE", JumpState},
   {"TURNONOFF",   JumpState},
   {"CALIBRATE",   JumpState},
   {"VELOCITY",    JumpState},
   {"GOTO",        JumpState},
   {"GOTO_OBJECT", JumpState},
   {"STOP",        JumpState},
   {"PARK",        JumpState},
   {"GRIPPER",     JumpState},
   {"MOVE",        JumpState},
   {"CHECKERR",    JumpState}
}
-- Transitions
fsm:add_transitions{
   {"DECIDE_MODE", "FAILED", cond="not katanaarm:has_writer()", precond_only=true, desc="no writer"},
   {"DECIDE_MODE", "TURNONOFF", cond="vars.enable ~= nil", precond_only=true, desc="enable parm"},
   {"DECIDE_MODE", "CALIBRATE", cond="vars.calibrate", precond_only=true, desc="calib parm"},
   {"DECIDE_MODE", "VELOCITY", cond="vars.velocity ~= nil", precond_only=true, desc="max velocity"},
   {"DECIDE_MODE", "GOTO", cond="vars.x ~= nil and vars.y ~= nil and vars.z ~= nil", precond_only=true,
    desc="goto parms"},
   {"DECIDE_MODE", "GOTO_OBJECT", cond="vars.object ~= nil", precond_only=true, desc="goto obj params"},
   {"DECIDE_MODE", "STOP", cond="vars.stop", precond_only=true},
   {"DECIDE_MODE", "PARK", cond="vars.park", precond_only=true},
   {"DECIDE_MODE", "GRIPPER", cond="vars.gripper", precond_only=true},
   {"DECIDE_MODE", "MOVE", cond="vars.move and vars.nr and (vars.enc or vars.angle)", precond_only=true},
   {"DECIDE_MODE", "FAILED", cond=true, precond_only=true, desc="No valid command"},

   {"TURNONOFF", "CHECKERR", cond=true},
   {"CALIBRATE", "CHECKERR", cond=jc_arm_is_final, desc="final"},
   {"CALIBRATE", "FAILED", cond=jc_next_msg, desc="next msg"},
   {"VELOCITY", "CHECKERR", cond=true},
   {"GOTO", "CHECKERR", cond=jc_arm_is_final, desc="final"},
   {"GOTO_OBJECT", "CHECKERR", cond=jc_arm_is_final, desc="final"},
   {"GOTO", "FAILED", cond=jc_next_msg, desc="next msg"},
   {"STOP", "CHECKERR", cond=true},
   {"GRIPPER", "CHECKERR", cond=jc_arm_is_final, desc="final"},
   {"GRIPPER", "FAILED", cond=jc_next_msg, desc="next msg"},
   {"PARK", "CHECKERR", cond=jc_arm_is_final, desc="final"},
   {"PARK", "FAILED", cond=jc_next_msg, desc="next msg"},
   {"MOVE", "CHECKERR", cond=jc_arm_is_final, desc="final"},
   {"MOVE", "FAILED", cond=jc_next_msg, desc="next msg"},

   {"CHECKERR", "FINAL", cond="katanaarm:error_code() == katanaarm.ERROR_NONE", desc="no error"},
   {"CHECKERR", "FAILED", cond=true, desc="error"},
}

function CALIBRATE:init()
   self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.CalibrateMessage:new())
end

function TURNONOFF:init()
   katanaarm:msgq_enqueue_copy(katanaarm.SetEnabledMessage:new(self.fsm.vars.enable))
end

function STOP:init()
   katanaarm:msgq_enqueue_copy(katanaarm.StopMessage:new())
end

function VELOCITY:init()
   katanaarm:msgq_enqueue_copy(katanaarm.SetMaxVelocityMessage:new(self.fsm.vars.velocity))
end

function PARK:init()
   self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.ParkMessage:new())
end

function GRIPPER:init()
   if self.fsm.vars.gripper == "open" then
      self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.OpenGripperMessage:new())
   else
      self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.CloseGripperMessage:new())
   end
end

function PLANNERPARAMS:init()
   self.fsm.vars.msgid = set_planner_params(self.fsm.vars.plannerparams, self.fsm.vars.straight)
end

function CHECKERR:init()
   local err = katanaarm:error_code()
   if err == katanaarm.ERROR_UNSPECIFIC then
      self.fsm:set_error("Unspecified error occured")
   elseif err == katanaarm.ERROR_CMD_START_FAILED then
      self.fsm:set_error("Failed to initiate command")
   elseif err == katanaarm.ERROR_NO_SOLUTION then
      self.fsm:set_error("No inverse kinematics solution found")
   elseif err == katanaarm.ERROR_COMMUNICATION then
      self.fsm:set_error("Error communicating with arm")
   elseif err == katanaarm.ERROR_MOTOR_CRASHED then
      self.fsm:set_error("Motor crashed")
   end
end

function GOTO:init()
   local x, y, z = self.fsm.vars.x, self.fsm.vars.y, self.fsm.vars.z
   --local phi, theta, psi = 0, 0, 0
   local phi         = self.fsm.vars.phi         or math.pi/2 + math.atan2(y,x)
   local theta       = self.fsm.vars.theta       or math.pi/2
   local psi         = self.fsm.vars.psi         or 0
   local theta_error = self.fsm.vars.theta_error or 0
   local offset      = self.fsm.vars.offset      or 0
   local straight    = self.fsm.vars.straight    or false
   local frame       = self.fsm.vars.frame       or "/base_link" -- default: values given in robot's coordinate system!
   local rot_frame   = self.fsm.vars.rot_frame   or "/base_link" -- default: values given in robot's coordinate system!

   -- check if distances are too high (means they are in libkni coordinate system)
   if math.abs(x) > 5 or
      math.abs(y) > 5 or
      math.abs(z) > 5 then

      local gm = katanaarm.LinearGotoKniMessage:new(x, y, z, phi, theta, psi)
      self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(gm)
   else
      local gm = katanaarm.LinearGotoMessage:new(theta_error, offset, straight, frame, rot_frame, x, y, z, phi, theta, psi)
      self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(gm)
   end
end

function GOTO_OBJECT:init()
   local rot_x = 0.0

   if self.fsm.vars.rot_x  ~= nil then rot_x   = self.fsm.vars.rot_x end

   local gm = katanaarm.ObjectGotoMessage:new(self.fsm.vars.object, rot_x)
   self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(gm)
end

function MOVE:init()
   if self.fsm.vars.enc then
      if self.fsm.vars.rel then
         self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.MoveMotorEncoderMessage:new(self.fsm.vars.nr, self.fsm.vars.enc))
      else
         self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.SetMotorEncoderMessage:new(self.fsm.vars.nr, self.fsm.vars.enc))
      end
   else
      if self.fsm.vars.rel then
         self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.MoveMotorAngleMessage:new(self.fsm.vars.nr, self.fsm.vars.angle))
      else
         self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(katanaarm.SetMotorAngleMessage:new(self.fsm.vars.nr, self.fsm.vars.angle))
      end
   end
end
