
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

katana{x=..., y=..., z=..., phi=..., theta=..., psi=...}
Move robot to the specified position.
The (x,y,z) is the position of the tool attached to the arm, relative
to the arms base. The (phi, theta, psi) angles are the Euler angles of
the tool.
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

-- States
fsm:add_transitions{
   closure={katanaarm=katanaarm},
   {"DECIDE_MODE", "FAILED", "not katanaarm:has_writer()", precond=true, desc="no writer"},
   {"DECIDE_MODE", "TURNONOFF", "vars.enable ~= nil", precond=true, desc="enable parm"},
   {"DECIDE_MODE", "CALIBRATE", "vars.calibrate", precond=true, desc="calib parm"},
   {"DECIDE_MODE", "VELOCITY", "vars.velocity ~= nil", desc="max velocity", precond=true},
   {"DECIDE_MODE", "GOTO", "vars.x ~= nil and vars.y ~= nil and vars.z ~= nil",
    desc="goto parms", precond=true},
   {"DECIDE_MODE", "STOP", "vars.stop", precond=true},
   {"DECIDE_MODE", "PARK", "vars.park", precond=true},
   {"DECIDE_MODE", "GRIPPER", "vars.gripper", precond=true},
   {"DECIDE_MODE", "FAILED", true, precond=true, desc="No valid command"},
   {"CALIBRATE", "CHECKERR", jc_arm_is_final, desc="final"},
   {"CALIBRATE", "FAILED", jc_next_msg, desc="next msg"},
   {"TURNONOFF", "CHECKERR", true},
   {"STOP", "CHECKERR", true},
   {"VELOCITY", "CHECKERR", true},
   {"GOTO", "CHECKERR", jc_arm_is_final, desc="final"},
   {"GOTO", "FAILED", jc_next_msg, desc="next msg"},
   {"GRIPPER", "CHECKERR", jc_arm_is_final, desc="final"},
   {"GRIPPER", "FAILED", jc_next_msg, desc="next msg"},
   {"PARK", "CHECKERR", jc_arm_is_final, desc="final"},
   {"PARK", "FAILED", jc_next_msg, desc="next msg"},
   {"CHECKERR", "FINAL", "katanaarm:error_code() == katanaarm.ERROR_NONE", desc="no error"},
   {"CHECKERR", "FAILED", "katanaarm:error_code() ~= katanaarm.ERROR_NONE", desc="error"},
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
   local phi, theta, psi = 0, 0, 0
   if self.fsm.vars.phi   ~= nil then phi   = self.fsm.vars.phi end
   if self.fsm.vars.theta ~= nil then theta = self.fsm.vars.theta end
   if self.fsm.vars.psi   ~= nil then psi   = self.fsm.vars.psi end
   local gm = katanaarm.LinearGotoMessage:new(x, y, z, phi, theta, psi)
   self.fsm.vars.msgid = katanaarm:msgq_enqueue_copy(gm)
end
