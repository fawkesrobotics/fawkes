
----------------------------------------------------------------------------
--  servo.lua - Servo control skill for the Nao
--
--  Created: Fri Aug 15 11:16:50 2008
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
name               = "servo"
fsm                = SkillHSM:new{name=name, start="SERVO"}
depends_skills     = nil
depends_interfaces = {
   {v = "naojoints", type = "NaoJointPositionInterface", id = "Nao Joint Positions"},
   {v = "naostiffness", type = "NaoJointStiffnessInterface", id = "Nao Joint Stiffness"},
   {v = "naomotion", type = "HumanoidMotionInterface", id = "NaoQi Motion"}
}

documentation = [==[Servo control for the Nao.

This skill allows for control of the Nao servos. It is most useful to
enable and disable the servos by using the following form:
servo{enable=true} Set enable to true to enable servos, to false to
disable them. Attention, the robot may fall when you disable the
servos, as this removes the hardness from the servos!
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Constants
local DEFAULT_TOLERANCE = 0.25
local DEFAULT_TIME_SEC  = 0.5

-- States
SERVO = WaitState:new{name="SERVO", fsm=fsm, next_state="PRINTERR"}
fsm:add_state(SERVO)
--fsm:new_jump_state("SERVO")

fsm:new_jump_state("PRINTERR")

function PRINTERR:init()
   for k,v in pairs(self.fsm.vars.goto_wait) do
      if math.abs(naojoints[k](naojoints) - v) > self.fsm.vars.tolerance then
	 self.fsm.error = string.format("%s servo timeout. actual: %f  expected: %f  "..
					"diff: %f  tolerance: %f", k,
				        naojoints[k](naojoints), v,
					math.abs(naojoints[k](naojoints) - v),
					self.fsm.vars.tolerance)
	 print_error(self.fsm.error)
      end
   end
end
PRINTERR:add_transition("FAILED", true, "Error printed")

function SERVO:maybe_enqueue_goto(servo, value)
   if value ~= nil then
      self.fsm.vars.goto_wait[servo] = value
      local m = naojoints.GotoAngleMessage:new(naojoints["SERVO_"..tostring(servo)], value,
					   self.fsm.vars.time_sec)
      naojoints:msgq_enqueue_copy(m)
   end
end

function SERVO:init()
   self.fsm.vars.goto_wait = {}
   self.fsm.vars.tolerance = tonumber(self.fsm.vars.tolerance) or DEFAULT_TOLERANCE
   self.fsm.vars.time_sec  = tonumber(self.fsm.vars.time_sec) or DEFAULT_TIME_SEC

   self.time_sec = 2 * self.fsm.vars.time_sec

   if self.fsm.vars.enable ~= nil then
      if self.fsm.vars.enable then
         naostiffness:msgq_enqueue_copy(naostiffness.SetBodyStiffnessMessage:new(1.0, 0.5))
      else
	 naostiffness:msgq_enqueue_copy(naostiffness.SetBodyStiffnessMessage:new(0, 0.5))
      end
   end

   if self.fsm.vars.body_angles then
      local m = naojoints.GotoAnglesMessage:new(self.fsm.vars.time_sec,
					    naojoints.INTERPOLATION_SMOOTH,
					    self.fsm.vars.body_angles.head_yaw or 0.0,
					    self.fsm.vars.body_angles.head_pitch or 0.0,
					    self.fsm.vars.body_angles.l_shoulder_pitch or 0.0,
					    self.fsm.vars.body_angles.l_shoulder_roll or 0.0,
					    self.fsm.vars.body_angles.l_elbow_yaw or 0.0,
					    self.fsm.vars.body_angles.l_elbow_roll or 0.0,
					    self.fsm.vars.body_angles.l_hip_yaw_pitch or 0.0,
					    self.fsm.vars.body_angles.l_hip_roll or 0.0,
					    self.fsm.vars.body_angles.l_hip_pitch or 0.0,
					    self.fsm.vars.body_angles.l_knee_pitch or 0.0,
					    self.fsm.vars.body_angles.l_ankle_pitch or 0.0,
					    self.fsm.vars.body_angles.l_ankle_roll or 0.0,
					    self.fsm.vars.body_angles.r_hip_yaw_pitch or 0.0,
					    self.fsm.vars.body_angles.r_hip_roll or 0.0,
					    self.fsm.vars.body_angles.r_hip_pitch or 0.0,
					    self.fsm.vars.body_angles.r_knee_pitch or 0.0,
					    self.fsm.vars.body_angles.r_ankle_pitch or 0.0,
					    self.fsm.vars.body_angles.r_ankle_roll or 0.0,
					    self.fsm.vars.body_angles.r_shoulder_pitch or 0.0,
					    self.fsm.vars.body_angles.r_shoulder_roll or 0.0,
					    self.fsm.vars.body_angles.r_elbow_yaw or 0.0,
					    self.fsm.vars.body_angles.r_elbow_roll or 0.0
					   )
      for k,v in pairs(self.fsm.vars.body_angles) do
	 self.fsm.vars.goto_wait[k] = v or 0.0
      end
      naojoints:msgq_enqueue_copy(m)
   else if self.fsm.vars.yaw ~= nil and self.fsm.vars.pitch ~= nil then
     local m = naomotion.YawPitchHeadMessage:new(
                  self.fsm.vars.yaw,
                  self.fsm.vars.pitch,
                  self.fsm.vars.time_sec)
     naomotion:msgq_enqueue_copy(m)
   else
      for _,s in ipairs({"head_yaw", "head_pitch",
			 "l_shoulder_pitch", "l_shoulder_roll",
			 "l_elbow_yaw", "l_elbow_roll",
			 "l_hip_yaw_pitch", "l_hip_roll", "l_hip_pitch",
			 "l_knee_pitch", "l_ankle_pitch", "l_ankle_roll",
			 "r_hip_yaw_pitch", "r_hip_roll", "r_hip_pitch",
			 "r_knee_pitch", "r_ankle_pitch", "r_ankle_roll",
			 "r_shoulder_pitch", "r_shoulder_roll",
			 "r_elbow_yaw", "r_elbow_roll"}) do
	 self:maybe_enqueue_goto(s, self.fsm.vars[s])
      end
   end
   end
end


-- check if the goto is finished
function SERVO:jumpcond_done()
   local finished = true
   for k,v in pairs(self.fsm.vars.goto_wait) do
      -- the following effectively calls the get method for the given servo
      if math.abs(naojoints[k](naojoints) - v) > self.fsm.vars.tolerance then
	 finished = false
	 break
      end
   end
   return finished
end

SERVO.nowriter_interfaces = {naojoints,naostiffness,naomotion}

SERVO:add_precond_trans(FAILED, JumpState.jumpcond_nowriter, "No writer for naojoints interface")
SERVO:add_transition(FINAL, SERVO.jumpcond_done, "Servo movements done")
