
----------------------------------------------------------------------------
--  pantilt.lua - Skill to control a pan/tilt unit via the pantilt plugin
--
--  Created: Thu Jun 18 18:12:22 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
--
--  $Id$
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
name               = "pantilt"
fsm                = SkillHSM:new{name=name, start="DECIDE_MODE"}
depends_skills     = nil
depends_interfaces = {
   {v = "ptu_RX28", id = "PanTilt RX28", type = "PanTiltInterface"},
   {v = "ptu_EviD100P", id = "PanTilt EviD100P", type = "PanTiltInterface"}
}

documentation      = [==[Pan/tilt skill.
Possible call modes:

pantilt{calibrate=true}
Calibrate the PTU.

pantilt{park=true}
Park the PTU in a safe position. Warning, make sure that the motion to
the parking position is possible and safe.

pantilt{enable=true/false}
Enable or disable motors. Warning, make sure the PTU is in a safe place
before turning off the motors.

pantilt{stop=true}
Immediately stop any running motion.

pantilt{pan=..., tilt=..., max_vel=...}
Move PTU to the specified position.
The angles are given in radians, pan=0 and tilt=0 is forward, pan is counter
clockwise positive, tilt is downwards positive.

All call styles support an optional argument ptu="PTU" to define the PTU in
question.
]==]

-- Initialize as skill module
skillenv.skill_module(...)

local ptus = { RX28 = ptu_RX28, EviD100P = ptu_EviD100P }
local default_ptu = "RX28"

--- Get selected PTU interface.
-- @param ptu_name name of the desired PTU, nil to select default
-- @return PTU blackboard interface
function ptu_interface(ptu_name)
   local rv = ptus[default_ptu]
   if ptu_name then rv = ptus[fsm.vars.ptu] end
   assert(rv, "No valid PTU chosen")
   return rv
end

--- Check if arm motion is final.
-- @return true if motion is final, false otherwise
function jc_ptu_is_final(state)
   local ptu = ptu_interface(state.fsm.vars.ptu)
   return state.fsm.vars.msgid == ptu:msgid() and ptu:is_final()
end

--- Check if pantilt plugin skipped our message
-- @return true if pantilt plugin skipped our message, false otherwise
function jc_next_msg(state)
   local ptu = ptu_interface(state.fsm.vars.ptu)
   return  ptu:msgid() > state.fsm.vars.msgid
end

-- States
fsm:add_transitions{
   closure={ptu=ptu_interface},
   {"DECIDE_MODE", "FAILED", "not ptu(vars.ptu):has_writer()", precond=true, desc="no writer"},
   {"DECIDE_MODE", "TURNONOFF", "vars.enable ~= nil", precond=true, desc="enable parm"},
   {"DECIDE_MODE", "CALIBRATE", "vars.calibrate", precond=true, desc="calib parm"},
   {"DECIDE_MODE", "GOTO", "vars.pan ~= nil and vars.tilt ~= nil",
    desc="goto parms", precond=true},
   {"DECIDE_MODE", "STOP", "vars.stop", precond=true},
   {"DECIDE_MODE", "PARK", "vars.park", precond=true},
   {"DECIDE_MODE", "SPEED", "vars.max_speed", precond=true},
   {"DECIDE_MODE", "FAILED", true, precond=true, desc="No valid command"},
   {"CALIBRATE", "CHECKERR", jc_ptu_is_final, desc="final"},
   {"CALIBRATE", "FAILED", jc_next_msg, desc="next msg"},
   {"TURNONOFF", "CHECKERR", true},
   {"STOP", "CHECKERR", true},
   {"SPEED", "CHECKERR", true},
   {"GOTO", "CHECKERR", jc_ptu_is_final, desc="final"},
   {"GOTO", "FAILED", jc_next_msg, desc="next msg"},
   {"PARK", "CHECKERR", jc_ptu_is_final, desc="final"},
   {"PARK", "FAILED", jc_next_msg, desc="next msg"},
   {"CHECKERR", "FINAL", "ptu(vars.ptu):error_code() == ptu(vars.ptu).ERROR_NONE", desc="no error"},
   {"CHECKERR", "FAILED", "ptu(vars.ptu):error_code() ~= ptu(vars.ptu).ERROR_NONE", desc="error"},
}

function CALIBRATE:init()
   local ptu = ptu_interface(self.fsm.vars.ptu)
   self.fsm.vars.msgid = ptu:msgq_enqueue_copy(ptu.CalibrateMessage:new())
end

function TURNONOFF:init()
   local ptu = ptu_interface(self.fsm.vars.ptu)
   ptu:msgq_enqueue_copy(ptu.SetEnabledMessage:new(self.fsm.vars.enable))
end

function STOP:init()
   local ptu = ptu_interface(self.fsm.vars.ptu)
   ptu:msgq_enqueue_copy(ptu.StopMessage:new())
end

function PARK:init()
   send_max_speed()
   local ptu = ptu_interface(self.fsm.vars.ptu)
   self.fsm.vars.msgid = ptu:msgq_enqueue_copy(ptu.ParkMessage:new())
end

function SPEED:init()
   send_max_speed()
end

function send_max_speed()
   if fsm.vars.max_speed ~= nil then
      local ptu = ptu_interface(fsm.vars.ptu)
      if fsm.vars.max_speed == "MAX" then
	 fsm.vars.max_speed = math.min(ptu:max_pan_velocity(), ptu:max_tilt_velocity())
      end
      local vm = ptu.SetVelocityMessage:new(fsm.vars.max_speed, fsm.vars.max_speed)
      ptu:msgq_enqueue_copy(vm)
   end
end

function CHECKERR:init()
   local ptu = ptu_interface(self.fsm.vars.ptu)
   local err = ptu:error_code()
   if err == ptu.ERROR_UNSPECIFIC then
      self.fsm:set_error("Unspecified error occured")
   elseif err == ptu.ERROR_COMMUNICATION then
      self.fsm:set_error("Error communicating with PTU")
   elseif err == ptu.ERROR_PAN_OUTOFRANGE then
      self.fsm:set_error("Pan is out of range")
   elseif err == ptu.ERROR_TILT_OUTOFRANGE then
      self.fsm:set_error("Tilt is out of range")
   end
end

function GOTO:init()
   local pan, tilt = self.fsm.vars.pan, self.fsm.vars.tilt
   local ptu = ptu_interface(self.fsm.vars.ptu)
   send_max_speed()
   local gm = ptu.GotoMessage:new(pan, tilt)
   self.fsm.vars.msgid = ptu:msgq_enqueue_copy(gm)
end
