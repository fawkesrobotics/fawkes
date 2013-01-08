
----------------------------------------------------------------------------
--  katana_grab.lua - Katana grabbing skill
--
--  Created: Thu Mar 03 14:32:47 2011
--  Copyright  2011  Bahram Maleki-Fard
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
name               = "katana_grab"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"katana", "katana_rel", "or_object"}
depends_interfaces = {
   {v = "katanaarm", type = "KatanaInterface"}
}

documentation      = [==[Katana grabbing skill

This skill moves the katana to a given targe (object by name) and grabs it
CAUTION: Better make sure that object exists!

Possible call modes:

katana_grab{object=OBJECT}
 Grabs object "OBJECT"
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Constants
local MAX_APPROACH_DIST = 0.1 --0.1 -- 10cm
local SLOW_DOWN_VELOCITY = 0.3

-- Jumpconditions
function jc_obj_is_grabable(state)
   --TODO: check sensor values, but which ones and what values?
   katanaarm:read()


   --[[ sensor values
     pos:       id:     def:    fire    fire
                                transl. dark
     ---------------------------------------------
     il          9      24-30   ~50     nA
     il2         8      24-30   ~50     ~120
     ce         11      0-6     ~50     ~120
     ir2         0      ~50     ~50     ~120
     ir          1      24-30   ~50     nA

   --]]
   local il, il2 = katanaarm:sensor_value(9), katanaarm:sensor_value(8)
   local ir, ir2 = katanaarm:sensor_value(1), katanaarm:sensor_value(0)
   local ce      = katanaarm:sensor_value(11)

   local ce_close = ce > 60           -- central sensor
   local i2_close = (il2 + ir2) > 100 -- inner sensors
   local i1_close = (il2 + ir2) > 100 -- inner sensors, fingertip, better not use

   --printf("ce: "..ce)
   --printf("i2: "..il2+ir2)
   --return ce_close or i2_close
   return ce > 120
end

-- States
fsm:define_states{
   export_to=_M,
   closure={katanaarm=katanaarm},

   {"INIT",     SkillJumpState, skills={{katana}},
                final_to="DECIDE", fail_to="FAILED"},
   {"DECIDE",   JumpState},

   {"PRE_GRAB_POS", SkillJumpState, skills={{katana}},
                final_to="START_APPROACH_OBJ", fail_to="FAILED"},
   {"PRE_GRAB_OBJ", SkillJumpState, skills={{katana}},
                final_to="START_APPROACH_OBJ", fail_to="FAILED"},
   {"START_APPROACH_OBJ", SkillJumpState, skills={{or_object}},
                final_to="SLOW_DOWN", fail_to="SLOW_DOWN"},
   {"SLOW_DOWN", SkillJumpState, skills={{katana}},
                final_to="TO_APPROACH_OBJ", fail_to="APPROACH_OBJ"},
   {"APPROACH_OBJ", SkillJumpState, skills={{katana_rel}},
                final_to="CHECK_GRABABILITY", fail_to="FAILED"},
   {"STOP_MOVEMENT", SkillJumpState, skills={{katana}},
                final_to="TO_GRAB", fail_to="FAILED"},
   {"GRAB", SkillJumpState, skills={{katana}},
                final_to="ATTACH_OBJECT", fail_to="FAILED"},
   {"ATTACH_OBJECT", SkillJumpState, skills={{or_object}},
                final_to="FINAL", fail_to="FAILED"},

   {"TO_APPROACH_OBJ", JumpState},
   {"CHECK_GRABABILITY", JumpState},
   {"TO_GRAB", JumpState},
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", cond="not katanaarm:has_writer()", desc="no writer", precond_only=true},

   {"DECIDE", "PRE_GRAB_OBJ", cond="vars.object", desc="object given"},
   {"DECIDE", "PRE_GRAB_POS", cond="vars.x and vars.y and vars.z", desc="pose given"},
   {"DECIDE", "FAILED", cond=true, desc="insufficient arguments"},

   --{"START_APPROACH_OBJ", "CHECK_GRABABILITY", cond=true, precond_only=true}, --TODO: skip approach for now

   {"SLOW_DOWN", "TO_APPROACH_OBJ", cond=true, precond_only=true}, --TODO: max_velocity crashes, so skip for now

   {"TO_APPROACH_OBJ", "APPROACH_OBJ", timeout=2.0},

   {"APPROACH_OBJ", "STOP_MOVEMENT", cond=jc_obj_is_grabable, desc="obj close enough"},

   {"CHECK_GRABABILITY", "TO_GRAB", cond=jc_obj_is_grabable, desc="obj close enough"},
   --{"CHECK_GRABABILITY", "FAILED", cond=true, desc="obj out of range"},
   {"CHECK_GRABABILITY", "TO_GRAB", cond=true, desc="obj out of range"},

   {"TO_GRAB", "GRAB", timeout=2.0},

   {"ATTACH_OBJECT", "FINAL", cond="not vars.object", desc="no object given", precond_only=true}
}

function INIT:init()
   self.args = {gripper="open"}
end

function PRE_GRAB_POS:init()
   self.args = {x=self.fsm.vars.x,
                y=self.fsm.vars.y,
                z=self.fsm.vars.z}
end

function PRE_GRAB_OBJ:init()
   self.args = {object=self.fsm.vars.object}
end

function START_APPROACH_OBJ:init()
   self.args = {attach=true, name=self.fsm.vars.object}
end

function SLOW_DOWN:init()
   self.args = {velocity=SLOW_DOWN_VELOCITY}
end

function APPROACH_OBJ:init()
   -- get direction vector of manipulator, set length to MAX_APPROACH_DIST, move relatively to there
   katanaarm:read()

   local vector_to_target = fawkes.HomVector:new(0, 0, 1)
   vector_to_target:rotate_y(katanaarm:theta())
   vector_to_target:rotate_z(katanaarm:phi() - math.pi/2)
   vector_to_target:set_length(MAX_APPROACH_DIST)

   self.args = {x = vector_to_target:x(),
                y = vector_to_target:y(),
                z = vector_to_target:z()}

end

function GRAB:init()
   self.args = {gripper="close"}
end

function ATTACH_OBJECT:init()
   self.args = {attach=true, name=self.fsm.vars.object}
end
