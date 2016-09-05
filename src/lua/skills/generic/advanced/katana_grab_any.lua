
----------------------------------------------------------------------------
--  katana_grab_any.lua - Katana grabbing skill
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
name               = "katana_grab_any"
fsm                = SkillHSM:new{name=name, start="INIT", debug=true}
depends_skills     = {"katana", "katana_rel", "katana_approach", "or_object", "say"}
depends_interfaces = {
   {v = "katanaarm", id = "Katana", type = "KatanaInterface"}
}

documentation      = [==[Katana grabbing skill

This skill moves the katana to a given targe (object by name) and grabs it
CAUTION: Better make sure that object exists!

Possible call modes:

args:
 x=X, y=Y, z=Z          -> target coordinates

 new_object=OBJECT      -> add new object to openrave, name OBJECT
 object=OBJECT          -> use existing object of openrave, name OBJECT

 table_height=HEIGHT    -> table top height

 frame=FRAME            -> frame of target's coordinates

katana_grab{object=OBJECT, table_height=HEIGHT}
 Grabs object "OBJECT", which is on a table with height HEIGHT

]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- Constants
local MIN_APPROACH_OFFSET = 0.08 -- pre-grab position: 10cm before
local MAX_APPROACH_DIST = 0.07 -- approach: 5cm more than acutal target position (=> 15cm difference to pre-grab pos)
local SLOW_DOWN_VELOCITY = 0.3
local DEF_GRAB_THETA = math.pi/2 -- such that it can grab cylindrical objects
local MAX_GRAB_THETA_ERROR = 0.5 -- ~17° +-
local MAX_GRAB_DISTANCE = 0.55

-- functions
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

   {"INIT",                 SkillJumpState, skills={{katana}},
                            final_to="DECIDE", fail_to="FAILED"}, --final: gripper open
   {"DECIDE",               JumpState},
   {"ADD_OBJECT",           SkillJumpState, skills={{or_object}},
                            final_to="MOVE_OBJECT", fail_to="MOVE_OBJECT"}, --final: object added
   {"MOVE_OBJECT",          SkillJumpState, skills={{or_object}},
                            final_to="PRE_GRAB_POS", fail_to="FAILED"}, --final: object moved
   {"PRE_GRAB_POS",         SkillJumpState, skills={{katana}},
                            final_to="TO_APPROACH_OBJ", fail_to="FAILED_PRE_GRAB_POS"}, --final: ready to approach
   {"FAILED_PRE_GRAB_POS",  SkillJumpState, skills={{say}},
                            final_to="FAILED", fail_to="FAILED"}, --final: target not in range
   {"TO_APPROACH_OBJ",      JumpState},
   {"REPOSITION_OBJ",       SkillJumpState, skills={{or_object}},
                            final_to="APPROACH_OBJ", fail_to="APPROACH_OBJ"}, --final: obj at safe distance
   {"APPROACH_OBJ",         SkillJumpState, skills={{katana_approach}},
                            final_to="CHECK_GRABABILITY", fail_to="APPROACH_AGAIN"}, --final: reached max approach distance
   {"APPROACH_AGAIN",       SkillJumpState, skills={{katana_rel}},
                            final_to="CHECK_GRABABILITY", fail_to="FAILED_APPROACH"}, --final: reached max approach distance
   {"FAILED_APPROACH",      SkillJumpState, skills={{say}},
                            final_to="FAILED", fail_to="FAILED"}, --final: unreachable
   {"CHECK_GRABABILITY",    JumpState},
   {"STOP_MOVEMENT",        SkillJumpState, skills={{katana}},
                            final_to="TO_GRAB", fail_to="TO_GRAB"}, --final: stopped
   {"TO_GRAB",              JumpState},
   {"GRAB",                 SkillJumpState, skills={{katana}},
                            final_to="TO_ATTACH_OBJECT", fail_to="FAILED"}, --final: grabbed object
   {"TO_ATTACH_OBJECT",     SkillJumpState, skills={{or_object}},
                            final_to="ATTACH_OBJECT", fail_to="ATTACH_OBJECT"}, --final: obj is in gripper
   {"ATTACH_OBJECT",        SkillJumpState, skills={{or_object}},
                            final_to="FINAL", fail_to="FAILED"} --final: attached
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", cond_and_precond="not katanaarm:has_writer()", desc="no writer"},

   {"DECIDE", "ADD_OBJECT", cond="vars.x and vars.y and vars.z", desc="pose given"},
   {"DECIDE", "FAILED", cond=true, desc="insufficient arguments"},

   {"ADD_OBJECT", "PRE_GRAB_POS", cond_and_precond="not (vars.new_object and vars.table_height)", desc="no object given"},

   {"TO_APPROACH_OBJ", "REPOSITION_OBJ", timeout = 1.0},
  -- {"TO_APPROACH_OBJ", "APPROACH_OBJ", timeout = 1.0},
   --{"TO_APPROACH_OBJ", "FINAL", timeout = 1.0}, --TODO: skipp approach and stuff

   {"REPOSITION_OBJ", "APPROACH_OBJ", cond_and_precond="not (vars.object and vars.table_height)", desc="no object given"},

   --{"APPROACH_OBJ", "STOP_MOVEMENT", cond=jc_obj_is_grabable, desc="obj close enough"},
   --{"APPROACH_AGAIN", "STOP_MOVEMENT", cond=jc_obj_is_grabable, desc="obj close enough"},

   {"CHECK_GRABABILITY", "TO_GRAB", cond=jc_obj_is_grabable, desc="obj close enough"},
   --{"CHECK_GRABABILITY", "FAILED", cond=true, desc="obj out of range"},
   {"CHECK_GRABABILITY", "TO_GRAB", cond=true, desc="obj probably out of range"},

   {"TO_GRAB", "GRAB", timeout = 2.0},

   {"TO_ATTACH_OBJECT", "FINAL", cond_and_precond="not (vars.object and vars.table_height)", desc="no object given"},
}

function INIT:init()
   self.fsm.vars.target_theta_count = 0

   self.args[katana] = {gripper="open"}
end

function ADD_OBJECT:init()
   self.fsm.vars.object = self.fsm.vars.new_object -- need this so that next states work properly

   self.args[or_object] = {add=true, name=self.fsm.vars.new_object, path="../fawkes/res/openrave/cylinder.kinbody.xml"}
end

function MOVE_OBJECT:init()
   self.args[or_object] = {move=true, name=self.fsm.vars.object, x=self.fsm.vars.x,
                                                      y=self.fsm.vars.y,
                                                      z=self.fsm.vars.table_height + 0.045 + 0.01 } -- TODO: check
end

function PRE_GRAB_POS:init()
   local theta = DEF_GRAB_THETA
   local frame = self.fsm.vars.frame or "/base_link"

   katanaarm:msgq_enqueue_copy(katanaarm.SetPlannerParamsMessage:new("", false))
   self.args[katana] = {x=self.fsm.vars.x,
                        y=self.fsm.vars.y,
                        z=self.fsm.vars.z,
                        theta=theta,
                        theta_error = MAX_GRAB_THETA_ERROR,
                        frame=frame,
                        offset= -MIN_APPROACH_OFFSET}
end

function FAILED_PRE_GRAB_POS:init()
   local text = "I can't grab it properly, it's too"
   local x,y = self.fsm.vars.x, self.fsm.vars.y
   local dist = math.sqrt(x*x + y*y)
   if dist >= MAX_GRAB_DISTANCE then
      text = text .. " far away."
   else
      text = text .. " close."
   end
   self.args[say] = {text=text, wait=true}
end

function REPOSITION_OBJ:init()
   self.args[or_object] = {move=true, name=self.fsm.vars.object, x=self.fsm.vars.x + 0.3,
                                                                 y=self.fsm.vars.y,
                                                                 z=self.fsm.vars.table_height + 0.045 + 0.01 } -- TODO: check
end


function APPROACH_OBJ:init()
   katanaarm:msgq_enqueue_copy(katanaarm.SetPlannerParamsMessage:new("default", true))
   self.args[katana_approach] = {dist = MIN_APPROACH_OFFSET + MAX_APPROACH_DIST,
                                 min = MIN_APPROACH_OFFSET,
                                 max = MAX_APPROACH_DIST, --goes this much further(!) than "dist"!
                                 orth_lower = 0.01,
                                 orth_upper = 0.01}

   --[[
   -- get direction vector of manipulator, set length to MAX_APPROACH_DIST, move relatively to there
   katanaarm:read()

   local vector_to_target = fawkes.HomVector:new(0, 0, 1)
   --vector_to_target:rotate_y(katanaarm:theta())
   vector_to_target:rotate_y(math.pi/2) --look straight forward
   vector_to_target:rotate_z(katanaarm:phi() - math.pi/2)
   vector_to_target:set_length(MIN_APPROACH_OFFSET + MAX_APPROACH_DIST)

   self.args = {x = vector_to_target:x(),
                y = vector_to_target:y(),
                z = 0,
                straight=true,
                theta_error = 0.5,
                theta=math.pi/2} --theta=0 -> preferably look forward
   --]]
end

function APPROACH_AGAIN:init()
   katanaarm:read()

   local vector_to_target = fawkes.tf.Vector3:new(MIN_APPROACH_OFFSET + MAX_APPROACH_DIST, 0, 0) --look straight forward
   vector_to_target = vector_to_target:rotate(fawkes.tf.Vector3:new(0,0,1), katanaarm:phi() - math.pi/2)

   katanaarm:msgq_enqueue_copy(katanaarm.SetPlannerParamsMessage:new("default", false))
   self.args[katana_rel] = {x = vector_to_target:x(),
                            y = vector_to_target:y(),
                            z = 0,
                            theta_error = 0.5,
                            theta=math.pi/2} --theta=0 -> preferably look forward
end

function FAILED_APPROACH:init()
   local text = "I can't reach it safley."
   self.args[say] = {text=text, wait=true}
end

function GRAB:init()
   self.args[katana] = {gripper="close"}
end

function STOP_MOVEMENT:init()
   self.args[katana] = {stop=true}
end

function TO_ATTACH_OBJECT:init()
   katanaarm:read()

   -- get current arm positions. Need to to that later with transforms!
   local kat_x = katanaarm:x()
   local kat_y = katanaarm:y()

   self.args[or_object] = {move=true, name=self.fsm.vars.object, x=kat_x,
                                                                 y=kat_y,
                                                                 z=self.fsm.vars.table_height + 0.045 + 0.01 } -- TODO: check
end

function ATTACH_OBJECT:init()
   self.args[or_object] = {attach=true, name=self.fsm.vars.object}
end
