
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
depends_skills     = {"katana", "katana_rel", "or_object", "say"}
depends_interfaces = {
   {v = "katanaarm", type = "KatanaInterface"}
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
local MAX_GRAB_DISTANCE = 0.60

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

fsm:add_transitions {
   closure={p=p, katanaarm=katanaarm},

   {"INIT", "DECIDE", skill=katana, fail_to="FAILED", desc="gripper open"},
   {"INIT", "FAILED", "not katanaarm:has_writer()", desc="no writer", precond=true},

   {"DECIDE", "ADD_OBJECT", "vars.x and vars.y and vars.z", desc="pose given"},
   {"DECIDE", "FAILED", true, desc="insufficient arguments"},

   {"ADD_OBJECT", "MOVE_OBJECT", fail_to="MOVE_OBJECT", desc="object added", skill=or_object},
   {"ADD_OBJECT", "PRE_GRAB_POS", "not (vars.new_object and vars.table_height)", desc="no object given", precond=true},

   {"MOVE_OBJECT", "PRE_GRAB_POS", fail_to="FAILED", desc="object moved", skill=or_object},

   {"PRE_GRAB_POS", "TO_APPROACH_OBJ", skill=katana, fail_to="FAILED_PRE_GRAB_POS", desc="ready to approach"},

   {"FAILED_PRE_GRAB_POS", "FAILED", fail_to="FAILED", desc="target not in range", skill=say},

   {"TO_APPROACH_OBJ", "REPOSITION_OBJ", wait_sec = 1.0},

   {"REPOSITION_OBJ", "APPROACH_OBJ", fail_to="APPROACH_OBJ", desc="obj at safe distance", skill=or_object},
   {"REPOSITION_OBJ", "APPROACH_OBJ", "not (vars.object and vars.table_height)", desc="no object given", precond=true},

   {"APPROACH_OBJ", "CHECK_GRABABILITY", skill=katana_rel, fail_to="FAILED_APPROACH", desc="reached max approach distance"},
   {"APPROACH_OBJ", "STOP_MOVEMENT", jc_obj_is_grabable, desc="obj close enough"},

   {"FAILED_APPROACH", "FAILED", fail_to="FAILED", desc="unreachable", skill=say},

   {"STOP_MOVEMENT", "TO_GRAB", skill=katana, args={stop=true}, fail_to="TO_GRAB", desc="stopped"},
   {"CHECK_GRABABILITY", "TO_GRAB", jc_obj_is_grabable, desc="obj close enough"},
   --{"CHECK_GRABABILITY", "FAILED", true, desc="obj out of range"},
   {"CHECK_GRABABILITY", "TO_GRAB", true, desc="obj out of range"},

   {"TO_GRAB", "GRAB", wait_sec = 2.0},

   {"GRAB", "TO_ATTACH_OBJECT", skill=katana, fail_to="FAILED", desc="grabbed object"},

   {"TO_ATTACH_OBJECT", "ATTACH_OBJECT", fail_to="ATTACH_OBJECT", desc="put obj into gripper", skill=or_object},
   {"TO_ATTACH_OBJECT", "FINAL", "not (vars.object and vars.table_height)", desc="no object given", precond=true},

   {"ATTACH_OBJECT", "FINAL", fail_to="FAILED", skill=or_object, desc="attached"}
}

function INIT:init()
   self.fsm.vars.target_theta_count = 0

   self.args = {gripper="open"}
end

function ADD_OBJECT:init()
   self.fsm.vars.object = self.fsm.vars.new_object -- need this so that next states work properly

   self.args = {add=true, name=self.fsm.vars.new_object, path="../fawkes/res/openrave/cylinder.kinbody.xml"}
end

function MOVE_OBJECT:init()
   self.args = {move=true, name=self.fsm.vars.object, x=self.fsm.vars.x,
                                                      y=self.fsm.vars.y,
                                                      z=self.fsm.vars.table_height + 0.045 + 0.01 } -- TODO: check
end

function PRE_GRAB_POS:init()
   local theta = DEF_GRAB_THETA
   local frame = self.fsm.vars.frame or "/base_link"

   self.args = {x=self.fsm.vars.x,
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
   self.args = {text=text, wait=true}
end

function REPOSITION_OBJ:init()
   self.args = {move=true, name=self.fsm.vars.object, x=self.fsm.vars.x + 0.3,
                                                      y=self.fsm.vars.y,
                                                      z=self.fsm.vars.table_height + 0.045 + 0.01 } -- TODO: check
end


function APPROACH_OBJ:init()
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
                theta_error = 0.5,
                theta=math.pi/2} --theta=0 -> preferably look forward

end

function FAILED_APPROACH:init()
   local text = "I can't reach it safley."
   self.args = {text=text, wait=true}
end

function GRAB:init()
   self.args = {gripper="close"}
end

function TO_ATTACH_OBJECT:init()
   katanaarm:read()

   -- get current arm positions. Need to to that later with transforms!
   local kat_x = katanaarm:x()
   local kat_y = katanaarm:y()

   self.args = {move=true, name=self.fsm.vars.object, x=kat_x,
                                                      y=kat_y,
                                                      z=self.fsm.vars.table_height + 0.045 + 0.01 } -- TODO: check
end

function ATTACH_OBJECT:init()
   self.args = {attach=true, name=self.fsm.vars.object}
end