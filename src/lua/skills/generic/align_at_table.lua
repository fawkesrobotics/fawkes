
----------------------------------------------------------------------------
--  align_at_table.lua - Align at a table
--
--  Created: Wed Apr 02 17:21:12 2014
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
name               = "align_at_table"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {"relgoto", "pantilt"}
depends_interfaces = {
   {v = "table_pos", type = "Position3DInterface", id = "Tabletop"},
   {v = "table_detect", type = "SwitchInterface", id = "tabletop-objects"},
   {v = "ptu_RX28", type="PanTiltInterface", id="PanTilt RX28"},
   {v = "navigator", type="NavigatorInterface", id="Navigator"}
}

local tfutils = require("fawkes.tfutils")

documentation      = [==[Align at table skill
This skill brings the robot into position in front of a table. The position
is meant to be "good" for grasping tasks.

The steps are: look down, detect table, adjust orientation, adjust distance
to table.
pantilt-position, table-detection, drive-modes etc. are reset when skill is
finished.

Possible call modes:
align_at_table{}
align_at_table{dist=DIST}
  where DIST ist the distance to keep to table (in meters), default is 0.2
]==]

-- Initialize as skill module
skillenv.skill_module(_M)

-- Constants
MIN_VISIBILITY_HISTORY = 5

TABLE_WIDTH = config:get_float("/perception/tabletop-detection/table_model_width") or 0.7

DIST_TO_FRONT = 0.12 -- distance from base_link to robot front
DIST_TABLE = 0.15 -- hold this distance to the table front

TABLE_POSITION_DISTANCE = 0.2 -- hold a distance of 0.2m to table

TIMEOUT = {DETECT_TABLE = 8.0,
           WAIT_FOR_STOPPING = 1.5}

FRAMES = {TARGET = config:get_string("/frames/base")}

YAW_LIMIT = math.pi/4
MIN_DIST = 0.1

-- position for PTU to be looking down at a table
PTU = {PAN = 0.0,
       TILT = 0.8,
       SPEED = 0.5}

function can_transform(source_frame)
   return tf:can_transform(FRAMES.TARGET, source_frame, fawkes.Time:new(0,0))
end

function transform_point(source_frame, x, y, z)
   local p  = fawkes.tf.Point:new(x, y, z)
   local sp = fawkes.tf.StampedPoint:new(p, fawkes.Time:new(0,0), source_frame)
   local tp = fawkes.tf.StampedPoint:new()
   tf:transform_point(FRAMES.TARGET, sp, tp)

   return tp:x(), tp:y(), tp:z()
end

function transform_object(obj)
   return transform_point(obj:frame(),
                          obj:translation(0),
                          obj:translation(1),
                          obj:translation(2))
end

-- Jumpconditions
function table_detected(state)
   if table_pos:has_writer() then
      if table_pos:visibility_history() > MIN_VISIBILITY_HISTORY then
         local x, y, z = table_pos:translation(0), table_pos:translation(1), table_pos:translation(2)
         if can_transform(table_pos:frame()) then
            x, y, z = transform_object(table_pos)
         else
            print_warn("can't transform from '%s' to '%s'", table_pos:frame(), FRAMES.TARGET)
         end
         state.fsm.vars.table_distance = x - TABLE_WIDTH/2 -- distance to table front
         local quat = fawkes.tf.Quaternion:new(table_pos:rotation(0),
                                               table_pos:rotation(1),
                                               table_pos:rotation(2),
                                               table_pos:rotation(3))
         state.fsm.vars.table_yaw = fawkes.tf.get_yaw(quat)
         --printf("detected table. dist="..state.fsm.vars.table_distance.." , yaw="..state.fsm.vars.table_yaw)
         if math.abs(state.fsm.vars.table_yaw) <= YAW_LIMIT and
            x >= MIN_DIST then
            return true
         end
      end
   end

   state.fsm.vars.table_distance = 0.0
   return false
end

-- States
fsm:define_states{
   export_to=_M,
   closure={table_detect=table_detect, navigator=navigator},

   {"INIT",     JumpState},

   {"LOOK_AT_TABLE", SkillJumpState, skills={{"pantilt"}},
                  final_to="DETECT_TABLE", fail_to="FAILED"},

   {"DETECT_TABLE", JumpState},

   -- align with the detected table
   {"ALIGN_AT_TABLE", SkillJumpState, skills={{"relgoto"}},
                  final_to="WAIT_FOR_STOPPING", fail_to="WAIT_FOR_STOPPING"},

   -- short waiting state, to make sure movement has stopped
   {"WAIT_FOR_STOPPING", JumpState},

   {"DETECT_TABLE_AGAIN", JumpState},

   -- get correct distance to the table
   {"POSITION_AT_TABLE", SkillJumpState, skills={{"relgoto"}},
                  final_to="RESTORE_PTU", fail_to="FAILED"},

   {"RESTORE_PTU", SkillJumpState, skills={{"pantilt"}},
                  final_to="FINAL", fail_to="FAILED"}
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", precond="not (table_detect:has_writer() and navigator:has_writer())", desc="no writer"},
   {"INIT", "LOOK_AT_TABLE", cond=true, desc="initialized"},

   {"DETECT_TABLE", "FAILED", timeout=TIMEOUT.DETECT_TABLE, desc="no table detected"},
   {"DETECT_TABLE", "ALIGN_AT_TABLE", cond=table_detected, desc="align at table"},

   {"WAIT_FOR_STOPPING", "DETECT_TABLE_AGAIN", timeout=TIMEOUT.WAIT_FOR_STOPPING},

   {"DETECT_TABLE_AGAIN", "FAILED", timeout=TIMEOUT.DETECT_TABLE, desc="no table detected"},
   {"DETECT_TABLE_AGAIN", "POSITION_AT_TABLE", cond=table_detected, desc="position at table"},
}

function INIT:init()
   self.fsm.vars.dist = self.fsm.vars.dist or DIST_TABLE
end

function LOOK_AT_TABLE:init()
   -- for restoring, save current data from interfaces
   table_detect:read()
   self.fsm.vars.table_detect = table_detect:is_enabled()

   navigator:read()
   self.fsm.vars.navi = {escaping = navigator:is_escaping_enabled(),
                         drive_mode = navigator:drive_mode()}

   ptu_RX28:read()
   self.fsm.vars.ptu = {pan=ptu_RX28:pan(),
                        tilt=ptu_RX28:tilt()}

   self.args["pantilt"] = {pan=PTU.PAN, tilt=PTU.TILT, max_speed=PTU.SPEED}
end

function DETECT_TABLE:init()
   table_detect:msgq_enqueue_copy(table_detect.EnableSwitchMessage:new())
end
function DETECT_TABLE:exit()
   if self.fsm.vars.table_detect == false then
      table_detect:msgq_enqueue_copy(table_detect.DisableSwitchMessage:new())
   end
end

function DETECT_TABLE_AGAIN:init()
   table_detect:msgq_enqueue_copy(table_detect.EnableSwitchMessage:new())
end
function DETECT_TABLE_AGAIN:exit()
   if self.fsm.vars.table_detect == false then
      table_detect:msgq_enqueue_copy(table_detect.DisableSwitchMessage:new())
   end
end

function ALIGN_AT_TABLE:init()
   navigator:msgq_enqueue_copy(navigator.SetEscapingMessage:new(false))
   self.args["relgoto"] = {x=0, y=0, ori=self.fsm.vars.table_yaw}
end

function POSITION_AT_TABLE:init()
   navigator:msgq_enqueue_copy(navigator.SetEscapingMessage:new(false))
   navigator:msgq_enqueue_copy(navigator.SetDriveModeMessage:new(navigator.AllowBackward))

   self.args["relgoto"] = {x=self.fsm.vars.table_distance - self.fsm.vars.dist - DIST_TO_FRONT, y=0, ori=0}
end
function POSITION_AT_TABLE:exit()
   navigator:msgq_enqueue_copy(navigator.SetEscapingMessage:new(self.fsm.vars.navi.escaping))
   navigator:msgq_enqueue_copy(navigator.SetDriveModeMessage:new(self.fsm.vars.navi.drive_mode))
end

function RESTORE_PTU:init()
   --print("(align_at_table) restoring to pan="..self.fsm.vars.ptu.pan..", tilt="..self.fsm.vars.ptu.tilt)
   self.args["pantilt"] = {pan=self.fsm.vars.ptu.pan,
                           tilt=self.fsm.vars.ptu.tilt,
                           max_speed=PTU.SPEED}
end

function FAILED:init()
   navigator:msgq_enqueue_copy(navigator.StopMessage:new())
end
