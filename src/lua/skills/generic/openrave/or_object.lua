
----------------------------------------------------------------------------
--  or_object.lua - object handling in openrave
--
--  Created: Thu Mar 03 14:32:47 2011
--  Copyright  2011-2014  Bahram Maleki-Fard
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
name               = "or_object"
fsm                = SkillHSM:new{name=name, start="INIT", debug=false}
depends_skills     = {}
depends_interfaces = {
   {v = "if_openrave", type = "OpenRaveInterface", id="OpenRAVE"}
}

documentation      = [==[OpenRAVE object handling skill.

This skill provides object manipulation for objects in OpenRAVE.
It basically allows all the object manipulation posiibilities provided in
*plugins/openrave/environment.h" (i.e. messages from OpenRAVEInterface)
to be called from the skill level.

We have some modes that apply to all objects:

  or_object{delete_all=true}
   Deletes all objects from environment (except robot parts)

  or_object{release_all=true}
   Releases all object from the currently active robot


The other modes handle individual objects, but we can pass multiple
objects in a table. Some only require the object name, so we pass
all the names in one table:

  or_object{delete={"obj1", "obj2", ...}}
   Deletes objects from environment.
   Each obj_ is a string: name of the object to be deleted.

  or_object{attach={"obj1", obj2, ...}}
   Attaches object to the currently active robot
   Each obj_ can be a string: name of the object to be attached.
                 or a table: {name="NAME", manip_name="MANIP_NAME"}
                     with   NAME: name of the object to be attached
                      MANIP_NAME: name of the manipulator to attach to

  or_object{release={"obj1", "obj2", ...}}
   Releases object from the currently active robot
   Each obj_ is a string: name of the object to be released.

Others require more arguments, so we pass tables for each object:

  or_object{add={obj1, obj2, ...}}
   Add objects to the environment.
   Each obj_ is a table: {name="NAME", path="PATH"}
    NAME: name to be given to object (string)
    PATH: path to xml file defining the object (string)

  or_object{move={obj1, obj2, ...}}
   Moves object to a global position in the environment. Need to consider, that
   coordinates define the center-point of the object.
   Each obj_ is a table: {name="NAME", x=X, y=Y, z=Z}
    NAME: name of the object (string)
    X: x-coordinate (float)
    Y: y-coordinate (float)
    Z: z-coordinate (float)

  or_object{rotate={obj1, obj2, ...}}
   Rotates object around its center point. Rotation values are absolute, not relative
   to current rotation.
   Each obj_ is a table: {name="NAME", x=X, y=Y, z=Z [,w=W]}
    NAME: name of the object (string)
    X: 1st rotation,  around x-axis (float)
    Y: 2nd rotation,  around y-axis (float)
    Z: 3rd rotation,  around z-axis (float)
    W: optional (float). If given, (X,Y,Z,W) is taken as a quaternion

  or_object{rename={obj1, obj2, ...}}
   Renames an object.
   Each obj_ is a table: {name="NAME", new_name="NEW_NAME"}
    NAME: current name of the object (string)
    NEW_NAME: new name of object (string)
]==]

-- Constants
local DEFAULT_ORI = 0.0
local DEFAULT_MARGIN = 0.2

-- Initialize as skill module
skillenv.skill_module(...)

-- Jumpconditions
-- Check if message handling is final
function jc_msg_final(state)
   return state.fsm.vars.msgid == if_openrave:msgid() and
          if_openrave:is_final()
end

function add_object(o)
   if not (o.name and o.path) then
      return 0
   end
   return if_openrave:msgq_enqueue_copy(if_openrave.AddObjectMessage:new( o.name, o.path ))
end
function delete_object(o)
   if not type(o)=="string" then
      return 0
   end
   return if_openrave:msgq_enqueue_copy(if_openrave.DeleteObjectMessage:new( o ))
end
function attach_object(o)
   if type(o)=="string" then
      return if_openrave:msgq_enqueue_copy(if_openrave.AttachObjectMessage:new( o, "" ))
   elseif type(o)=="table" and o.name and o.manip_name then
      return if_openrave:msgq_enqueue_copy(if_openrave.AttachObjectMessage:new( o.name, o.manip_name ))
   else
      return 0
   end
end
function release_object(o)
   if not type(o)=="string" then
      return 0
   end
   return if_openrave:msgq_enqueue_copy(if_openrave.ReleaseObjectMessage:new( o ))
end
function move_object(o)
   if not (o.name and o.x and o.y and o.z) then
      return 0
   end
   return if_openrave:msgq_enqueue_copy(if_openrave.MoveObjectMessage:new( o.name, o.x, o.y, o.z ))
end
function rotate_object(o)
   if not (o.name and o.x and o.y and o.z) then
      return 0
   end

   if o.w then
      return if_openrave:msgq_enqueue_copy(if_openrave.RotateObjectMessage:new( o.name, o.x, o.y, o.z, o.w ))
   else
      return if_openrave:msgq_enqueue_copy(if_openrave.RotateObjectMessage:new( o.name, o.x, o.y, o.z ))
   end
end
function rename_object(o)
   if not (o.name and o.new_name) then
      return 0
   end
   return if_openrave:msgq_enqueue_copy(if_openrave.RenameObjectMessage:new( o.name, o.new_name ))
end

-- States
fsm:define_states{
   export_to=_M,
   closure={if_openrave=if_openrave},

   {"INIT", JumpState},

   {"ADD",         JumpState},
   {"DELETE",      JumpState},
   {"DELETE_ALL",  JumpState},
   {"ATTACH",      JumpState},
   {"RELEASE",     JumpState},
   {"RELEASE_ALL", JumpState},
   {"MOVE",        JumpState},
   {"ROTATE",      JumpState},
   {"RENAME",      JumpState},

   {"PROCESS",      JumpState},

   {"CHECK", JumpState},
   {"CHECK_PROCESS", JumpState}
}

-- Transitions
fsm:add_transitions {
   --~ {"INIT", "FAILED", precond="not if_openrave:has_writer()", desc="no writer for interface"},

   {"INIT", "ADD", cond="vars.add", desc="add"},
   {"INIT", "DELETE", cond="vars.delete", desc="delete"},
   {"INIT", "DELETE_ALL", cond="vars.delete_all", desc="delete all"},
   {"INIT", "ATTACH", cond="vars.attach", desc="attach"},
   {"INIT", "RELEASE", cond="vars.release", desc="release"},
   {"INIT", "RELEASE_ALL", cond="vars.release_all", desc="release all"},
   {"INIT", "MOVE", cond="vars.move", desc="move"},
   {"INIT", "ROTATE", cond="vars.rotate", desc="rotate"},
   {"INIT", "RENAME", cond="vars.rename", desc="rename"},
   {"INIT", "FAILED", cond=true, desc="unknown argument"},

   {"DELETE_ALL", "CHECK", cond=jc_msg_final, desc="final"},
   {"RELEASE_ALL", "CHECK", cond=jc_msg_final, desc="final"},

   {"ADD", "PROCESS", cond=true, desc="set up"},
   {"DELETE", "PROCESS", cond=true, desc="set up"},
   {"ATTACH", "PROCESS", cond=true, desc="set up"},
   {"RELEASE", "PROCESS", cond=true, desc="set up"},
   {"MOVE", "PROCESS", cond=true, desc="set up"},
   {"ROTATE", "PROCESS", cond=true, desc="set up"},
   {"RENAME", "PROCESS", cond=true, desc="set up"},

   {"PROCESS", "FAILED", precond="type(vars.objects) ~= 'table'", desc="arguments needs to be a table"},
   {"PROCESS", "FAILED", precond="#vars.objects == 0", desc="empty argument table"},
   {"PROCESS", "FAILED", cond="vars.bad_args", desc="insufficient arguments"},
   {"PROCESS", "CHECK_PROCESS", cond=jc_msg_final, desc="final"},

   {"CHECK_PROCESS", "CHECK", precond="#vars.objects == 0", desc="all objects processed"},
   {"CHECK_PROCESS", "PROCESS", precond=true, desc="process next object"},

   {"CHECK", "FINAL", cond="vars.success", desc="command succeeded"},
   {"CHECK", "FAILED", cond=true, desc="command failed"}
}

function INIT:init()
   self.fsm.vars.done = false
   self.fsm.vars.success = true
   self.fsm.vars.bad_args = false

   self.fsm.vars.objects = {}
   self.fsm.vars.func = nil
end

function DELETE_ALL:init()
   self.fsm.vars.msgid = if_openrave:msgq_enqueue_copy(if_openrave.DeleteAllObjectsMessage:new())
end

function RELEASE_ALL:init()
   self.fsm.vars.msgid = if_openrave:msgq_enqueue_copy(if_openrave.ReleaseAllObjectsMessage:new())
end

function ADD:init()
   self.fsm.vars.objects = self.fsm.vars.add
   self.fsm.vars.func = add_object
end

function DELETE:init()
   self.fsm.vars.objects = self.fsm.vars.delete
   self.fsm.vars.func = delete_object
end

function ATTACH:init()
   self.fsm.vars.objects = self.fsm.vars.attach
   self.fsm.vars.func = attach_object
end

function RELEASE:init()
   self.fsm.vars.objects = self.fsm.vars.release
   self.fsm.vars.func = release_object
end

function MOVE:init()
   self.fsm.vars.objects = self.fsm.vars.move
   self.fsm.vars.func = move_object
end

function ROTATE:init()
   self.fsm.vars.objects = self.fsm.vars.rotate
   self.fsm.vars.func = rotate_object
end

function RENAME:init()
   self.fsm.vars.objects = self.fsm.vars.rename
   self.fsm.vars.func = rename_object
end


function PROCESS:init()
   local obj = self.fsm.vars.objects[1]
   if obj == nil then
      --print("Object is nil. Forgot table-brackets or string-quotes?")
      self.fsm.vars.bad_args = true
      return
   end

   self.fsm.vars.msgid = self.fsm.vars.func( obj )
   if self.fsm.vars.msgid == 0 then
      --print("Insufficient arguments for this object")
      self.fsm.vars.bad_args = true
   else
      --print("Processed one of "..#self.fsm.vars.objects.." remaining objects")
      table.remove(self.fsm.vars.objects, 1)
   end
end
function PROCESS:exit()
   self.fsm.vars.success = self.fsm.vars.success and if_openrave:is_success()
end

function CHECK:init()
   self.fsm.vars.success = self.fsm.vars.success and if_openrave:is_success()
end
