
----------------------------------------------------------------------------
--  openrave.lua - general openrave skill
--
--  Created: Fri Aug 05 13:31:47 2014
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
name               = "openrave"
fsm                = SkillHSM:new{name=name, start="INIT"}
depends_skills     = {}
depends_interfaces = {
   {v = "if_openrave", type = "OpenRaveInterface", id="OpenRAVE"}
}

documentation      = [==[Generic OpenRAVE skill.

This skill can be used to control openrave in general, for example
starting the viewer, change debug levels, modify the environment etc.

Available modes:

openrave{start_viewer=true}
 Starts the openrave viewer.


]==]

-- Constants
-- Initialize as skill module
skillenv.skill_module(...)

-- Jumpconditions
-- Check if message handling is final
function jc_msg_final(state)
   return state.fsm.vars.msgid == if_openrave:msgid() and
          if_openrave:is_final()
end

-- States
fsm:define_states{
   export_to=_M,
   closure={if_openrave=if_openrave},

   {"INIT", JumpState},

   {"START_VIEWER", JumpState},

   {"CHECK", JumpState}
}

-- Transitions
fsm:add_transitions {
   {"INIT", "FAILED", precond="not if_openrave:has_writer()", desc="no writer for interface"},

   {"INIT", "START_VIEWER", precond="vars.start_viewer", desc="start viewer"},

   {"START_VIEWER", "CHECK", cond=jc_msg_final, desc="final"},

   {"CHECK", "FINAL", cond="if_openrave:is_success()", desc="command succeeded"},
   {"CHECK", "FAILED", cond=true, desc="command failed"}
}

function START_VIEWER:init()
   self.fsm.vars.msgid = if_openrave:msgq_enqueue_copy(if_openrave.StartViewerMessage:new())
end
