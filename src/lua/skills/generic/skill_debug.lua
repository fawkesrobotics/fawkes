
----------------------------------------------------------------------------
--  skill_debug.lua - Debugging skills
--
--  Created: Sat Mar 31 23:22:53 2012
--  Copyright  2012  Bahram Maleki-Fard
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
name               = "skill_debug"
fsm                = SkillHSM:new{name=name, start="START", debug=true}
depends_skills     = nil
depends_interfaces = {
   {v = "skiller_debug", type="SkillerDebugInterface", id="Skiller"}
}

documentation      = [==[Skill debugger.
Use this skill to send commands to SkillerDebugInterface.

Optional arguments:
 skill=SKILL;
   Name of the skill to be debugged. Can optionally be "LIST" to
   list all loaded skills, or "SKILL_DEP" to get a skill-dependency
   graph.
   If none is given, "LIST" will be default.

 print=TRUE;
   The result will also be printed in console (can be useful if
   not using webview to check interface values).
]==]

-- Initialize as skill module
skillenv.skill_module(...)

local LIST_STRING = "LIST"

--- Check if graph in interface is the one we desired
-- @return true if we can fetch correct graph
function jc_debug_info_final(state)
   return state.fsm.vars.graph_fsm == skiller_debug:graph_fsm()
end

-- States
fsm:add_transitions{
   closure={LIST_STRING=LIST_STRING},

   {"START", "WAIT", true, desc="msg sent"},

   {"WAIT", "DONE", jc_debug_info_final},

   {"DONE", "PRINT", "vars.print", desc="print result"},
   {"DONE", "FINAL", true}
}

function START:init()
  self.fsm.vars.graph_fsm = self.fsm.vars.skill or LIST_STRING
  skiller_debug:msgq_enqueue_copy(skiller_debug.SetGraphMessage:new(self.fsm.vars.graph_fsm))
end

function WAIT:loop()
  skiller_debug:read()
end

function PRINT:init()
  printf("Graph '%s':\n%s", self.fsm.vars.graph_fsm, skiller_debug:graph())
end