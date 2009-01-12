
----------------------------------------------------------------------------
--  say.lua - Speech Synth skill
--
--  Created: Wed Sep 10 14:44:02 2008 (Cape Town UCT Trip)
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
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
name               = "say"
fsm                = SkillHSM:new{name=name, start="SAY"}
depends_skills     = nil
depends_interfaces = {
   {v = "speechsynth", type = "SpeechSynthInterface"}
}

documentation      = [==[Speech synthesis skill.
Say some string via a speech synthesis device.
and disable the servos by using the following form:

servo(saytext)
]==]

-- Initialize as skill module
skillenv.skill_module(...)

-- States
fsm:new_jump_state("SAY")
fsm:new_jump_state("WAIT")

function SAY:init()
   local text = self.fsm.vars[1] or self.fsm.vars.text
   self.fsm.vars.msgid = speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(text))
end

function WAIT:jumpcond_speechsynth_fail()
   return not speechsynth:has_writer()
          or self.fsm.vars.msgid < speechsynth:msgid()
end

function WAIT:jumpcond_speechsynth_done()
   return self.fsm.vars.msgid == speechsynth:msgid()
          and speechsynth:is_final()
end

SAY:add_precond_trans(FAILED, function (state) return not speechsynth:has_writer() end, "No SpeechSynth provider")
SAY:add_precond_trans(FAILED, function (state) return not state.fsm.vars[1] and not state.fsm.vars.text end, "No text given")
SAY:add_transition(FINAL, function (state) return not state.fsm.vars.wait end, "Speech ordered")
SAY:add_transition(WAIT, function (state) return state.fsm.vars.wait end, "Wait for final")
WAIT:add_transition(FAILED, WAIT.jumpcond_speechsynth_fail, "SpeechSynth failure")
WAIT:add_transition(FINAL, WAIT.jumpcond_speechsynth_done, "SpeechSynth done")

SAY:get_transitions(FINAL).dotattr = { labeloffsety = -15 }
