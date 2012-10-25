
----------------------------------------------------------------------------
--  say.lua - Speech Synth skill
--
--  Created: Wed Sep 10 14:44:02 2008 (Cape Town UCT Trip)
--  Copyright  2008  Tim Niemueller [http://www.niemueller.de]
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

-- Jumpconditions
function jumpcond_speechsynth_fail(state)
   return not speechsynth:has_writer()
          or state.fsm.vars.msgid < speechsynth:msgid()
end

function jumpcond_speechsynth_done(state)
   return state.fsm.vars.msgid == speechsynth:msgid()
          and speechsynth:is_final()
end

-- States
fsm:define_states{
   export_to=_M,
   closure={speechsynth=speechsynth},

   {"SAY",  JumpState},
   {"WAIT", JumpState}
}

-- Transitions
fsm:add_transitions{
   {"SAY", "FAILED", cond="not speechsynth:has_writer()", precond_only=true, desc="No SpeechSynth provider"},
   {"SAY", "FAILED", cond="not vars.text", precond_only=true, desc="No text given"},
   {"SAY", "FINAL",  cond="not vars.wait", desc="Speech ordered"},
   {"SAY", "WAIT", cond=true, desc="Wait for final"},

   {"WAIT", "FAILED", cond=jumpcond_speechsynth_fail, desc="SpeechSynth failure"},
   {"WAIT", "FINAL",  cond=jumpcond_speechsynth_done, desc="SpeechSynth done"}
}

-- State functions
function SAY:init()
   local text = self.fsm.vars[1] or self.fsm.vars.text
   self.fsm.vars.msgid = speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(text))
end


-- Transition cosmetics
SAY:get_transitions(FINAL).dotattr = { labeloffsety = -15 }
