
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

function SAY:init()
   if speechsynth:has_writer() then
      self.text = self.fsm.vars[1] or self.fsm.vars.text
      if self.text then
	 speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(self.text))
      end
   end
end

SAY:add_transition(FINAL, function (state) return speechsynth:has_writer() and state.text end, "Text spoken")
SAY:add_transition(FAILED, function (state) return not speechsynth:has_writer() end, "No SpeechSynth provider")
SAY:add_transition(FAILED, function (state) return not state.text end, "No text given")
