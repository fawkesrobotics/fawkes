
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

local skillenv = require("skills.skiller.skillenv")
module(..., skillenv.module_init)

function execute(saytext)
   if not speechsynth:has_writer() then
      print_warn("say(): Cannot execute without a SpeechSynth provider")
      return S_FAILED
   end

   speechsynth:msgq_enqueue_copy(speechsynth.SayMessage:new(saytext))
   return S_FINAL
end

function reset()
end

-- Global variables required for a skill
name               = "say"
depends_skills     = nil
depends_interfaces = {
   {v = "speechsynth", type = "SpeechSynthInterface"}
}

documentation      = [==[Speech synthesis skill.
Say some string via a speech synthesis device.
and disable the servos by using the following form:

servo(saytext)
]==]
