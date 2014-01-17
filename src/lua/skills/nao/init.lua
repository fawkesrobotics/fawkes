
----------------------------------------------------------------------------
--  nao.lua - Nao league skill space initialization
--
--  Created: Tue Jul 15 23:18:02 2008
--  Copyright  2008-2009  Tim Niemueller [www.niemueller.de]
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

require("fawkes.modinit")
module(..., fawkes.modinit.register_all);

print("Initializing Lua skill space for Nao robots")


-- Generic skills for all platforms
skillenv.use_skill("skills.generic.relgoto")
skillenv.use_skill("skills.generic.goto")
skillenv.use_skill("skills.generic.turn")
skillenv.use_skill("skills.generic.say")

-- Nao specific but general skills
skillenv.use_skill("skills.nao.led")
skillenv.use_skill("skills.nao.stop")
skillenv.use_skill("skills.nao.walk")
skillenv.use_skill("skills.nao.getup")
skillenv.use_skill("skills.nao.park")
skillenv.use_skill("skills.nao.servo")

--skillenv.use_skill("skills.nao.naorelgoto")
--skillenv.use_skill("skills.nao.naoturn")
skillenv.use_skill("skills.nao.naostrafe")
--skillenv.use_skill("skills.nao.head_pantilt")

skillenv.use_skill("skills.nao.beckon")
skillenv.use_skill("skills.nao.standup")

-- Nao soccer skills
skillenv.use_skill("skills.nao.soccer.kick")
