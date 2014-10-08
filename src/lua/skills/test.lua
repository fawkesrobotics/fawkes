
----------------------------------------------------------------------------
--  test.lua - Skiller test skill space
--
--  Created: Thu Aug 14 17:30:45 2008
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

local _G = _G

require("fawkes.modinit")
module(..., fawkes.modinit.register_all);

--skillenv.use_skill("skills.generic.relgoto")
--skillenv.use_skill("skills.generic.goto")
skillenv.use_skill("skills.generic.say")

if _G.HAVE_ROS then
   local action_skill = require("skiller.ros.action_skill")
   local service_skill = require("skiller.ros.service_skill")

   action_skill.use("test.fibo", "/fibonacci", "actionlib_tutorials/Fibonacci")
end

