
----------------------------------------------------------------------------
--  walk.lua - Skill to start the walk using a motion vector
--
--  Created: Thu Jan 22 00:07:06 2009
--  Copyright  2008-2009  Tim Niemueller [http://www.niemueller.de]
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
name               = "walk"
depends_interfaces = {
   {v = "naomotion", id = "NaoQi Motion", type = "HumanoidMotionInterface"}
}

documentation      = [==[Start or stop walk]==]

-- Initialize as skill module
skillenv.skill_module(...)

function execute(args)
   local args = args or {}
   if naomotion:has_writer() then
      local x = args.x or args[1] or 0
      local y = args.y or args[2] or 0
      local theta = args.theta or args[3] or 0
      local speed = args.speed or args[4] or 0
      local msg = naomotion.WalkMessage:new(x, y, theta, speed)
      args.msgid = naomotion:msgq_enqueue_copy(msg)
      return S_FINAL
   else
      return S_FAILED
   end
end

function reset()
end
