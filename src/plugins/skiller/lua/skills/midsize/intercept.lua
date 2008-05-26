
----------------------------------------------------------------------------
--  intercept.lua - Intercept skill
--
--  Created: Mon Apr 07 16:26:49 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

require("skills.midsize")
module("skills.midsize.intercept", skills.midsize.module_init)
require("skills.midsize.relgoto");


local intercept_margin = 0.3;

--- Check status of goto.
-- @param margin the radius of a circle around the destination point,
-- if the robot is within that circle the goto is considered final.
function intercept_checkstatus(margin)
   return midsize.relgoto.relgoto_checkstatus(margin);
end


--- Reset intercept skill.
-- Calls relgoto_reset().
function intercept_reset()
   midsize.relgoto.relgoto_reset();
end


--- Intercept ball skill.
-- See skill documentation for more info.
-- @param ... see skill documentation about supported call styles
function intercept(...)
   -- Check if we reached the destination or if we cannot at all
   local status = intercept_checkstatus(intercept_margin);
   if status ~= S_RUNNING then
      return status;
   end

   local ball_x = wm_ball:relative_x();
   local ball_y = wm_ball:relative_y();

   return relgoto{x=ball_x, y=-ball_y, margin=intercept_margin};
end


intercept_skill_doc = [==[Intercept ball skill.
This skill tries to intercept the ball. It will use the relative ball position and
instruct the monitor to drive to this coordinates. Currently it is pretty dumb.

Parameters:
intercept() does not take any parameters.

The skill status is derived from the internally use relgoto() to get to the ball
position.
]==]

register_skill{name       = "intercept",
	       func       = intercept,
	       reset_func = intercept_reset,
	       doc        = intercept_skill_doc
	      };
