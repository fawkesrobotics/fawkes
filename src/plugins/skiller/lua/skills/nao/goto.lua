
----------------------------------------------------------------------------
--  goto.lua - global goto
--
--  Created: Tue Mar 25 23:22:55 2008
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
module("skills.midsize.goto", skills.nao.module_init)
require("skills.midsize.relgoto");

-- Check goto status
function goto_checkstatus(margin)
   return skills.midsize.relgoto.relgoto_checkstatus(margin);
end


-- parameter parsing to support different call styles
function goto_parseparams(...)
   local x, y, ori, margin;

   local f = ...; -- first var
   if type(f) == "table" then
      -- it's called with style 2. or 3.
      if f.x ~= nil and f.y ~= nil then
	 -- style 2.
	 x      = f.x;
	 y      = f.y;
	 ori    = f.ori;
	 margin = f.margin;
	 print_debug("2. goto(x=" .. x .. ", y=" .. y .. ", ori=" .. tostring(ori) .. ", margin=" .. tostring(margin) .. ")");
      else
	 error("goto called with insufficient parameters (named args)");
      end
   else
      -- positional style
      x, y, ori, margin = ...;
      if x == nil or y == nil then
	 error("Insufficient parameters for goto (positional args)");
      end
      print_debug("1. goto(x=" .. x .. ", y=" .. y .. ", ori=" .. tostring(ori) .. ", margin=" .. tostring(margin) .. ")");
   end

   return x, y, ori, margin;
end


function goto(...)
   local x, y, ori, margin = goto_parseparams(...);

   -- ori not yet calculated, not yet in interface
   local rx, ry= wm_pose:world_x(), wm_pose:world_y(); -- robot position

   local relx = x - rx;
   local rely = y - ry;

   printf("Pose(x,y)=(%f, %f)  Dest(x,y)=(%f, %f)  Rel(x,y)=(%f, %f)", rx, ry, x, y, relx, rely);

   return relgoto(relx, rely, ori, margin);
end


function goto_reset()
   skills.midsize.relgoto.relgoto_reset();
end


goto_skill_doc = [==[Global goto skill.
This skill takes you to a position given in global coordinates in the global world
coordinate system. The orientation is the final orientation, nothing is said about the
intermediate orientation while on the way. The margin is the precision of the goto
command. The goto is considered final if the current (x,y) is in a radius of the
given margin around the destination. For example is the margin is 0.5 then the goto
is considered final if the robot is in a circle of 0.5m radius around the target point.
The default margin is 0.2m.

NOTE: This is pretty much coded in the sky, without much backing from lower level software,
do not consider this code to be working, yet.

There are several forms to call this skill:
1. goto(x, y[, ori[, margin]])
   This will goto the position giving in the global world coordinates.
2. goto{x=X, y=Y[, ori=ORI][, margin=MARGIN]}
   Go to the relative cartesian coordinates (X,Y) with the optional final orientation ORI.

Parameters:
x, y:      robot-relative cartesian coordinates of target point
ori:       orientation of robot at destination
margin:    radius of a circle around the destination point, if the robot is within
           that circle the goto is considered final.

The skill is S_RUNNING as long as the target can still be reached, S_FINAL if the target
has been reached (at least once, the robot might move afterwards for example if it did not
brake fast enough or if another robot crashed into this robot). The skill is S_FAILED if
the navigator started processing another goto message.
]==]

register_skill{name       = "goto",
	       func       = goto,
	       reset_func = goto_reset,
	       doc        = goto_skill_doc
	      };
