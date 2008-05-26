
----------------------------------------------------------------------------
--  example.lua - midsize example skill
--
--  Created: Thu Mar 13 16:08:04 2008
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
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software Foundation,
--  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02111-1307, USA.

require("midsize")
module("midsize.example", midsize.module_init)

function example(arg)
   local status = S_RUNNING;
   local rets   = "foo";

   if ( arg ~= nil ) then
      status = S_FINAL;
      if ( type(arg) == "string" ) then
	 rets = arg;
      end
   else
      print_warn("Argument was NIL");
   end

   print("Example skill has been executed");
   return status, rets;
end

function example_reset()
   print_debug("Example skill reset called");
end

example_skill_doc = [[Simple example skill.
This skill does nothing meaningful, it is just here to show you what
you can do.]]

register_skill{name       = "example",
	       func       = example,
	       reset_func = example_reset,
	       doc        = example_skill_doc};
