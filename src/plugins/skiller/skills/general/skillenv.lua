
----------------------------------------------------------------------------
--  skillenv.lua - Skiller skill environment functions
--
--  Created: Fir Mar 14 22:04:43 2008
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

require("general.utils");

module("general.skillenv", general.utils.register_all);

local print         = general.utils.print_info;
local skills        = {};
local skill_status  = { running = {}, final = {} };

function print_skill_info(skill_entry)
   print("Skill: " .. skill_entry.name);
   print("======================================================================");
   print(skill_entry.doc);
   print("======================================================================");
end

function skill_info(skill)
   local found = false;
   if ( type(skill) == "string" ) then
      for _, s in ipairs(skills) do
	 if ( s.name == skill ) then
	    print_skill_info(s);
	    break;
	 end
      end
   elseif type(skill) == "function" then
      for _, s in ipairs(skills) do
	 if ( s.func == skill ) then
	    print_skill_info(s);
	    break;
	 end
      end
   end

   if ( not found ) then
      print("The queried skill has not been registered");
   end
end

local skill_env_template = {
   -- Skiller related stuff
   logger   = logger,
   config   = config,
   clock    = clock,

   -- Packages
   math     = math,
   os       = { date = os.date, getenv = os.getenv, time = os.time, difftime = os.difftime },
   string   = string,
   table    = table,

   -- Functions
   assert      = assert,
   error       = error,
   ipairs      = ipairs,
   next        = next,
   print       = print,
   pairs       = pairs,
   print       = general.utils.print_info,
   print_debug = general.utils.print_debug,
   print_info  = general.utils.print_info,
   print_warn  = general.utils.print_warn,
   print_error = general.utils.print_error,
   select      = select,
   skill_info  = skill_info,
   tostring    = tostring,
   type        = type,
   unpack      = unpack,
   xpcall      = xpcall
};


function gensandbox()
   local rv = {}
   for k,v in pairs(skill_env_template) do
      rv[k] = v;
   end
   for _, s in ipairs(skills) do
      rv[s.name] = s.func;
   end
   return rv;
end


function reset_final()
   skill_final  = false;
   skill_status = { running = {}, final = {} };
end

function skill_loop_begin(skill_name)
   print("Skill " .. skill_name .. " starts execution");
   table.insert(skill_status.running, skill_name);
end

function skill_loop_end(skill_name, final)
   if ( type(final) ~= "boolean" ) then
      print("Skill " .. skill_name .. " did not return a valid final result.");
   end

   table.insert(skill_status.final, skill_name);
end

function create_skill_wrapper(name, func)
   return function(...)
	     skill_loop_begin(name);
	     rv = {func(...)};
	     skill_loop_end(name, rv[1]);
	     return unpack(rv);
	  end
end

function skills_final()
   return #skill_status.running == #skill_status.final;
end

function register_skill(n, f, d)
   local skillentry = { name = n, func = create_skill_wrapper(n, f), doc = d };
   table.insert(skills, skillentry);
end
