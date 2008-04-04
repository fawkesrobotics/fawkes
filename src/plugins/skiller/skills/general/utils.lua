
----------------------------------------------------------------------------
--  utils.lua - General skiller Lua utils
--
--  Created: Thu Mar 13 16:31:29 2008
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

-- store reference to global environment
local _G = _G;
-- these functions we need to register all the others
local pairs = pairs;
local type  = type;

module("general.utils");

-- we want all functions here, basically what register_global_funcs does for others
for k,v in pairs(_G) do
   _M[k] = v;
end

require("general.stringext");


-- Write to log with log level debug.
-- @param ... Anything, will be converted to string
function print_debug(...)
   logger:log_debug(string.join(" ", {...}));
end


-- Write to log with log level info.
-- @param ... Anything, will be converted to string
function print_info(...)
   logger:log_info(string.join(" ", {...}));
end


-- Write to log with log level warn.
-- @param ... Anything, will be converted to string
function print_warn(...)
   logger:log_warn(string.join(" ", {...}));
end


-- Write to log with log level error.
-- @param ... Anything, will be converted to string
function print_error(...)
   logger:log_error(string.join(" ", {...}));
end


-- Print formatted string.
-- Uses string.format to format the string and print_info to print it.
-- @param format format of the string
function printf(format, ...)
   logger:log_info(string.format(format, ...));
end

-- Registers global functions for module.
-- @param m module
function register_global_funcs(m)
   for k,v in pairs(_G) do
      if type(v) == "function" then
	 m[k] = v;
      end
   end
end


-- Register print functions for module.
-- @param m module
function register_print_funcs(m)
   m.print_debug = print_debug;
   m.print_info  = print_info;
   m.print_warn  = print_warn;
   m.print_error = print_error;
   m.print       = print_info;
   m.printf      = printf;
end


-- Initializes a module and references everything from the global environment.
-- @param m module that contains a skill
function register_all(m)
   for k,v in pairs(_G) do
      m[k] = v;
   end
   register_print_funcs(m)
end


-- Initializes a skill module.
-- @param m module that contains a skill
function module_init(m)
   register_global_funcs(m);
   register_print_funcs(m);
   m.register_skill = general.skillenv.register_skill;
end
