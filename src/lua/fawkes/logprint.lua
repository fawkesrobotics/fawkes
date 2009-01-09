
------------------------------------------------------------------------
--  logprint.lua - Print functions for logger
--
--  Created: Sat May 24 18:48:42 2008
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
--
--  $Id$
--
------------------------------------------------------------------------

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

-- store reference to global environment
local _G = _G;
-- these functions we need to register all the others
local pairs = pairs;
local type  = type;

module("fawkes.logprint");

-- we want all functions here, basically what register_global_funcs does for others
for k,v in pairs(_G) do
   _M[k] = v;
end

require("fawkes.stringext");

local logger = nil;

--- Set logger.
-- @param logger_ logger to use
function init(logger_)
   logger = logger_;
end

--- Write to log with log level debug.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function print_debug(format, ...)
   logger:log_debug(string.format(format, ...));
end


--- Write to log with log level info.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function print_info(format, ...)
   logger:log_info(string.format(format, ...));
end


--- Write to log with log level warn.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function print_warn(format, ...)
   logger:log_warn(string.format(format, ...));
end


--- Write to log with log level error.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function print_error(format, ...)
   logger:log_error(string.format(format, ...));
end


--- Print formatted string.
-- Uses string.format to format the string and print_info to print it.
-- @param format format of the string
function printf(format, ...)
   logger:log_info(string.format(format, ...))
end


--- Register print functions for module.
-- @param m module
function register_print_funcs(m)
   m.print_debug = print_debug;
   m.print_info  = print_info;
   m.print_warn  = print_warn;
   m.print_error = print_error;
   m.print       = print_info;
   m.printf      = printf;
end
