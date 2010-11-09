
------------------------------------------------------------------------
--  logprint.lua - Print functions for logger
--
--  Created: Sat May 24 18:48:42 2008
--  Copyright  2008-2010  Tim Niemueller [www.niemueller.de]
--             2010       Carnegie Mellon University
--             2010       Intel Labs Pittsburgh
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
local _G = _G
-- these functions we need to register all the others
local pairs = pairs
local string = string
local print = print
local require = require

module(...)

require("fawkes.stringext")

local logger = nil

--- Set logger.
-- @param logger_ logger to use
function init(logger_)
   logger = logger_
   _M.print       = logger_print_info_unformatted
   _M.printf      = logger_printf
   _M.print_debug = logger_print_debug
   _M.print_info  = logger_print_info
   _M.print_warn  = logger_print_warn
   _M.print_error = logger_print_error
end

--- Write to log with log level debug.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function logger_print_debug(format, ...)
   logger:log_debug(string.format(format, ...))
end


--- Write to log with log level info.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function logger_print_info(format, ...)
   logger:log_info(string.format(format, ...))
end


--- Write to log with log level warn.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function logger_print_warn(format, ...)
   logger:log_warn(string.format(format, ...))
end


--- Write to log with log level error.
-- @param format format of the string
-- @param ... Anything, will be converted to string
function logger_print_error(format, ...)
   logger:log_error(string.format(format, ...))
end


--- Write unformatted to log with log level info.
-- @param ... Anything, will be converted to string
function logger_print_info_unformatted(...)
   logger:log_info(string.join(", ", {...}))
end


--- Print formatted string.
-- Uses string.format to format the string and print_info to print it.
-- @param format format of the string
function logger_printf(format, ...)
   logger:log_info(string.format(format, ...))
end


local function fallback_printf(format, ...)
   print(string.format(format, ...))
end


--- Register print functions for module.
-- @param m module
function register_print_funcs(m)
   if _M.printf then
      m.print_debug = _M.print_debug
      m.print_info  = _M.print_info
      m.print_warn  = _M.print_warn
      m.print_error = _M.print_error
      m.print       = _M.print
      m.printf      = _M.printf
   else
      m.print_debug = fallback_printf
      m.print_info  = fallback_printf
      m.print_warn  = fallback_printf
      m.print_error = fallback_printf
      m.print       = print
      m.printf      = fallback_printf
   end
end
