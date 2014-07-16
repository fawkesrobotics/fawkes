
------------------------------------------------------------------------
--  modinit.lua - Module initialization functions
--
--  Created: Thu Mar 13 16:31:29 2008 (was utils.lua in skiller)
--  Copyright  2008  Tim Niemueller [www.niemueller.de]
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
local _G = _G
-- these functions we need to register all the others
local pairs   = pairs
local type    = type
local require = require

module("fawkes.modinit")
require("fawkes.logprint")

-- we want all functions here, basically what register_global_funcs does for others
for k,v in pairs(_G) do
   _M[k] = v
end

--- Registers global functions for module.
-- @param m module
function register_global_funcs(m)
   for k,v in pairs(_G) do
      if type(v) == "function" then
	 m[k] = v
      end
   end
end


--- Initializes a module and references everything from the global environment.
-- @param m module that contains a skill
function register_all(m)
   for k,v in pairs(_G) do
      m[k] = v
   end
   fawkes.logprint.register_print_funcs(m)
end


--- Initializes a skill module.
-- @param m module that contains a skill
function module_init(m)
   for k,v in pairs(_G) do
      if type(v) == "function" or type(v) == "table" then
	 m[k] = v
      end
   end
   fawkes.logprint.register_print_funcs(m);

   --Export some important Fawkes utils to the module
   m.Time  = fawkes.Time
   m.Clock = fawkes.Clock

   if features_env_template then
      for k,v in pairs(features_env_template) do
	 m[k] = v
      end
   end
end
