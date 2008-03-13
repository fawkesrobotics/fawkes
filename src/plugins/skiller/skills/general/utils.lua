
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
local __G = _G;
-- these functions we need to register all the others
local pairs = pairs;
local type  = type;

module("general.utils");

-- we want all functions here, basically what register_global_funcs does for others
for k,v in pairs(__G) do
   if type(v) == "function" then
      _M[k] = v;
   end
end


function register_global_funcs(m)
   for k,v in pairs(__G) do
      if type(v) == "function" then
	 m[k] = v;
      end
   end
end

function module_init(m)
   register_global_funcs(m);
end
