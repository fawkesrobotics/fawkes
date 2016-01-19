
------------------------------------------------------------------------
--  toluaext.lua - tolua++ generator extensions
--
--  Created: Tue Jan 19 14:13:52 2016
--  Copyright  2015  Tim Niemueller [www.niemueller.de]
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

--- Code to add before function calls.
-- Initiates exception handling.
-- @param f
function pre_call_hook(f)
	 print("	 bool exc_caught = false;\n"..
						"  try {")
end

--- Code to add after function calls.
-- Exception handling for fawkes and std exceptions specifically.
-- @param f
function post_call_hook(f)
	 print("  } catch (fawkes::Exception &e) {\n"..
						"    exc_caught = true;\n"..
						"    lua_pushstring(tolua_S, e.what_no_backtrace());\n"..
						"  }\n"..
						"  catch (std::exception &e) {\n"..
						"    exc_caught = true;\n"..
						"    lua_pushstring(tolua_S, e.what());\n"..
						"  }\n"..
				 "  if (exc_caught)  lua_error(tolua_S);\n");
end
