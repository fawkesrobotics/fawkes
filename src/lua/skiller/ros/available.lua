
----------------------------------------------------------------------------
--  cond_start.lua - Skiller conditional startup script -- ROS version
--
--  Created: Tue Sep 30 18:00:17 2014
--  Copyright  2014  Tim Niemueller [www.niemueller.de]
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

local require = require

local _M = {}

function check_availability()
   local ok, errmsg = require("roslua")
   return ok, errmsg
end

setmetatable(_M, { __call = check_availability })
return _M
