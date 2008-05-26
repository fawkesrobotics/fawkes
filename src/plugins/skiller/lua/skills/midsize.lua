
----------------------------------------------------------------------------
--  midsize.lua - Mid-size league skill initialization
--
--  Created: Thu Mar 13 16:23:43 2008
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

local require = require;

require("general.utils")
require("general.skillenv");

module("midsize", general.utils.register_all);

function module_init(m)
   general.utils.module_init(m);
   general.skillenv.module_init(m);

   m.midsize = _M;
end

-- Require all mid-size skills
require("midsize.example");
require("midsize.relgoto");
require("midsize.goto");
require("midsize.intercept");
require("midsize.agent");
