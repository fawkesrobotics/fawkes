
----------------------------------------------------------------------------
--  nao.lua - Nao league skill initialization
--
--  Created: Tue Jul 15 23:18:02 2008
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

require("fawkes.modinit")
module("skills.nao", fawkes.modinit.register_all);
require("skills.common.skillenv");

function module_init(m)
   skills.common.skillenv.module_init(m);

   m.nao = _M;
end

-- Require all nao skills

-- *** Interfaces are not yet available at this point! ***
require("skills.nao.relgoto");
require("skills.nao.intercept");
require("skills.nao.trivialagent");
