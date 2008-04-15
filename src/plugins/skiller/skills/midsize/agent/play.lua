
----------------------------------------------------------------------------
--  play.lua - Mid-size league reactive agent: play
--
--  Created: Tue Apr 15 11:02:16 2008
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

require("midsize");
module("midsize.agent.play", midsize.module_init);


--- Execute code for GS_PLAY
-- @param role role
-- @param last_game_state_team game state from the last cycle
function exec(role, last_game_state)
   print_debug("Agent play");
end
