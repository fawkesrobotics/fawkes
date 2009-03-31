
------------------------------------------------------------------------
--  general.lua - General Soccer Predicates
--
--  Created: Sat Mar 14 17:43:05 2009
--  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

require("fawkes.predlib")

local math=math

--- This module provides a generic finite state machine (FSM).
-- @author Tim Niemueller
module(..., fawkes.predlib.module_init)

depends_interfaces = {
   {v="wm_ball", id="WM Ball", type="ObjectPositionInterface"},
   {v="gamestate", id="WM GameState", type="GameStateInterface"}
}

local ball_close_dist  = 0.18
local ball_front_angle = 0.25

function ball_visible()
   return wm_ball:is_valid() and wm_ball:is_visible()
end

function ball_close()
   return wm_ball:distance() < ball_close_dist
end

function ball_infront()
   return math.abs(wm_ball:bearing()) < ball_front_angle
end

function gamestate_frozen()
   return gamestate:game_state() == gamestate.GS_FROZEN
end

function gamestate_play()
   return gamestate:game_state() == gamestate.GS_PLAY
end

function gamestate_kick_off()
   return gamestate:game_state() == gamestate.GS_KICK_OFF
end

function gamestate_drop_ball()
   return gamestate:game_state() == gamestate.GS_DROP_BALL
end

function gamestate_penalty()
   return gamestate:game_state() == gamestate.GS_PENALTY
end

function gamestate_corner_kick()
   return gamestate:game_state() == gamestate.GS_CORNER_KICK
end

function gamestate_throw_in()
   return gamestate:game_state() == gamestate.GS_THROW_IN
end

function gamestate_free_kick()
   return gamestate:game_state() == gamestate.GS_FREE_KICK
end

function gamestate_goal_kick()
   return gamestate:game_state() == gamestate.GS_GOAL_KICK
end

function gamestate_half_time()
   return gamestate:game_state() == gamestate.GS_HALF_TIME
end
