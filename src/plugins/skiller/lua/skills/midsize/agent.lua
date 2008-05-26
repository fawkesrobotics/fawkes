
----------------------------------------------------------------------------
--  agent.lua - Mid-size league reactive agent
--
--  Created: Fri Apr 11 10:33:42 2008
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

require("midsize")
module("midsize.agent", midsize.module_init)

require("midsize.agent.cornerkick");
require("midsize.agent.dropball");
require("midsize.agent.freekick");
require("midsize.agent.goalkick");
require("midsize.agent.kickoff");
require("midsize.agent.penalty");
require("midsize.agent.play");
require("midsize.agent.throwin");

-- rebind functions to have shorter names
local agent_cornerkick = midsize.agent.cornerkick.exec;
local agent_dropball   = midsize.agent.dropball.exec;
local agent_freekick   = midsize.agent.freekick.exec;
local agent_goalkick   = midsize.agent.goalkick.exec;
local agent_kickoff    = midsize.agent.kickoff.exec;
local agent_penalty    = midsize.agent.penalty.exec;
local agent_play       = midsize.agent.play.exec;
local agent_throwin    = midsize.agent.throwin.exec;

-- local state variables
local last_game_state = gamestate.GS_FROZEN;


--- Agent frozen.
-- Stops immediately execution of any active actuator.
function agent_frozen()
   -- stop navigator
end

--- Agent.
-- See skill documentation for info.
-- @param ... see skill documentation about supported call styles
function lua_agent(args)

   if type(args) ~= "table" then
      error("agent() may only be called with named arguments.");
   end

   local game_state = args.game_state;
   local state_team = args.state_team;
   local role = args.role;
   if game_state == nil then
      game_state = gamestate:game_state();
   end
   if state_team == nil then
      state_team = gamestate:state_team();
   end
   if role == nil then
      role = gamestate:role();
   end

   if game_state == gamestate.GS_FROZEN then
      print_debug("Frozen");
      agent_frozen();
   elseif game_state == gamestate.GS_PLAY then
      print_debug("Playing");
      agent_play(role, last_game_state);
   elseif game_state == gamestate.GS_KICK_OFF then
      print_debug("Kick-off");
      agent_kickoff(role, state_team);
   elseif game_state == gamestate.GS_DROP_BALL then
      print_debug("Dropped Ball");
      agent_dropball(role);
   elseif game_state == gamestate.GS_PENALTY then
      print_debug("Penalty");
      agent_penalty(state_team);
   elseif game_state == gamestate.GS_CORNER_KICK then
      print_debug("Corner kick");
      agent_cornerkick(role, state_team);
   elseif game_state == gamestate.GS_THROW_IN then
      print_debug("Throw-in");
      agent_throwin(role, state_team);
   elseif game_state == gamestate.GS_FREE_KICK then
      print_debug("Free kick");
      agent_freekick(role, state_team);
   elseif game_state == gamestate.GS_GOAL_KICK then
      print_debug("Goal Kick");
      agent_goalkick(role, state_team);
   elseif game_state == gamestate.GS_HALF_TIME then
      print_debug("Half time");
      agent_frozen();
   else
      error("Unknown game state set");
   end

   last_game_state = game_state;

   return S_RUNNING;
end

agent_skill_doc = [==[Reactive agent for the mid-size league.
This "skill" is a simple reactive agent. It is meant to serve as a prototyping and
testing playground.

Parameters:
none

The skill is always S_RUNNING and never finishes.
]==]

register_skill{name       = "agent",
	       func       = lua_agent,
	       doc        = agent_skill_doc
	      };
