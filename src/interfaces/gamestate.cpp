
/***************************************************************************
 *  gamestate.cpp - Fawkes BlackBoard Interface - GameStateInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <interfaces/gamestate.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

/** @class GameStateInterface interfaces/gamestate.h
 * GameStateInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to the current game state. It is closely related to
      the WorldInfo network protocol.
      @see WorldInfoTransceiver
    
 */



/** Constructor */
GameStateInterface::GameStateInterface() : Interface()
{
  data_size = sizeof(GameStateInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (GameStateInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_UINT, "score_cyan", &data->score_cyan);
  add_fieldinfo(Interface::IFT_UINT, "score_magenta", &data->score_magenta);
  unsigned char tmp_hash[] = {0x18, 0x6a, 0xf7, 0xc0, 0x53, 0x9b, 0x6b, 0x7b, 0xab, 0x64, 0x54, 0x50, 0xda, 0xd4, 0x80, 0x79};
  set_hash(tmp_hash);
}

/** Destructor */
GameStateInterface::~GameStateInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get game_state value.
 * Current game state
 * @return game_state value
 */
GameStateInterface::if_gamestate_t
GameStateInterface::game_state()
{
  return data->game_state;
}

/** Get maximum length of game_state value.
 * @return length of game_state value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_game_state() const
{
  return 1;
}

/** Set game_state value.
 * Current game state
 * @param new_game_state new game_state value
 */
void
GameStateInterface::set_game_state(const if_gamestate_t new_game_state)
{
  data->game_state = new_game_state;
}

/** Get state_team value.
 * Team referred to by game state
 * @return state_team value
 */
GameStateInterface::if_gamestate_team_t
GameStateInterface::state_team()
{
  return data->state_team;
}

/** Get maximum length of state_team value.
 * @return length of state_team value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_state_team() const
{
  return 1;
}

/** Set state_team value.
 * Team referred to by game state
 * @param new_state_team new state_team value
 */
void
GameStateInterface::set_state_team(const if_gamestate_team_t new_state_team)
{
  data->state_team = new_state_team;
}

/** Get our_team value.
 * Our team color
 * @return our_team value
 */
GameStateInterface::if_gamestate_team_t
GameStateInterface::our_team()
{
  return data->our_team;
}

/** Get maximum length of our_team value.
 * @return length of our_team value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_our_team() const
{
  return 1;
}

/** Set our_team value.
 * Our team color
 * @param new_our_team new our_team value
 */
void
GameStateInterface::set_our_team(const if_gamestate_team_t new_our_team)
{
  data->our_team = new_our_team;
}

/** Get our_goal_color value.
 * Our own goal color
 * @return our_goal_color value
 */
GameStateInterface::if_gamestate_goalcolor_t
GameStateInterface::our_goal_color()
{
  return data->our_goal_color;
}

/** Get maximum length of our_goal_color value.
 * @return length of our_goal_color value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_our_goal_color() const
{
  return 1;
}

/** Set our_goal_color value.
 * Our own goal color
 * @param new_our_goal_color new our_goal_color value
 */
void
GameStateInterface::set_our_goal_color(const if_gamestate_goalcolor_t new_our_goal_color)
{
  data->our_goal_color = new_our_goal_color;
}

/** Get half value.
 * Current game half
 * @return half value
 */
GameStateInterface::if_gamestate_half_t
GameStateInterface::half()
{
  return data->half;
}

/** Get maximum length of half value.
 * @return length of half value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_half() const
{
  return 1;
}

/** Set half value.
 * Current game half
 * @param new_half new half value
 */
void
GameStateInterface::set_half(const if_gamestate_half_t new_half)
{
  data->half = new_half;
}

/** Get score_cyan value.
 * Score of team cyan
 * @return score_cyan value
 */
unsigned int
GameStateInterface::score_cyan()
{
  return data->score_cyan;
}

/** Get maximum length of score_cyan value.
 * @return length of score_cyan value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_score_cyan() const
{
  return 1;
}

/** Set score_cyan value.
 * Score of team cyan
 * @param new_score_cyan new score_cyan value
 */
void
GameStateInterface::set_score_cyan(const unsigned int new_score_cyan)
{
  data->score_cyan = new_score_cyan;
}

/** Get score_magenta value.
 * Score of team magenta
 * @return score_magenta value
 */
unsigned int
GameStateInterface::score_magenta()
{
  return data->score_magenta;
}

/** Get maximum length of score_magenta value.
 * @return length of score_magenta value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_score_magenta() const
{
  return 1;
}

/** Set score_magenta value.
 * Score of team magenta
 * @param new_score_magenta new score_magenta value
 */
void
GameStateInterface::set_score_magenta(const unsigned int new_score_magenta)
{
  data->score_magenta = new_score_magenta;
}

/* =========== message create =========== */
Message *
GameStateInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
GameStateInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(GameStateInterface)
/// @endcond

