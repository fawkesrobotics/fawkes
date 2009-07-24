
/***************************************************************************
 *  GameStateInterface.cpp - Fawkes BlackBoard Interface - GameStateInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
 *
 *  $Id: cpp_generator.cpp 2510 2009-06-09 09:32:58Z tim $
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

#include <interfaces/GameStateInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class GameStateInterface <interfaces/GameStateInterface.h>
 * GameStateInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to the current game state. It is closely related to
      the WorldInfo network protocol.
      @see WorldInfoTransceiver
    
 * @ingroup FawkesInterfaces
 */


/** GS_FROZEN constant */
const unsigned int GameStateInterface::GS_FROZEN = 0;
/** GS_PLAY constant */
const unsigned int GameStateInterface::GS_PLAY = 1;
/** GS_KICK_OFF constant */
const unsigned int GameStateInterface::GS_KICK_OFF = 2;
/** GS_DROP_BALL constant */
const unsigned int GameStateInterface::GS_DROP_BALL = 3;
/** GS_PENALTY constant */
const unsigned int GameStateInterface::GS_PENALTY = 4;
/** GS_CORNER_KICK constant */
const unsigned int GameStateInterface::GS_CORNER_KICK = 5;
/** GS_THROW_IN constant */
const unsigned int GameStateInterface::GS_THROW_IN = 6;
/** GS_FREE_KICK constant */
const unsigned int GameStateInterface::GS_FREE_KICK = 7;
/** GS_GOAL_KICK constant */
const unsigned int GameStateInterface::GS_GOAL_KICK = 8;
/** GS_HALF_TIME constant */
const unsigned int GameStateInterface::GS_HALF_TIME = 9;
/** GS_SPL_INITIAL constant */
const unsigned int GameStateInterface::GS_SPL_INITIAL = 0;
/** GS_SPL_READY constant */
const unsigned int GameStateInterface::GS_SPL_READY = 1;
/** GS_SPL_SET constant */
const unsigned int GameStateInterface::GS_SPL_SET = 2;
/** GS_SPL_PLAY constant */
const unsigned int GameStateInterface::GS_SPL_PLAY = 3;
/** GS_SPL_FINISHED constant */
const unsigned int GameStateInterface::GS_SPL_FINISHED = 4;

/** Constructor */
GameStateInterface::GameStateInterface() : Interface()
{
  data_size = sizeof(GameStateInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (GameStateInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT, "game_state", 1, &data->game_state);
  add_fieldinfo(IFT_UINT, "score_cyan", 1, &data->score_cyan);
  add_fieldinfo(IFT_UINT, "score_magenta", 1, &data->score_magenta);
  add_messageinfo("SetTeamColorMessage");
  add_messageinfo("SetStateTeamMessage");
  unsigned char tmp_hash[] = {0x1f, 0x8c, 0x58, 0x3b, 0x4d, 0xa8, 0x14, 0x3d, 0xcd, 0x36, 0xb4, 0x46, 0x68, 0xcd, 0xc, 0x45};
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
unsigned int
GameStateInterface::game_state() const
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
GameStateInterface::set_game_state(const unsigned int new_game_state)
{
  data->game_state = new_game_state;
}

/** Get state_team value.
 * Team referred to by game state
 * @return state_team value
 */
GameStateInterface::if_gamestate_team_t
GameStateInterface::state_team() const
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
GameStateInterface::our_team() const
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
GameStateInterface::our_goal_color() const
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
GameStateInterface::half() const
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

/** Get role value.
 * Current role of this robot
 * @return role value
 */
GameStateInterface::if_gamestate_role_t
GameStateInterface::role() const
{
  return data->role;
}

/** Get maximum length of role value.
 * @return length of role value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_role() const
{
  return 1;
}

/** Set role value.
 * Current role of this robot
 * @param new_role new role value
 */
void
GameStateInterface::set_role(const if_gamestate_role_t new_role)
{
  data->role = new_role;
}

/** Get score_cyan value.
 * Score of team cyan
 * @return score_cyan value
 */
unsigned int
GameStateInterface::score_cyan() const
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
GameStateInterface::score_magenta() const
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
  if ( strncmp("SetTeamColorMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetTeamColorMessage();
  } else if ( strncmp("SetStateTeamMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetStateTeamMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
GameStateInterface::copy_values(const Interface *other)
{
  const GameStateInterface *oi = dynamic_cast<const GameStateInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(GameStateInterface_data_t));
}

/* =========== messages =========== */
/** @class GameStateInterface::SetTeamColorMessage <interfaces/GameStateInterface.h>
 * SetTeamColorMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_our_team initial value for our_team
 */
GameStateInterface::SetTeamColorMessage::SetTeamColorMessage(const if_gamestate_team_t ini_our_team) : Message("SetTeamColorMessage")
{
  data_size = sizeof(SetTeamColorMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTeamColorMessage_data_t *)data_ptr;
  data->our_team = ini_our_team;
}
/** Constructor */
GameStateInterface::SetTeamColorMessage::SetTeamColorMessage() : Message("SetTeamColorMessage")
{
  data_size = sizeof(SetTeamColorMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTeamColorMessage_data_t *)data_ptr;
}

/** Destructor */
GameStateInterface::SetTeamColorMessage::~SetTeamColorMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
GameStateInterface::SetTeamColorMessage::SetTeamColorMessage(const SetTeamColorMessage *m) : Message("SetTeamColorMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetTeamColorMessage_data_t *)data_ptr;
}

/* Methods */
/** Get our_team value.
 * Our team color
 * @return our_team value
 */
GameStateInterface::if_gamestate_team_t
GameStateInterface::SetTeamColorMessage::our_team() const
{
  return data->our_team;
}

/** Get maximum length of our_team value.
 * @return length of our_team value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::SetTeamColorMessage::maxlenof_our_team() const
{
  return 1;
}

/** Set our_team value.
 * Our team color
 * @param new_our_team new our_team value
 */
void
GameStateInterface::SetTeamColorMessage::set_our_team(const if_gamestate_team_t new_our_team)
{
  data->our_team = new_our_team;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
GameStateInterface::SetTeamColorMessage::clone() const
{
  return new GameStateInterface::SetTeamColorMessage(this);
}
/** @class GameStateInterface::SetStateTeamMessage <interfaces/GameStateInterface.h>
 * SetStateTeamMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_state_team initial value for state_team
 */
GameStateInterface::SetStateTeamMessage::SetStateTeamMessage(const if_gamestate_team_t ini_state_team) : Message("SetStateTeamMessage")
{
  data_size = sizeof(SetStateTeamMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStateTeamMessage_data_t *)data_ptr;
  data->state_team = ini_state_team;
}
/** Constructor */
GameStateInterface::SetStateTeamMessage::SetStateTeamMessage() : Message("SetStateTeamMessage")
{
  data_size = sizeof(SetStateTeamMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStateTeamMessage_data_t *)data_ptr;
}

/** Destructor */
GameStateInterface::SetStateTeamMessage::~SetStateTeamMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
GameStateInterface::SetStateTeamMessage::SetStateTeamMessage(const SetStateTeamMessage *m) : Message("SetStateTeamMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetStateTeamMessage_data_t *)data_ptr;
}

/* Methods */
/** Get state_team value.
 * Team referred to by game state
 * @return state_team value
 */
GameStateInterface::if_gamestate_team_t
GameStateInterface::SetStateTeamMessage::state_team() const
{
  return data->state_team;
}

/** Get maximum length of state_team value.
 * @return length of state_team value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::SetStateTeamMessage::maxlenof_state_team() const
{
  return 1;
}

/** Set state_team value.
 * Team referred to by game state
 * @param new_state_team new state_team value
 */
void
GameStateInterface::SetStateTeamMessage::set_state_team(const if_gamestate_team_t new_state_team)
{
  data->state_team = new_state_team;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
GameStateInterface::SetStateTeamMessage::clone() const
{
  return new GameStateInterface::SetStateTeamMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
GameStateInterface::message_valid(const Message *message) const
{
  const SetTeamColorMessage *m0 = dynamic_cast<const SetTeamColorMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  const SetStateTeamMessage *m1 = dynamic_cast<const SetStateTeamMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(GameStateInterface)
/// @endcond


} // end namespace fawkes
