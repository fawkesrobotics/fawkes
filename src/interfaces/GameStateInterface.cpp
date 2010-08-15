
/***************************************************************************
 *  GameStateInterface.cpp - Fawkes BlackBoard Interface - GameStateInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2008  Tim Niemueller
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
const uint32_t GameStateInterface::GS_FROZEN = 0u;
/** GS_PLAY constant */
const uint32_t GameStateInterface::GS_PLAY = 1u;
/** GS_KICK_OFF constant */
const uint32_t GameStateInterface::GS_KICK_OFF = 2u;
/** GS_DROP_BALL constant */
const uint32_t GameStateInterface::GS_DROP_BALL = 3u;
/** GS_PENALTY constant */
const uint32_t GameStateInterface::GS_PENALTY = 4u;
/** GS_CORNER_KICK constant */
const uint32_t GameStateInterface::GS_CORNER_KICK = 5u;
/** GS_THROW_IN constant */
const uint32_t GameStateInterface::GS_THROW_IN = 6u;
/** GS_FREE_KICK constant */
const uint32_t GameStateInterface::GS_FREE_KICK = 7u;
/** GS_GOAL_KICK constant */
const uint32_t GameStateInterface::GS_GOAL_KICK = 8u;
/** GS_HALF_TIME constant */
const uint32_t GameStateInterface::GS_HALF_TIME = 9u;
/** GS_SPL_INITIAL constant */
const uint32_t GameStateInterface::GS_SPL_INITIAL = 0u;
/** GS_SPL_READY constant */
const uint32_t GameStateInterface::GS_SPL_READY = 1u;
/** GS_SPL_SET constant */
const uint32_t GameStateInterface::GS_SPL_SET = 2u;
/** GS_SPL_PLAY constant */
const uint32_t GameStateInterface::GS_SPL_PLAY = 3u;
/** GS_SPL_FINISHED constant */
const uint32_t GameStateInterface::GS_SPL_FINISHED = 4u;

/** Constructor */
GameStateInterface::GameStateInterface() : Interface()
{
  data_size = sizeof(GameStateInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (GameStateInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "game_state", 1, &data->game_state);
  add_fieldinfo(IFT_ENUM, "state_team", 1, &data->state_team, "if_gamestate_team_t");
  add_fieldinfo(IFT_ENUM, "our_team", 1, &data->our_team, "if_gamestate_team_t");
  add_fieldinfo(IFT_ENUM, "our_goal_color", 1, &data->our_goal_color, "if_gamestate_goalcolor_t");
  add_fieldinfo(IFT_ENUM, "half", 1, &data->half, "if_gamestate_half_t");
  add_fieldinfo(IFT_BOOL, "kickoff", 1, &data->kickoff);
  add_fieldinfo(IFT_ENUM, "role", 1, &data->role, "if_gamestate_role_t");
  add_fieldinfo(IFT_UINT32, "score_cyan", 1, &data->score_cyan);
  add_fieldinfo(IFT_UINT32, "score_magenta", 1, &data->score_magenta);
  add_messageinfo("SetTeamColorMessage");
  add_messageinfo("SetKickoffMessage");
  add_messageinfo("SetStateTeamMessage");
  unsigned char tmp_hash[] = {0xf5, 0x19, 0x26, 0x77, 0x6, 0x54, 0x44, 0xb4, 0xe1, 0x61, 0x40, 0x2a, 0x65, 0xfc, 0xaf, 0xa1};
  set_hash(tmp_hash);
}

/** Destructor */
GameStateInterface::~GameStateInterface()
{
  free(data_ptr);
}
/** Convert if_gamestate_team_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
GameStateInterface::tostring_if_gamestate_team_t(if_gamestate_team_t value) const
{
  switch (value) {
  case TEAM_NONE: return "TEAM_NONE";
  case TEAM_CYAN: return "TEAM_CYAN";
  case TEAM_MAGENTA: return "TEAM_MAGENTA";
  case TEAM_BOTH: return "TEAM_BOTH";
  default: return "UNKNOWN";
  }
}
/** Convert if_gamestate_goalcolor_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
GameStateInterface::tostring_if_gamestate_goalcolor_t(if_gamestate_goalcolor_t value) const
{
  switch (value) {
  case GOAL_BLUE: return "GOAL_BLUE";
  case GOAL_YELLOW: return "GOAL_YELLOW";
  default: return "UNKNOWN";
  }
}
/** Convert if_gamestate_half_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
GameStateInterface::tostring_if_gamestate_half_t(if_gamestate_half_t value) const
{
  switch (value) {
  case HALF_FIRST: return "HALF_FIRST";
  case HALF_SECOND: return "HALF_SECOND";
  default: return "UNKNOWN";
  }
}
/** Convert if_gamestate_role_t constant to string.
 * @param value value to convert to string
 * @return constant value as string.
 */
const char *
GameStateInterface::tostring_if_gamestate_role_t(if_gamestate_role_t value) const
{
  switch (value) {
  case ROLE_GOALIE: return "ROLE_GOALIE";
  case ROLE_DEFENDER: return "ROLE_DEFENDER";
  case ROLE_MID_LEFT: return "ROLE_MID_LEFT";
  case ROLE_MID_RIGHT: return "ROLE_MID_RIGHT";
  case ROLE_ATTACKER: return "ROLE_ATTACKER";
  default: return "UNKNOWN";
  }
}
/* Methods */
/** Get game_state value.
 * Current game state
 * @return game_state value
 */
uint32_t
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
GameStateInterface::set_game_state(const uint32_t new_game_state)
{
  data->game_state = new_game_state;
  data_changed = true;
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
  data_changed = true;
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
  data_changed = true;
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
  data_changed = true;
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
  data_changed = true;
}

/** Get kickoff value.
 * Whether we have kickoff
 * @return kickoff value
 */
bool
GameStateInterface::is_kickoff() const
{
  return data->kickoff;
}

/** Get maximum length of kickoff value.
 * @return length of kickoff value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::maxlenof_kickoff() const
{
  return 1;
}

/** Set kickoff value.
 * Whether we have kickoff
 * @param new_kickoff new kickoff value
 */
void
GameStateInterface::set_kickoff(const bool new_kickoff)
{
  data->kickoff = new_kickoff;
  data_changed = true;
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
  data_changed = true;
}

/** Get score_cyan value.
 * Score of team cyan
 * @return score_cyan value
 */
uint32_t
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
GameStateInterface::set_score_cyan(const uint32_t new_score_cyan)
{
  data->score_cyan = new_score_cyan;
  data_changed = true;
}

/** Get score_magenta value.
 * Score of team magenta
 * @return score_magenta value
 */
uint32_t
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
GameStateInterface::set_score_magenta(const uint32_t new_score_magenta)
{
  data->score_magenta = new_score_magenta;
  data_changed = true;
}

/* =========== message create =========== */
Message *
GameStateInterface::create_message(const char *type) const
{
  if ( strncmp("SetTeamColorMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetTeamColorMessage();
  } else if ( strncmp("SetKickoffMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SetKickoffMessage();
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

const char *
GameStateInterface::enum_tostring(const char *enumtype, int val) const
{
  if (strcmp(enumtype, "if_gamestate_team_t") == 0) {
    return tostring_if_gamestate_team_t((if_gamestate_team_t)val);
  }
  if (strcmp(enumtype, "if_gamestate_goalcolor_t") == 0) {
    return tostring_if_gamestate_goalcolor_t((if_gamestate_goalcolor_t)val);
  }
  if (strcmp(enumtype, "if_gamestate_half_t") == 0) {
    return tostring_if_gamestate_half_t((if_gamestate_half_t)val);
  }
  if (strcmp(enumtype, "if_gamestate_role_t") == 0) {
    return tostring_if_gamestate_role_t((if_gamestate_role_t)val);
  }
  throw UnknownTypeException("Unknown enum type %s", enumtype);
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
  data_ts   = (message_data_ts_t *)data_ptr;
  data->our_team = ini_our_team;
  add_fieldinfo(IFT_ENUM, "our_team", 1, &data->our_team, "if_gamestate_team_t");
}
/** Constructor */
GameStateInterface::SetTeamColorMessage::SetTeamColorMessage() : Message("SetTeamColorMessage")
{
  data_size = sizeof(SetTeamColorMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetTeamColorMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "our_team", 1, &data->our_team, "if_gamestate_team_t");
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
  data_ts   = (message_data_ts_t *)data_ptr;
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
/** @class GameStateInterface::SetKickoffMessage <interfaces/GameStateInterface.h>
 * SetKickoffMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_kickoff initial value for kickoff
 */
GameStateInterface::SetKickoffMessage::SetKickoffMessage(const bool ini_kickoff) : Message("SetKickoffMessage")
{
  data_size = sizeof(SetKickoffMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetKickoffMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->kickoff = ini_kickoff;
  add_fieldinfo(IFT_BOOL, "kickoff", 1, &data->kickoff);
}
/** Constructor */
GameStateInterface::SetKickoffMessage::SetKickoffMessage() : Message("SetKickoffMessage")
{
  data_size = sizeof(SetKickoffMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetKickoffMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_BOOL, "kickoff", 1, &data->kickoff);
}

/** Destructor */
GameStateInterface::SetKickoffMessage::~SetKickoffMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
GameStateInterface::SetKickoffMessage::SetKickoffMessage(const SetKickoffMessage *m) : Message("SetKickoffMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SetKickoffMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get kickoff value.
 * Whether we have kickoff
 * @return kickoff value
 */
bool
GameStateInterface::SetKickoffMessage::is_kickoff() const
{
  return data->kickoff;
}

/** Get maximum length of kickoff value.
 * @return length of kickoff value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
GameStateInterface::SetKickoffMessage::maxlenof_kickoff() const
{
  return 1;
}

/** Set kickoff value.
 * Whether we have kickoff
 * @param new_kickoff new kickoff value
 */
void
GameStateInterface::SetKickoffMessage::set_kickoff(const bool new_kickoff)
{
  data->kickoff = new_kickoff;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
GameStateInterface::SetKickoffMessage::clone() const
{
  return new GameStateInterface::SetKickoffMessage(this);
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
  data_ts   = (message_data_ts_t *)data_ptr;
  data->state_team = ini_state_team;
  add_fieldinfo(IFT_ENUM, "state_team", 1, &data->state_team, "if_gamestate_team_t");
}
/** Constructor */
GameStateInterface::SetStateTeamMessage::SetStateTeamMessage() : Message("SetStateTeamMessage")
{
  data_size = sizeof(SetStateTeamMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SetStateTeamMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_ENUM, "state_team", 1, &data->state_team, "if_gamestate_team_t");
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
  data_ts   = (message_data_ts_t *)data_ptr;
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
  const SetKickoffMessage *m1 = dynamic_cast<const SetKickoffMessage *>(message);
  if ( m1 != NULL ) {
    return true;
  }
  const SetStateTeamMessage *m2 = dynamic_cast<const SetStateTeamMessage *>(message);
  if ( m2 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(GameStateInterface)
/// @endcond


} // end namespace fawkes
