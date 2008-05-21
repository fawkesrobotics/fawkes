
/***************************************************************************
 *  gamestate.h - Fawkes BlackBoard Interface - GameStateInterface
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

#ifndef __INTERFACES_GAMESTATE_H_
#define __INTERFACES_GAMESTATE_H_

#include <interface/interface.h>
#include <interface/message.h>

namespace fawkes {

class GameStateInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(GameStateInterface)
 /// @endcond
 public:
  /* constants */

  /** 
        Enumeration defining the different game states. Keep in sync with
	worldinfo_gamestate_t.
       */
  typedef enum {
    GS_FROZEN /**< Frozen, nothing moves. */,
    GS_PLAY /**< Play, normal play */,
    GS_KICK_OFF /**< Kick off */,
    GS_DROP_BALL /**< Referee drops ball, both teams can wrestle for the ball */,
    GS_PENALTY /**< Penalty kick */,
    GS_CORNER_KICK /**< Corner kick */,
    GS_THROW_IN /**< Throw in */,
    GS_FREE_KICK /**< Free kick */,
    GS_GOAL_KICK /**< Goal kick */,
    GS_HALF_TIME /**< Half time */
  } if_gamestate_t;

  /** 
        Enumeration defining the different teams. Keep in sync with
	worldinfo_gamestate_team_t.
       */
  typedef enum {
    TEAM_NONE /**< No team, not team-specific */,
    TEAM_CYAN /**< Cyan team */,
    TEAM_MAGENTA /**< Magenta team */,
    TEAM_BOTH /**< Both teams */
  } if_gamestate_team_t;

  /** 
        Enumeration defining the different teams. Keep in sync with
	worldinfo_gamestate_goalcolor_t.
       */
  typedef enum {
    GOAL_BLUE /**< Blue goal */,
    GOAL_YELLOW /**< Yellow goal */
  } if_gamestate_goalcolor_t;

  /** 
        Enumeration defining the different teams. Keep in sync with
	worldinfo_gamestate_half_t.
       */
  typedef enum {
    HALF_FIRST /**< First half */,
    HALF_SECOND /**< Second half */
  } if_gamestate_half_t;

  /** 
        Enumeration defining the different robot roles. Keep in sync with
	worldinfo_gamestate_role_t.
       */
  typedef enum {
    ROLE_GOALIE /**< Goalie */,
    ROLE_DEFENDER /**< Defender */,
    ROLE_ATTACKER /**< Attacker */
  } if_gamestate_role_t;

 private:
  /** Internal data storage, do NOT modify! */
  typedef struct {
    unsigned int score_cyan; /**< Score of team cyan */
    unsigned int score_magenta; /**< Score of team magenta */
    if_gamestate_t game_state; /**< Current game state */
    if_gamestate_team_t state_team; /**< Team referred to by game state */
    if_gamestate_team_t our_team; /**< Our team color */
    if_gamestate_goalcolor_t our_goal_color; /**< Our own goal color */
    if_gamestate_half_t half; /**< Current game half */
    if_gamestate_role_t role; /**< Current role of this robot */
  } GameStateInterface_data_t;

  GameStateInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  GameStateInterface();
  ~GameStateInterface();

 public:
  virtual Message * create_message(const char *type) const;

  /* Methods */
  if_gamestate_t game_state();
  void set_game_state(const if_gamestate_t new_game_state);
  size_t maxlenof_game_state() const;
  if_gamestate_team_t state_team();
  void set_state_team(const if_gamestate_team_t new_state_team);
  size_t maxlenof_state_team() const;
  if_gamestate_team_t our_team();
  void set_our_team(const if_gamestate_team_t new_our_team);
  size_t maxlenof_our_team() const;
  if_gamestate_goalcolor_t our_goal_color();
  void set_our_goal_color(const if_gamestate_goalcolor_t new_our_goal_color);
  size_t maxlenof_our_goal_color() const;
  if_gamestate_half_t half();
  void set_half(const if_gamestate_half_t new_half);
  size_t maxlenof_half() const;
  if_gamestate_role_t role();
  void set_role(const if_gamestate_role_t new_role);
  size_t maxlenof_role() const;
  unsigned int score_cyan();
  void set_score_cyan(const unsigned int new_score_cyan);
  size_t maxlenof_score_cyan() const;
  unsigned int score_magenta();
  void set_score_magenta(const unsigned int new_score_magenta);
  size_t maxlenof_score_magenta() const;

};

} // end namespace fawkes

#endif
