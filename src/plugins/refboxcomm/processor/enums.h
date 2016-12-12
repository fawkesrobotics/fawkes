
/***************************************************************************
 *  enums.h - World Info Transceiver Enums
 *
 *  Created: Wed Apr 09 17:01:54 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __REFBOXCOMM_ENUMS_H_
#define __REFBOXCOMM_ENUMS_H_

namespace fawkes {

/** Game states for RoboCup MSL. */
typedef enum {
  GS_FROZEN       =  0,	/**< Frozen, nothing moves */
  GS_PLAY         =  1,	/**< Play, normal play */
  GS_KICK_OFF     =  2,	/**< Kick off */
  GS_DROP_BALL    =  3,	/**< Referee drops ball, both teams can wrestle for the ball */
  GS_PENALTY      =  4,	/**< Penalty kick */
  GS_CORNER_KICK  =  5,	/**< Corner kick */
  GS_THROW_IN     =  6,	/**< Throw in */
  GS_FREE_KICK    =  7,	/**< Free kick */
  GS_GOAL_KICK    =  8,	/**< Goal kick */
  GS_HALF_TIME    =  9	/**< Half time */
} worldinfo_msl_gamestate_t;

/** Game states for RoboCup SPL. */
typedef enum {
  GS_SPL_INITIAL       =  0,	/**< Initial setup phase. */
  GS_SPL_READY         =  1,	/**< Move to kick-off positions. */
  GS_SPL_SET           =  2,	/**< Wait for kick-off. */
  GS_SPL_PLAY          =  3,	/**< Play! */
  GS_SPL_FINISHED      =  4	/**< Corner kick */
} worldinfo_spl_gamestate_t;


/** Team. */
typedef enum {
  TEAM_NONE    = 0,	/**< No team, not team-specific */
  TEAM_CYAN    = 1,	/**< Cyan team */
  TEAM_MAGENTA = 2,	/**< Magenta team */
  TEAM_BOTH    = 3	/**< Both teams */
} worldinfo_gamestate_team_t;


/** Goal color. */
typedef enum {
  GOAL_BLUE    = 0,	/**< Blue goal */
  GOAL_YELLOW  = 1	/**< Yellow goal */
} worldinfo_gamestate_goalcolor_t;


/** Game time half. */
typedef enum {
  HALF_FIRST   = 0,	/**< First half */
  HALF_SECOND  = 1	/**< Second half */
} worldinfo_gamestate_half_t;

/** Robot penalty code. */
typedef enum {
  PENALTY_NONE              =  0,	/**< No penalty. */
  PENALTY_BALL_HOLDING      =  1,	/**< Robot hold the ball. */
  PENALTY_GOALIE_PUSHING    =  2,	/**< Robot pushed the goalie. */
  PENALTY_PLAYER_PUSHING    =  3,	/**< Robot pushed a player. */
  PENALTY_ILLEGAL_DEFENDER  =  4,	/**< Robot is an illegal defender. */
  PENALTY_ILLEGAL_DEFENSE   =  5,	/**< Illegal defense. */
  PENALTY_OBSTRUCTION       =  6,	/**< Robot obstructs path way. */
  PENALTY_REQ_FOR_PICKUP    =  7,	/**< Robot was requested for pick up. */
  PENALTY_LEAVING           =  8,	/**< Robot has to leave. */
  PENALTY_DAMAGE            =  9,	/**< Robot is damaged. */
  PENALTY_MANUAL            = 10	/**< Manually penalized. */
} worldinfo_penalty_t;

const char * worldinfo_msl_gamestate_tostring(worldinfo_msl_gamestate_t gamestate);
const char * worldinfo_spl_gamestate_tostring(worldinfo_spl_gamestate_t gamestate);
const char * worldinfo_gamestate_team_tostring(worldinfo_gamestate_team_t team);
const char * worldinfo_gamestate_goalcolor_tostring(worldinfo_gamestate_goalcolor_t goal_color);
const char * worldinfo_gamestate_half_tostring(worldinfo_gamestate_half_t half);

const char * worldinfo_penalty_tostring(worldinfo_penalty_t penalty);

} // end namespace fawkes

#endif
