
/***************************************************************************
 *  enums.h - World Info Transceiver Enums
 *
 *  Created: Wed Apr 09 17:01:54 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __NETCOMM_WORLDINFO_ENUMS_H_
#define __NETCOMM_WORLDINFO_ENUMS_H_

/** Game states. */
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
} worldinfo_gamestate_t;


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


const char * worldinfo_gamestate_tostring(worldinfo_gamestate_t gamestate);
const char * worldinfo_gamestate_team_tostring(worldinfo_gamestate_team_t team);
const char * worldinfo_gamestate_goalcolor_tostring(worldinfo_gamestate_goalcolor_t goal_color);
const char * worldinfo_gamestate_half_tostring(worldinfo_gamestate_half_t half);

#endif
