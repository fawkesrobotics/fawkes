
/***************************************************************************
 *  enums.cpp - World Info Transceiver Enums
 *
 *  Created: Wed Apr 09 17:05:15 2008
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

#include "enums.h"

#define CASE_STRING(x) case x: return #x

namespace fawkes {

/** Convert MSL gamestate to a string.
 * @param gamestate game state to translate into a string
 * @return string representation of the given state.
 */
const char *
worldinfo_msl_gamestate_tostring(worldinfo_msl_gamestate_t gamestate)
{
  switch (gamestate) {
    CASE_STRING(GS_FROZEN);
    CASE_STRING(GS_PLAY);
    CASE_STRING(GS_KICK_OFF);
    CASE_STRING(GS_DROP_BALL);
    CASE_STRING(GS_PENALTY);
    CASE_STRING(GS_CORNER_KICK);
    CASE_STRING(GS_THROW_IN);
    CASE_STRING(GS_FREE_KICK);
    CASE_STRING(GS_GOAL_KICK);
    CASE_STRING(GS_HALF_TIME);
  default: return "Unknown MSL Gamestate";
  }
}

/** Convert MSL gamestate to a string.
 * @param gamestate game state to translate into a string
 * @return string representation of the given state.
 */
const char *
worldinfo_spl_gamestate_tostring(worldinfo_spl_gamestate_t gamestate)
{
  switch (gamestate) {
    CASE_STRING(GS_SPL_INITIAL);
    CASE_STRING(GS_SPL_READY);
    CASE_STRING(GS_SPL_SET);
    CASE_STRING(GS_SPL_PLAY);
    CASE_STRING(GS_SPL_FINISHED);
  default: return "Unknown SPL Gamestate";
  }
}


/** Convert gamestate team to a string.
 * @param team game state team to translate into a string
 * @return string representation of the given team.
 */
const char *
worldinfo_gamestate_team_tostring(worldinfo_gamestate_team_t team)
{
  switch (team) {
    CASE_STRING(TEAM_CYAN);
    CASE_STRING(TEAM_MAGENTA);
    CASE_STRING(TEAM_NONE);
    CASE_STRING(TEAM_BOTH);
  default: return "Unknown Team";
  }
}


/** Convert goal color to a string.
 * @param goal_color goal color
 * @return string representation of the given goal color.
 */
const char *
worldinfo_gamestate_goalcolor_tostring(worldinfo_gamestate_goalcolor_t goal_color)
{
  switch (goal_color) {
    CASE_STRING(GOAL_BLUE);
    CASE_STRING(GOAL_YELLOW);
  default: return "Unknown Goal Color";
  }

}


/** Convert half time to a string.
 * @param half half time
 * @return string representation of the given half time.
 */
const char *
worldinfo_gamestate_half_tostring(worldinfo_gamestate_half_t half)
{
  switch (half) {
    CASE_STRING(HALF_FIRST);
    CASE_STRING(HALF_SECOND);
  default: return "Unknown Half";
  }
}


/** Convert penalty to a string.
 * @param penalty penalty to translate into a string
 * @return string representation of the penalty
 */
const char *
worldinfo_penalty_tostring(worldinfo_penalty_t penalty)
{
  switch (penalty) {
    CASE_STRING(PENALTY_NONE);
    CASE_STRING(PENALTY_BALL_HOLDING);
    CASE_STRING(PENALTY_GOALIE_PUSHING);
    CASE_STRING(PENALTY_PLAYER_PUSHING);
    CASE_STRING(PENALTY_ILLEGAL_DEFENDER);
    CASE_STRING(PENALTY_ILLEGAL_DEFENSE);
    CASE_STRING(PENALTY_OBSTRUCTION);
    CASE_STRING(PENALTY_REQ_FOR_PICKUP);
    CASE_STRING(PENALTY_LEAVING);
    CASE_STRING(PENALTY_DAMAGE);
    CASE_STRING(PENALTY_MANUAL);
  default: return "Unknown Penalty";
  }
}

} // end namespace fawkes
