
/***************************************************************************
 *  enums.cpp - World Info Transceiver Enums
 *
 *  Created: Wed Apr 09 17:05:15 2008
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

#include <netcomm/worldinfo/enums.h>

#define CASE_STRING(x) case x: return #x

const char *
worldinfo_gamestate_tostring(worldinfo_gamestate_t gamestate)
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
  default: return "Unknown Gamestate";
  }
}


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


const char *
worldinfo_gamestate_goalcolor_tostring(worldinfo_gamestate_goalcolor_t goal_color)
{
  switch (goal_color) {
    CASE_STRING(GOAL_BLUE);
    CASE_STRING(GOAL_YELLOW);
  default: return "Unknown Goal Color";
  }

}


const char * worldinfo_gamestate_half_tostring(worldinfo_gamestate_half_t half)
{
  switch (half) {
    CASE_STRING(HALF_FIRST);
    CASE_STRING(HALF_SECOND);
  default: return "Unknown Half";
  }
}
