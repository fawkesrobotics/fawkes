
/***************************************************************************
 *  state_handler.h - Fawkes RefBox State Handler Pure Virtual Class
 *
 *  Created: Sun Apr 19 17:17:13 2009 (German Open 2009)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_REFBOXCOMM_STATE_HANDLER_H_
#define __PLUGINS_REFBOXCOMM_STATE_HANDLER_H_

#include "enums.h"

class RefBoxStateHandler
{
 public:
  virtual ~RefBoxStateHandler();

  virtual void set_gamestate(int game_state,
			     fawkes::worldinfo_gamestate_team_t state_team)   = 0;
  virtual void set_score(unsigned int score_cyan, unsigned int score_magenta) = 0;
  virtual void set_team_goal(fawkes::worldinfo_gamestate_team_t our_team,
			     fawkes::worldinfo_gamestate_goalcolor_t goal_color) = 0;
  virtual void set_half(fawkes::worldinfo_gamestate_half_t half,
                        bool kickoff = false) = 0;
  virtual void add_penalty(unsigned int penalty,
                           unsigned int seconds_remaining) = 0;


  virtual void handle_refbox_state() = 0;

};

#endif
