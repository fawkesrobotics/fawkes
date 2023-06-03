
/***************************************************************************
 *  state_handler.cpp - Fawkes RefBox State Handler Pure Virtual Class
 *
 *  Created: Mon Apr 20 09:49:06 2009 (German Open 2009)
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

#include "state_handler.h"

/** @class RefBoxStateHandler "processor/state_handler.h"
 * Referee Box state handler for RefBoxProcessor.
 * Handlers that implement this interface are used by processors to announce
 * received information.
 * @author Tim Niemueller
 *
 *
 * @fn void RefBoxStateHandler::set_gamestate(int game_state, fawkes::worldinfo_gamestate_team_t state_team) = 0
 * Set current game state.
 * @param game_state current game state
 * @param state_team team referenced by the game state
 *
 * @fn void RefBoxStateHandler::set_score(unsigned int score_cyan, unsigned int score_magenta) = 0
 * Set score.
 * @param score_cyan current score of team cyan
 * @param score_magenta current score of team magenta
 *
 * @fn void RefBoxStateHandler::set_team_goal(fawkes::worldinfo_gamestate_team_t our_team, fawkes::worldinfo_gamestate_goalcolor_t goal_color) = 0
 * Set team and goal info.
 * @param our_team our team color
 * @param goal_color our goal color
 *
 * @fn void RefBoxStateHandler::set_half(fawkes::worldinfo_gamestate_half_t half, bool kickoff = false) = 0
 * Set current half of the game time.
 * @param half current half
 * @param kickoff whether we have kickoff
 *
 * @fn void RefBoxStateHandler::add_penalty(unsigned int penalty, unsigned int seconds_remaining) = 0
 * Add penalty.
 * @param penalty penalty code
 * @param seconds_remaining estimated time when the penalty will be lifted
 *
 * @fn void RefBoxStateHandler::handle_refbox_state() = 0
 * Process the information set up to now.
 */

/** Empty destructor. */
RefBoxStateHandler::~RefBoxStateHandler()
{
}
