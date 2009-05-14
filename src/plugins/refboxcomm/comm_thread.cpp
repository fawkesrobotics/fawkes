
/***************************************************************************
 *  comm_thread.cpp - Fawkes RefBox Communication Thread
 *
 *  Created: Sun Apr 19 13:13:43 2009 (on way to German Open 2009)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *
 *  $Id$
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

#include "comm_thread.h"
#include "processor/msl2008.h"
#include "processor/spl.h"

#include <interfaces/GameStateInterface.h>
#include <interfaces/SplPenaltyInterface.h>

using namespace fawkes;

/** @class RefBoxCommThread "comm_thread.h"
 * Referee Box Communication Thread for robotic soccer.
 * This thread communicates with the refbox.
 * @author Tim Niemueller
 */


/** Constructor. */
RefBoxCommThread::RefBoxCommThread()
  : Thread("RefBoxCommThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR)
{
  __refboxproc = NULL;
}


void
RefBoxCommThread::init()
{
  try {
    __refboxproc   = NULL;
    __gamestate_if = NULL;
    __penalty_if   = NULL;
    __last_half    = (worldinfo_gamestate_half_t)-1;
    __last_score_cyan    = 0xFFFFFFFF;
    __last_score_magenta = 0xFFFFFFFF;
    __last_gamestate     = -1;
    __gamestate_modified = false;

    std::string  league  = config->get_string("/general/league");
    if ( league == "MSL" ) {
      std::string  refbox_host = config->get_string("/refboxcomm/MSL/host");
      unsigned int refbox_port = config->get_uint("/refboxcomm/MSL/port");
      __refboxproc = new Msl2008RefBoxProcessor(refbox_host.c_str(), refbox_port);
    } else if ( league == "SPL" ) {
      unsigned int refbox_port = config->get_uint("/refboxcomm/SPL/port");
      __refboxproc = new SplRefBoxProcessor(logger, refbox_port,
					    TEAM_CYAN, GOAL_BLUE);
    } else {
      throw Exception("League %s is not supported by refboxcomm plugin", league.c_str());
    }
    __refboxproc->set_handler(this);
    __gamestate_if = blackboard->open_for_writing<GameStateInterface>("RefBoxComm");
    __penalty_if   = blackboard->open_for_writing<SplPenaltyInterface>("SPL Penalties");
  } catch (Exception &e) {
    finalize();
    throw;
  }
}


void
RefBoxCommThread::finalize()
{
  delete __refboxproc;
  blackboard->close(__gamestate_if);
  blackboard->close(__penalty_if);
}

void
RefBoxCommThread::loop()
{
  while (!__gamestate_if->msgq_empty()) {
    if (__gamestate_if->msgq_first_is<GameStateInterface::SetTeamColorMessage>()) {
      GameStateInterface::SetTeamColorMessage *msg;
      msg = __gamestate_if->msgq_first<GameStateInterface::SetTeamColorMessage>();
      __gamestate_if->set_our_team(msg->our_team());
      __gamestate_modified = true;
    } else if (__gamestate_if->msgq_first_is<GameStateInterface::SetStateTeamMessage>()) {
      GameStateInterface::SetStateTeamMessage *msg;
      msg = __gamestate_if->msgq_first<GameStateInterface::SetStateTeamMessage>();
      __gamestate_if->set_state_team(msg->state_team());
      __gamestate_modified = true;
    }
  }

  __refboxproc->refbox_process();
  if (__gamestate_modified) {
    __gamestate_if->write();
    __penalty_if->write();
    __gamestate_modified = false;
  }
}


void
RefBoxCommThread::set_gamestate(int game_state,
				fawkes::worldinfo_gamestate_team_t state_team)
{
  if (game_state != __last_gamestate) {
    __last_gamestate = game_state;
    __gamestate_modified = true;

    logger->log_debug("RefBoxCommThread", "Gamestate: %d   State team: %s",
		      game_state, worldinfo_gamestate_team_tostring(state_team));
    __gamestate_if->set_game_state(game_state);
    switch (state_team) {
    case TEAM_NONE:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_NONE); break;
    case TEAM_CYAN:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_CYAN); break;
    case TEAM_MAGENTA:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_MAGENTA); break;
    case TEAM_BOTH:
      __gamestate_if->set_state_team(GameStateInterface::TEAM_BOTH); break;
    }
  }
}

void
RefBoxCommThread::set_score(unsigned int score_cyan, unsigned int score_magenta)
{
  if ( (score_cyan != __last_score_cyan) || (score_magenta != __last_score_magenta) ) {
    __last_score_cyan    = score_cyan;
    __last_score_magenta = score_magenta;
    __gamestate_modified = true;

    logger->log_debug("RefBoxCommThread", "Score (cyan:magenta): %u:%u",
		      score_cyan, score_magenta);
    __gamestate_if->set_score_cyan(score_cyan);
    __gamestate_if->set_score_magenta(score_magenta);
  }
}


void
RefBoxCommThread::set_team_goal(fawkes::worldinfo_gamestate_team_t our_team,
				fawkes::worldinfo_gamestate_goalcolor_t goal_color)
{
  logger->log_debug("RefBoxCommThread", "Our team: %s   Our goal: %s",
		    worldinfo_gamestate_team_tostring(our_team),
		    worldinfo_gamestate_goalcolor_tostring(goal_color));

  switch (our_team) {
  case TEAM_CYAN:
    __gamestate_if->set_our_team(GameStateInterface::TEAM_CYAN); break;
  case TEAM_MAGENTA:
    __gamestate_if->set_our_team(GameStateInterface::TEAM_MAGENTA); break;
  default: break;
  }

  switch (goal_color) {
  case GOAL_BLUE:
    __gamestate_if->set_our_goal_color(GameStateInterface::GOAL_BLUE);   break;
  case GOAL_YELLOW:
    __gamestate_if->set_our_goal_color(GameStateInterface::GOAL_YELLOW); break;
  }
}


void
RefBoxCommThread::set_half(fawkes::worldinfo_gamestate_half_t half)
{
  if (half != __last_half) {
    __last_half = half;
    __gamestate_modified = true;

    logger->log_debug("RefBoxCommThread", "Half time: %s",
		      worldinfo_gamestate_half_tostring(half));

    switch (half) {
    case HALF_FIRST:
      __gamestate_if->set_half(GameStateInterface::HALF_FIRST);  break;
    case HALF_SECOND:
      __gamestate_if->set_half(GameStateInterface::HALF_SECOND); break;
    }
  }
}


void
RefBoxCommThread::add_penalty(unsigned int player, unsigned int penalty,
			      unsigned int seconds_remaining)
{
  if (player < __penalty_if->maxlenof_penalty()) {
    if ((penalty != __penalty_if->penalty(player)) ||
	(seconds_remaining != __penalty_if->remaining(player))) {
      __gamestate_modified = true;
      logger->log_debug("RefBoxCommThread", "Penalty for player %u: %u (%u sec)",
			player, penalty, seconds_remaining);
      __penalty_if->set_penalty(player, penalty);
      __penalty_if->set_remaining(player, seconds_remaining);
    }
  } else {
    logger->log_warn("RefBoxCommThread", "Received penalty for player %u but "
		     "maximum is %u", player, __penalty_if->maxlenof_penalty());
  }
}



void
RefBoxCommThread::handle_refbox_state()
{
  __gamestate_if->write();
}
