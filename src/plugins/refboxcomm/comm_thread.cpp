
/***************************************************************************
 *  comm_thread.cpp - Fawkes RefBox Communication Thread
 *
 *  Created: Sun Apr 19 13:13:43 2009 (on way to German Open 2009)
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
 *             2009  Tobias Kellner
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
#include "processor/remotebb.h"
#ifdef HAVE_MSL2010
#  include "processor/msl2010.h"
#endif
#ifdef HAVE_SPL
#  include "processor/spl.h"
#endif

#include <interfaces/GameStateInterface.h>
#include <interfaces/SwitchInterface.h>
#ifdef HAVE_SPL
#  include <interfaces/SoccerPenaltyInterface.h>
#endif

#define CONFPREFIX "/plugins/refboxcomm"

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
    __beep_if      = NULL;
#ifdef HAVE_SPL
    __penalty_if   = NULL;
#endif
    __last_half    = (worldinfo_gamestate_half_t)-1;
    __last_score_cyan    = 0xFFFFFFFF;
    __last_score_magenta = 0xFFFFFFFF;
    __last_gamestate     = -1;
    __our_team = TEAM_NONE;
    __our_goal_color = GOAL_BLUE;
    __kickoff = false;
    __gamestate_modified = false;

    std::string processor = "";
    try {
      processor = config->get_string(CONFPREFIX"/processor");
    } catch (Exception &e) {
      // try to get league
      std::string  league  = config->get_string("/general/league");
      if (league == "MSL" || league == "SPL") {
	processor = league;
      }
    }
    if (processor == "") {
      throw Exception("No valid processor defined");
    }

    __cfg_beep_on_change = true;
    __cfg_beep_frequency = 1000.;
    __cfg_beep_duration  = 0.5;
    try {
      __cfg_beep_on_change = config->get_bool(CONFPREFIX"/beep_on_change");
    } catch (Exception &e) {} // ignored
    try {
      __cfg_beep_frequency = config->get_float(CONFPREFIX"/beep_frequency");
    } catch (Exception &e) {} // ignored
    try {
      __cfg_beep_duration = config->get_float(CONFPREFIX"/beep_duration");
    } catch (Exception &e) {} // ignored
    if (__cfg_beep_on_change) {
      __beep_if = blackboard->open_for_reading<SwitchInterface>("Beep");
    }

    if ( processor == "MSL" ) {
#ifdef HAVE_MSL2010
      std::string  refbox_host = config->get_string(CONFPREFIX"/MSL/host");
      unsigned int refbox_port = config->get_uint(CONFPREFIX"/MSL/port");
      __refboxproc = new Msl2010RefBoxProcessor(logger,
						refbox_host.c_str(), refbox_port);
#else
      throw Exception("MSL2010 support not available at compile time");
#endif
    } else if ( processor == "SPL" ) {
#ifdef HAVE_SPL
      unsigned int refbox_port = config->get_uint(CONFPREFIX"/SPL/port");
      __team_number = config->get_uint("/general/team_number");
      __player_number = config->get_uint("/general/player_number");
      __refboxproc = new SplRefBoxProcessor(logger, refbox_port,
                                            __team_number, __player_number);
#else
      throw Exception("SPL support not available at compile time");
#endif
    } else if ( processor == "RemoteBB" ) {
      std::string  bb_host  = config->get_string(CONFPREFIX"/RemoteBB/host");
      unsigned int bb_port  = config->get_uint(CONFPREFIX"/RemoteBB/port");
      std::string  iface_id = config->get_string(CONFPREFIX"/RemoteBB/interface_id");
      __refboxproc = new RemoteBlackBoardRefBoxProcessor(logger,
							 bb_host.c_str(), bb_port,
							 iface_id.c_str());
    } else {
      throw Exception("Processor %s is not supported by refboxcomm plugin",
		      processor.c_str());
    }
    __refboxproc->set_handler(this);
    __gamestate_if = blackboard->open_for_writing<GameStateInterface>("RefBoxComm");
#ifdef HAVE_SPL
    __penalty_if   = blackboard->open_for_writing<SoccerPenaltyInterface>("SPL Penalty");
#endif
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
  blackboard->close(__beep_if);
#ifdef HAVE_SPL
  blackboard->close(__penalty_if);
#endif
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
    } else if (__gamestate_if->msgq_first_is<GameStateInterface::SetKickoffMessage>()) {
      GameStateInterface::SetKickoffMessage *msg;
      msg = __gamestate_if->msgq_first<GameStateInterface::SetKickoffMessage>();
      __gamestate_if->set_kickoff(msg->is_kickoff());
      __gamestate_modified = true;
    }
    __gamestate_if->msgq_pop();
  }
#ifdef HAVE_SPL
  while (!__penalty_if->msgq_empty()) {
    if (__penalty_if->msgq_first_is<SoccerPenaltyInterface::SetPenaltyMessage>()) {
      SoccerPenaltyInterface::SetPenaltyMessage *msg;
      msg = __penalty_if->msgq_first<SoccerPenaltyInterface::SetPenaltyMessage>();
      __penalty_if->set_penalty(msg->penalty());
      __gamestate_modified = true;
    }
    __penalty_if->msgq_pop();
  }
#endif
  if (__refboxproc->check_connection()) {
    __refboxproc->refbox_process();
  }
  if (__gamestate_modified) {
    if (__cfg_beep_on_change && __beep_if->has_writer()) {
      try {
	__beep_if->msgq_enqueue(
	 new SwitchInterface::EnableDurationMessage(__cfg_beep_duration,
						    __cfg_beep_frequency));
      } catch (Exception &e) {} // ignored
    }

    __gamestate_if->write();
#ifdef HAVE_SPL
    __penalty_if->write();
#endif
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
  if (our_team != __our_team)
  {
    logger->log_debug("RefBoxCommThread", "Team: %s",
                      worldinfo_gamestate_team_tostring(our_team));

    __our_team = our_team;
    switch (our_team) {
      case TEAM_CYAN:
        __gamestate_if->set_our_team(GameStateInterface::TEAM_CYAN);
        break;
      case TEAM_MAGENTA:
        __gamestate_if->set_our_team(GameStateInterface::TEAM_MAGENTA);
        break;
      default:
        break;
    }
    __gamestate_modified = true;
  }

  if (goal_color != __our_goal_color)
  {
    logger->log_debug("RefBoxCommThread", "Our Goal: %s",
                      worldinfo_gamestate_goalcolor_tostring(goal_color));
    __our_goal_color = goal_color;
    switch (goal_color)
    {
      case GOAL_BLUE:
        __gamestate_if->set_our_goal_color(GameStateInterface::GOAL_BLUE);
        break;
      case GOAL_YELLOW:
        __gamestate_if->set_our_goal_color(GameStateInterface::GOAL_YELLOW);
        break;
    }
    __gamestate_modified = true;
  }
}


void
RefBoxCommThread::set_half(fawkes::worldinfo_gamestate_half_t half,
                           bool kickoff)
{
  if (half != __last_half) {
    __last_half = half;
    __gamestate_modified = true;

    logger->log_debug("RefBoxCommThread", "Half time: %s (Kickoff? %s)",
                      worldinfo_gamestate_half_tostring(half),
                      kickoff ? "yes" : "no");

    switch (half) {
    case HALF_FIRST:
      __gamestate_if->set_half(GameStateInterface::HALF_FIRST);  break;
    case HALF_SECOND:
      __gamestate_if->set_half(GameStateInterface::HALF_SECOND); break;
    }
  }

  if (kickoff != __kickoff)
  {
    __kickoff = kickoff;
    __gamestate_modified = true;
    __gamestate_if->set_kickoff(kickoff);
  }
}


void
RefBoxCommThread::add_penalty(unsigned int penalty,
                              unsigned int seconds_remaining)
{
#ifdef HAVE_SPL
  if ((penalty != __penalty_if->penalty()) ||
      (seconds_remaining != __penalty_if->remaining()))
  {
    __gamestate_modified = true;
    logger->log_debug("RefBoxCommThread", "Penalty %u (%u sec remaining)",
                      penalty, seconds_remaining);
    __penalty_if->set_penalty(penalty);
    __penalty_if->set_remaining(seconds_remaining);
  }
#endif
}



void
RefBoxCommThread::handle_refbox_state()
{
  __gamestate_if->write();
}
