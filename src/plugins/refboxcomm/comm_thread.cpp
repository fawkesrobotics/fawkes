
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
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_ACQUIRE)
{
  refboxproc_ = NULL;
}


void
RefBoxCommThread::init()
{
  try {
    refboxproc_   = NULL;
    gamestate_if_ = NULL;
    beep_if_      = NULL;
#ifdef HAVE_SPL
    penalty_if_   = NULL;
#endif
    last_half_    = (worldinfo_gamestate_half_t)-1;
    last_score_cyan_    = 0xFFFFFFFF;
    last_score_magenta_ = 0xFFFFFFFF;
    last_gamestate_     = -1;
    our_team_ = TEAM_NONE;
    our_goal_color_ = GOAL_BLUE;
    kickoff_ = false;
    gamestate_modified_ = false;

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

    cfg_beep_on_change_ = true;
    cfg_beep_frequency_ = 1000.;
    cfg_beep_duration_  = 0.5;
    try {
      cfg_beep_on_change_ = config->get_bool(CONFPREFIX"/beep_on_change");
    } catch (Exception &e) {} // ignored
    try {
      cfg_beep_frequency_ = config->get_float(CONFPREFIX"/beep_frequency");
    } catch (Exception &e) {} // ignored
    try {
      cfg_beep_duration_ = config->get_float(CONFPREFIX"/beep_duration");
    } catch (Exception &e) {} // ignored
    if (cfg_beep_on_change_) {
      beep_if_ = blackboard->open_for_reading<SwitchInterface>("Beep");
    }

    if ( processor == "MSL" ) {
#ifdef HAVE_MSL2010
      std::string  refbox_host = config->get_string(CONFPREFIX"/MSL/host");
      unsigned int refbox_port = config->get_uint(CONFPREFIX"/MSL/port");
      refboxproc_ = new Msl2010RefBoxProcessor(logger,
						refbox_host.c_str(), refbox_port);
#else
      throw Exception("MSL2010 support not available at compile time");
#endif
    } else if ( processor == "SPL" ) {
#ifdef HAVE_SPL
      unsigned int refbox_port = config->get_uint(CONFPREFIX"/SPL/port");
      team_number_ = config->get_uint("/general/team_number");
      player_number_ = config->get_uint("/general/player_number");
      refboxproc_ = new SplRefBoxProcessor(logger, refbox_port,
                                            team_number_, player_number_);
#else
      throw Exception("SPL support not available at compile time");
#endif
    } else if ( processor == "RemoteBB" ) {
      std::string  bb_host  = config->get_string(CONFPREFIX"/RemoteBB/host");
      unsigned int bb_port  = config->get_uint(CONFPREFIX"/RemoteBB/port");
      std::string  iface_id = config->get_string(CONFPREFIX"/RemoteBB/interface_id");
      refboxproc_ = new RemoteBlackBoardRefBoxProcessor(logger,
							 bb_host.c_str(), bb_port,
							 iface_id.c_str());
    } else {
      throw Exception("Processor %s is not supported by refboxcomm plugin",
		      processor.c_str());
    }
    refboxproc_->set_handler(this);
    gamestate_if_ = blackboard->open_for_writing<GameStateInterface>("RefBoxComm");
#ifdef HAVE_SPL
    penalty_if_   = blackboard->open_for_writing<SoccerPenaltyInterface>("SPL Penalty");
#endif
  } catch (Exception &e) {
    finalize();
    throw;
  }
}


void
RefBoxCommThread::finalize()
{
  delete refboxproc_;
  blackboard->close(gamestate_if_);
  blackboard->close(beep_if_);
#ifdef HAVE_SPL
  blackboard->close(penalty_if_);
#endif
}

void
RefBoxCommThread::loop()
{
  while (!gamestate_if_->msgq_empty()) {
    if (gamestate_if_->msgq_first_is<GameStateInterface::SetTeamColorMessage>()) {
      GameStateInterface::SetTeamColorMessage *msg;
      msg = gamestate_if_->msgq_first<GameStateInterface::SetTeamColorMessage>();
      gamestate_if_->set_our_team(msg->our_team());
      gamestate_modified_ = true;
    } else if (gamestate_if_->msgq_first_is<GameStateInterface::SetStateTeamMessage>()) {
      GameStateInterface::SetStateTeamMessage *msg;
      msg = gamestate_if_->msgq_first<GameStateInterface::SetStateTeamMessage>();
      gamestate_if_->set_state_team(msg->state_team());
      gamestate_modified_ = true;
    } else if (gamestate_if_->msgq_first_is<GameStateInterface::SetKickoffMessage>()) {
      GameStateInterface::SetKickoffMessage *msg;
      msg = gamestate_if_->msgq_first<GameStateInterface::SetKickoffMessage>();
      gamestate_if_->set_kickoff(msg->is_kickoff());
      gamestate_modified_ = true;
    }
    gamestate_if_->msgq_pop();
  }
#ifdef HAVE_SPL
  while (!penalty_if_->msgq_empty()) {
    if (penalty_if_->msgq_first_is<SoccerPenaltyInterface::SetPenaltyMessage>()) {
      SoccerPenaltyInterface::SetPenaltyMessage *msg;
      msg = penalty_if_->msgq_first<SoccerPenaltyInterface::SetPenaltyMessage>();
      penalty_if_->set_penalty(msg->penalty());
      gamestate_modified_ = true;
    }
    penalty_if_->msgq_pop();
  }
#endif
  if (refboxproc_->check_connection()) {
    refboxproc_->refbox_process();
  }
  if (gamestate_modified_) {
    if (cfg_beep_on_change_ && beep_if_->has_writer()) {
      try {
	beep_if_->msgq_enqueue(
	 new SwitchInterface::EnableDurationMessage(cfg_beep_duration_,
						    cfg_beep_frequency_));
      } catch (Exception &e) {} // ignored
    }

    gamestate_if_->write();
#ifdef HAVE_SPL
    penalty_if_->write();
#endif
    gamestate_modified_ = false;
  }
}


void
RefBoxCommThread::set_gamestate(int game_state,
				fawkes::worldinfo_gamestate_team_t state_team)
{
  if (game_state != last_gamestate_) {
    last_gamestate_ = game_state;
    gamestate_modified_ = true;

    logger->log_debug("RefBoxCommThread", "Gamestate: %d   State team: %s",
		      game_state, worldinfo_gamestate_team_tostring(state_team));
    gamestate_if_->set_game_state(game_state);
    switch (state_team) {
    case TEAM_NONE:
      gamestate_if_->set_state_team(GameStateInterface::TEAM_NONE); break;
    case TEAM_CYAN:
      gamestate_if_->set_state_team(GameStateInterface::TEAM_CYAN); break;
    case TEAM_MAGENTA:
      gamestate_if_->set_state_team(GameStateInterface::TEAM_MAGENTA); break;
    case TEAM_BOTH:
      gamestate_if_->set_state_team(GameStateInterface::TEAM_BOTH); break;
    }
  }
}

void
RefBoxCommThread::set_score(unsigned int score_cyan, unsigned int score_magenta)
{
  if ( (score_cyan != last_score_cyan_) || (score_magenta != last_score_magenta_) ) {
    last_score_cyan_    = score_cyan;
    last_score_magenta_ = score_magenta;
    gamestate_modified_ = true;

    logger->log_debug("RefBoxCommThread", "Score (cyan:magenta): %u:%u",
		      score_cyan, score_magenta);
    gamestate_if_->set_score_cyan(score_cyan);
    gamestate_if_->set_score_magenta(score_magenta);
  }
}


void
RefBoxCommThread::set_team_goal(fawkes::worldinfo_gamestate_team_t our_team,
				fawkes::worldinfo_gamestate_goalcolor_t goal_color)
{
  if (our_team != our_team_)
  {
    logger->log_debug("RefBoxCommThread", "Team: %s",
                      worldinfo_gamestate_team_tostring(our_team));

    our_team_ = our_team;
    switch (our_team) {
      case TEAM_CYAN:
        gamestate_if_->set_our_team(GameStateInterface::TEAM_CYAN);
        break;
      case TEAM_MAGENTA:
        gamestate_if_->set_our_team(GameStateInterface::TEAM_MAGENTA);
        break;
      default:
        break;
    }
    gamestate_modified_ = true;
  }

  if (goal_color != our_goal_color_)
  {
    logger->log_debug("RefBoxCommThread", "Our Goal: %s",
                      worldinfo_gamestate_goalcolor_tostring(goal_color));
    our_goal_color_ = goal_color;
    switch (goal_color)
    {
      case GOAL_BLUE:
        gamestate_if_->set_our_goal_color(GameStateInterface::GOAL_BLUE);
        break;
      case GOAL_YELLOW:
        gamestate_if_->set_our_goal_color(GameStateInterface::GOAL_YELLOW);
        break;
    }
    gamestate_modified_ = true;
  }
}


void
RefBoxCommThread::set_half(fawkes::worldinfo_gamestate_half_t half,
                           bool kickoff)
{
  if (half != last_half_) {
    last_half_ = half;
    gamestate_modified_ = true;

    logger->log_debug("RefBoxCommThread", "Half time: %s (Kickoff? %s)",
                      worldinfo_gamestate_half_tostring(half),
                      kickoff ? "yes" : "no");

    switch (half) {
    case HALF_FIRST:
      gamestate_if_->set_half(GameStateInterface::HALF_FIRST);  break;
    case HALF_SECOND:
      gamestate_if_->set_half(GameStateInterface::HALF_SECOND); break;
    }
  }

  if (kickoff != kickoff_)
  {
    kickoff_ = kickoff;
    gamestate_modified_ = true;
    gamestate_if_->set_kickoff(kickoff);
  }
}


void
RefBoxCommThread::add_penalty(unsigned int penalty,
                              unsigned int seconds_remaining)
{
#ifdef HAVE_SPL
  if ((penalty != penalty_if_->penalty()) ||
      (seconds_remaining != penalty_if_->remaining()))
  {
    gamestate_modified_ = true;
    logger->log_debug("RefBoxCommThread", "Penalty %u (%u sec remaining)",
                      penalty, seconds_remaining);
    penalty_if_->set_penalty(penalty);
    penalty_if_->set_remaining(seconds_remaining);
  }
#endif
}



void
RefBoxCommThread::handle_refbox_state()
{
  gamestate_if_->write();
}
