
/***************************************************************************
 *  spl.cpp - Fawkes SPL refbox repeater
 *
 *  Created: Tue Jul 08 13:50:06 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "spl.h"
#include "state_handler.h"
#include <core/exception.h>
#include <netcomm/socket/datagram.h>
#include <utils/logging/logger.h>

#ifdef USE_SPL_GC6
#  include <interfaces/SoccerPenaltyInterface.h>
#endif

#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <cerrno>
// it it was defined, Exception::errno() could not be called...
#ifdef errno
#  undef errno
#endif
using namespace fawkes;

#ifdef USE_SPL_GC6
static const uint32_t  SPL_STRUCT_VERSION = 6;
#else
static const uint32_t  SPL_STRUCT_VERSION = 7;
#endif

static const uint8_t   SPL_STATE_INITIAL  = 0;
static const uint8_t   SPL_STATE_READY    = 1;
static const uint8_t   SPL_STATE_SET      = 2;
static const uint8_t   SPL_STATE_PLAYING  = 3;
static const uint8_t   SPL_STATE_FINISHED = 4;

static const uint8_t   SPL_STATE2_NORMAL       = 0;
static const uint8_t   SPL_STATE2_PENALTYSHOOT = 1;

static const uint8_t   SPL_PENALTY_NONE               =  0;
#ifdef USE_SPL_GC6
static const uint8_t   SPL_PENALTY_BALL_HOLDING       =  1;
static const uint8_t   SPL_PENALTY_GOALIE_PUSHING     =  2;
static const uint8_t   SPL_PENALTY_PLAYER_PUSHING     =  3;
static const uint8_t   SPL_PENALTY_ILLEGAL_DEFENDER   =  4;
static const uint8_t   SPL_PENALTY_ILLEGAL_DEFENSE    =  5;
static const uint8_t   SPL_PENALTY_OBSTRUCTION        =  6;
static const uint8_t   SPL_PENALTY_REQ_FOR_PICKUP     =  7;
static const uint8_t   SPL_PENALTY_LEAVING            =  8;
static const uint8_t   SPL_PENALTY_DAMAGE             =  9;
static const uint8_t   SPL_PENALTY_MANUAL             = 10;
#else
static const uint8_t   SPL_PENALTY_BALL_HOLDING       =  1;
static const uint8_t   SPL_PENALTY_PLAYER_PUSHING     =  2;
static const uint8_t   SPL_PENALTY_OBSTRUCTION        =  3;
static const uint8_t   SPL_PENALTY_INACTIVE_PLAYER    =  4;
static const uint8_t   SPL_PENALTY_ILLEGAL_DEFENDER   =  5;
static const uint8_t   SPL_PENALTY_LEAVING_THE_FIELD  =  6;
static const uint8_t   SPL_PENALTY_PLAYING_WITH_HANDS =  7;
static const uint8_t   SPL_PENALTY_REQ_FOR_PICKUP     =  8;
static const uint8_t   SPL_PENALTY_MANUAL             = 15;
#endif

// team numbers
static const uint8_t   SPL_TEAM_BLUE                  =  0;
static const uint8_t   SPL_TEAM_RED                   =  1;

static const uint8_t   SPL_GOAL_BLUE                  =  0;
static const uint8_t   SPL_GOAL_YELLOW                =  1;

static const char    SPL_GAMECONTROL_HEADER[SPL_HEADER_SIZE] = {'R','G','m','e'};


/** @class SplRefBoxProcessor "processor/spl.h"
 * SPL league refbox repeater.
 * This class will listen to SPL refbox commands and derive matching
 * game states from the communication stream and send this via the world info.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger Logger
 * @param broadcast_port Broadcast port
 * @param team_number our team number
 * @param player_number individual player number
 */
SplRefBoxProcessor::SplRefBoxProcessor(fawkes::Logger *logger,
                                       unsigned short int broadcast_port,
                                       unsigned int team_number,
                                       unsigned int player_number)
{
  __player_number = player_number;
  __team_number = team_number;
  __logger = logger;
  __quit = false;
  __s = new DatagramSocket(0.0000000001);
  __s->bind(broadcast_port);

  __penalty = SPL_PENALTY_NONE;
}


/** Destructor. */
SplRefBoxProcessor::~SplRefBoxProcessor()
{
  __s->close();
  delete __s;
}


/** Process received struct. */
void
SplRefBoxProcessor::process_struct(spl_gamecontrol_t *msg)
{
  fawkes::worldinfo_gamestate_team_t our_team;
  //fawkes::worldinfo_gamestate_goalcolor_t our_goal;

  int team_index;
  if (msg->teams[0].team_number == __team_number) team_index = 0;
  else if (msg->teams[1].team_number == __team_number) team_index = 1;
  else return; //Message doesn't concern us

  switch (msg->teams[team_index].team_color) {
    case SPL_TEAM_BLUE:
      our_team = TEAM_CYAN;
      break;
    case SPL_TEAM_RED:
      our_team = TEAM_MAGENTA;
      break;
    default:
      printf("Ignoring faulty packet\n");
      return;
  }

  _rsh->set_score(msg->teams[team_index].score, msg->teams[(team_index == 1 ? 0 : 1)].score);
  _rsh->set_team_goal(our_team, (our_team == TEAM_CYAN ? GOAL_BLUE : GOAL_YELLOW)); //blue team defends blue goal

  for (unsigned int pl_num = 0; pl_num < SPL_MAX_NUM_PLAYERS; ++pl_num)
  {
    if ((pl_num + 1) == __player_number)
    {
      if ((msg->teams[team_index].players[pl_num].penalty != __penalty) ||
          (msg->teams[team_index].players[pl_num].penalty != PENALTY_NONE))
      {
        __penalty = msg->teams[team_index].players[pl_num].penalty;

#ifdef USE_SPL_GC6
	// convert GC6 codes to new GC7 codes, "closest match"
	switch (__penalty) {
	case SPL_PENALTY_BALL_HOLDING:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_BALL_HOLDING; break;
	case SPL_PENALTY_GOALIE_PUSHING:
	case SPL_PENALTY_PLAYER_PUSHING:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_PLAYER_PUSHING; break;
	case SPL_PENALTY_ILLEGAL_DEFENDER:
	case SPL_PENALTY_ILLEGAL_DEFENSE:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_ILLEGAL_DEFENDER; break;
	case SPL_PENALTY_OBSTRUCTION:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_OBSTRUCTION; break;
	case SPL_PENALTY_REQ_FOR_PICKUP:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_REQ_FOR_PICKUP; break;
	case SPL_PENALTY_LEAVING:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_LEAVING_THE_FIELD; break;
	case SPL_PENALTY_DAMAGE:
	case SPL_PENALTY_MANUAL:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_MANUAL; break;
	default:
	  __penalty = SoccerPenaltyInterface::SPL_PENALTY_NONE; break;
	}
#endif

        _rsh->add_penalty(__penalty,
                          msg->teams[team_index].players[pl_num].secs_till_unpenalized);
      }
      break;
    }
  }

  switch (msg->state) {
  case SPL_STATE_INITIAL:
    _rsh->set_gamestate(GS_SPL_INITIAL, TEAM_BOTH);
    break;
  case SPL_STATE_READY:
    _rsh->set_gamestate(GS_SPL_READY, TEAM_BOTH);
    break;
  case SPL_STATE_SET:
    _rsh->set_gamestate(GS_SPL_SET, TEAM_BOTH);
    break;
  case SPL_STATE_PLAYING:
    _rsh->set_gamestate(GS_SPL_PLAY, TEAM_BOTH);
    break;
  case SPL_STATE_FINISHED:
    _rsh->set_gamestate(GS_SPL_FINISHED, TEAM_BOTH);
    break;
  default:
    _rsh->set_gamestate(GS_SPL_FINISHED, TEAM_BOTH);
    break;
  }

  _rsh->set_half((msg->first_half == 1) ? HALF_FIRST : HALF_SECOND,
                 msg->kick_off_team == team_index);
}


void
SplRefBoxProcessor::refbox_process()
{
  try {
    spl_gamecontrol_t ctrlmsg;
    size_t bytes_read = __s->recv((void *)&ctrlmsg, sizeof(ctrlmsg));
    if ( bytes_read == sizeof(ctrlmsg) ) {
      if ((strncmp(ctrlmsg.header, SPL_GAMECONTROL_HEADER, SPL_HEADER_SIZE) == 0) &&
	  (ctrlmsg.version == SPL_STRUCT_VERSION) ) {
	process_struct(&ctrlmsg);
      }
    }
  } catch (fawkes::Exception &e) {
    if ( e.errno() != EAGAIN ) {
      __logger->log_warn("SplRefBoxProcessor", "Receiving failed, exception follows");
      __logger->log_warn("SplRefBoxProcessor", e);
    } // else just no data available this time
  }
}

bool
SplRefBoxProcessor::check_connection()
{
  return true;
}


/** Run.
 * Reads messages from the network, processes them and calls the refbox state sender.
 */
void
SplRefBoxProcessor::run()
{
  spl_gamecontrol_t ctrlmsg;
  while ( ! __quit ) {
    size_t bytes_read = __s->recv((void *)&ctrlmsg, sizeof(ctrlmsg));
    if ( bytes_read == sizeof(ctrlmsg) ) {
      if ( (strncmp(ctrlmsg.header, SPL_GAMECONTROL_HEADER, SPL_HEADER_SIZE) == 0) &&
	   (ctrlmsg.version == SPL_STRUCT_VERSION) ) {
	process_struct(&ctrlmsg);
	_rsh->handle_refbox_state();
      } else {
	printf("Received illegal package\n");
      }
    }
  }
}
