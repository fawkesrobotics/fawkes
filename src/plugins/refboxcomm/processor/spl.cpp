
/***************************************************************************
 *  spl.cpp - Fawkes SPL refbox repeater
 *
 *  Created: Tue Jul 08 13:50:06 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include "spl.h"
#include "state_handler.h"
#include <core/exception.h>
#include <netcomm/socket/datagram.h>
#include <utils/logging/logger.h>

#include <cstring>
#include <cstdio>
#include <unistd.h>
#include <cerrno>
// it it was defined, Exception::errno() could not be called...
#ifdef errno
#  undef errno
#endif
using namespace fawkes;

static const uint32_t  SPL_STRUCT_VERSION = 6;

static const uint8_t   SPL_STATE_INITIAL  = 0;
static const uint8_t   SPL_STATE_READY    = 1;
static const uint8_t   SPL_STATE_SET      = 2;
static const uint8_t   SPL_STATE_PLAYING  = 3;
static const uint8_t   SPL_STATE_FINISHED = 4;

static const uint8_t   SPL_STATE2_NORMAL       = 0;
static const uint8_t   SPL_STATE2_PENALTYSHOOT = 1;

static const uint8_t   SPL_PENALTY_NONE              =  0;
static const uint8_t   SPL_PENALTY_BALL_HOLDING      =  1;
static const uint8_t   SPL_PENALTY_GOALIE_PUSHING    =  2;
static const uint8_t   SPL_PENALTY_PLAYER_PUSHING    =  3;
static const uint8_t   SPL_PENALTY_ILLEGAL_DEFENDER  =  4;
static const uint8_t   SPL_PENALTY_ILLEGAL_DEFENSE   =  5;
static const uint8_t   SPL_PENALTY_OBSTRUCTION       =  6;
static const uint8_t   SPL_PENALTY_REQ_FOR_PICKUP    =  7;
static const uint8_t   SPL_PENALTY_LEAVING           =  8;
static const uint8_t   SPL_PENALTY_DAMAGE            =  9;
static const uint8_t   SPL_PENALTY_MANUAL            = 10;

// team numbers
static const uint8_t   SPL_TEAM_BLUE                 =  0;
static const uint8_t   SPL_TEAM_RED                  =  1;

static const char    SPL_GAMECONTROL_HEADER[GCHS]  = {'R', 'G', 'm', 'e'};


/** @class SplRefBoxProcessor "processor/spl.h"
 * SPL league refbox repeater.
 * This class will listen to SPL refbox commands and derive matching
 * game states from the communication stream and send this via the world info.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param logger Logger
 * @param broadcast_port Broadcast port
 * @param our_team our initial team
 * @param our_goal our initial goal
 */
SplRefBoxProcessor::SplRefBoxProcessor(fawkes::Logger *logger,
				       unsigned short int broadcast_port,
				       fawkes::worldinfo_gamestate_team_t our_team,
				       fawkes::worldinfo_gamestate_goalcolor_t our_goal)
{
  __logger = logger;
  __quit = false;
  __s = new DatagramSocket(0.0000000001);
  __s->bind(broadcast_port);

  for (unsigned int i = 0; i < MAX_NUM_PLAYERS; ++i) {
    __penalties[i] = SPL_PENALTY_NONE;
  }

  switch (our_team) {
  case TEAM_CYAN:
    __our_team = SPL_TEAM_BLUE; break;
  default:
    __our_team = SPL_TEAM_RED;  break;
  }
  __our_goal = our_goal;
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
  switch (msg->state) {
  case SPL_STATE_INITIAL:
    _rsh->set_gamestate(GS_SPL_INITIAL, TEAM_BOTH);
    break;
  case SPL_STATE_READY:
    _rsh->set_gamestate(GS_SPL_READY,
			(msg->kick_off_team == SPL_TEAM_BLUE) ? TEAM_CYAN : TEAM_MAGENTA);
    break;
  case SPL_STATE_SET:
    _rsh->set_gamestate(GS_SPL_SET,
			(msg->kick_off_team == SPL_TEAM_BLUE) ? TEAM_CYAN : TEAM_MAGENTA);
    break;
  case SPL_STATE_PLAYING:
    _rsh->set_gamestate(GS_SPL_PLAY,
			(msg->kick_off_team == SPL_TEAM_BLUE) ? TEAM_CYAN : TEAM_MAGENTA);
    break;
  case SPL_STATE_FINISHED:
    _rsh->set_gamestate(GS_SPL_FINISHED, TEAM_BOTH);
    break;
  default:
    _rsh->set_gamestate(GS_SPL_FINISHED, TEAM_BOTH); break;
  }

  _rsh->set_half( (msg->first_half == 1) ? HALF_FIRST : HALF_SECOND);

  if (msg->teams[0].team_color == SPL_TEAM_BLUE) {
    _rsh->set_score( msg->teams[0].score, msg->teams[1].score);
  } else {
    _rsh->set_score( msg->teams[1].score, msg->teams[0].score);
  }

  int oti = (msg->teams[0].team_color == __our_team) ? 0 : 1;
  for (unsigned int i = 0; i < MAX_NUM_PLAYERS; ++i) {
    if ( (__penalties[i] != msg->teams[oti].players[i].penalty) ||
	 (msg->teams[oti].players[i].penalty != SPL_PENALTY_NONE) ) {
      _rsh->add_penalty(i, msg->teams[oti].players[i].penalty,
			msg->teams[oti].players[i].secs_till_unpenalized);
      __penalties[i] = msg->teams[oti].players[i].penalty;
    }
  }
}



void
SplRefBoxProcessor::refbox_process()
{
  try {
    spl_gamecontrol_t ctrlmsg;
    size_t bytes_read = __s->recv((void *)&ctrlmsg, sizeof(ctrlmsg));
    if ( bytes_read == sizeof(ctrlmsg) ) {
      if ( (strncmp(ctrlmsg.header, SPL_GAMECONTROL_HEADER, GCHS) == 0) &&
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
      if ( (strncmp(ctrlmsg.header, SPL_GAMECONTROL_HEADER, GCHS) == 0) &&
	   (ctrlmsg.version == SPL_STRUCT_VERSION) ) {
	process_struct(&ctrlmsg);
	_rsh->handle_refbox_state();
      } else {
	printf("Received illegal package\n");
      }
    }
  }
}
