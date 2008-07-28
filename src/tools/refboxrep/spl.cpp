
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
#include <netcomm/socket/datagram.h>

#include <cstring>
#include <cstdio>
#include <unistd.h>

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


/** @class SplRefBoxRepeater <tools/refboxrep/spl.h>
 * SPL league refbox repeater.
 * This class will listen to SPL refbox commands and derive matching
 * game states from the communication stream and send this via the world info.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rss refbox state sender
 * @param broadcast_ip Broadcast IP
 * @param broadcast_port Broadcast port
 */
SplRefBoxRepeater::SplRefBoxRepeater(RefBoxStateSender &rss,
				     const char *broadcast_ip,
				     unsigned short int broadcast_port)
  : __rss(rss)
{
  __quit = false;
  __s = new DatagramSocket();
  __s->bind(broadcast_port);
}


/** Destructor. */
SplRefBoxRepeater::~SplRefBoxRepeater()
{
  __s->close();
  delete __s;
}


/** Process received struct. */
void
SplRefBoxRepeater::process_struct(spl_gamecontrol_t *msg)
{
  switch (msg->state) {
  case SPL_STATE_READY:
    __rss.set_gamestate(GS_KICK_OFF,
			(msg->kick_off_team == SPL_TEAM_BLUE) ? TEAM_CYAN : TEAM_MAGENTA);
    break;
  case SPL_STATE_PLAYING:
    __rss.set_gamestate(GS_PLAY, TEAM_BOTH); break;
  default:
    __rss.set_gamestate(GS_FROZEN, TEAM_BOTH); break;
  }

  __rss.set_half( (msg->first_half == 1) ? HALF_FIRST : HALF_SECOND);

  if (msg->teams[0].team_color == SPL_TEAM_BLUE) {
    __rss.set_score( msg->teams[0].score, msg->teams[1].score);
  } else {
    __rss.set_score( msg->teams[1].score, msg->teams[0].score);
  }

  __rss.send();
}


/** Run.
 * Reads messages from the network, processes them and calls the refbox state sender.
 */
void
SplRefBoxRepeater::run()
{
  spl_gamecontrol_t ctrlmsg;
  while ( ! __quit ) {
    size_t bytes_read = __s->recv((void *)&ctrlmsg, sizeof(ctrlmsg));
    if ( bytes_read == sizeof(ctrlmsg) ) {
      if ( (strncmp(ctrlmsg.header, SPL_GAMECONTROL_HEADER, GCHS) == 0) &&
	   (ctrlmsg.version == SPL_STRUCT_VERSION) ) {
	process_struct(&ctrlmsg);
      } else {
	printf("Received illegal package\n");
      }
    }
  }
}
