
/***************************************************************************
 *  msl2007.cpp - Fawkes mid-size refbox repeater
 *
 *  Created: Wed Apr 09 10:38:16 2008
 *  Copyright  2008  Tim Niemueller [www.niemueller.de]
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

#include <tools/refboxrep/msl2007.h>
#include <netcomm/socket/stream.h>

#include <cstring>
#include <cstdio>
#include <unistd.h>

using namespace fawkes;

// REFBOX_CODES //////////////////////////
static const char REFBOX_TEST                = '*';
static const char REFBOX_CANCEL              = 'x';
static const char REFBOX_START               = 's';
static const char REFBOX_HALT                = 'H';
static const char REFBOX_STOP                = 'S';
//static const char REFBOX_ENDHALF             = 'H';
static const char REFBOX_DROPPED_BALL        = 'N';

static const char REFBOX_FIRST_HALF          = '1';
static const char REFBOX_SECOND_HALF         = '2';
static const char REFBOX_HALF_TIME           = 'h';
//static const char REFBOX_END_GAME            = 'e';

static const char REFBOX_GOAL_CYAN           = 'A';
static const char REFBOX_GOAL_MAGENTA        = 'a';

static const char REFBOX_KICK_OFF_CYAN       = 'K';
static const char REFBOX_KICK_OFF_MAGENTA    = 'k';
static const char REFBOX_FREE_KICK_CYAN      = 'F';
static const char REFBOX_FREE_KICK_MAGENTA   = 'f';
static const char REFBOX_GOAL_KICK_CYAN      = 'G';
static const char REFBOX_GOAL_KICK_MAGENTA   = 'g';
static const char REFBOX_THROW_IN_CYAN       = 'T';
static const char REFBOX_THROW_IN_MAGENTA    = 't';
static const char REFBOX_CORNER_KICK_CYAN    = 'C';
static const char REFBOX_CORNER_KICK_MAGENTA = 'c';
static const char REFBOX_PENALTY_CYAN        = 'P';
static const char REFBOX_PENALTY_MAGENTA     = 'p';

static const char *  REFBOX_WELCOME          = "Welcome..";
//static const char *  REFBOX_RECONNECT        = "Reconnect";

/** @class MidsizeRefBoxRepeater <tools/refboxrep/msl2007.h>
 * Mid-size league refbox repeater.
 * This class will communicate with the mid-size league refbox and derive matching
 * game states from the communiation stream and send this via the world info.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rss refbox state sender
 * @param refbox_host refbox host
 * @param refbox_port refbox port
 */
MidsizeRefBoxRepeater::MidsizeRefBoxRepeater(RefBoxStateSender &rss,
					     const char *refbox_host,
					     unsigned short int refbox_port)
  : __rss(rss)
{
  __quit = false;
  __s = NULL;
  __score_cyan = __score_magenta = 0;

  __refbox_host = strdup(refbox_host);
  __refbox_port = refbox_port;

  reconnect();
}


/** Destructor. */
MidsizeRefBoxRepeater::~MidsizeRefBoxRepeater()
{
  free(__refbox_host);
  __s->close();
  delete __s;
}


/** Reconnect to refbox. */
void
MidsizeRefBoxRepeater::reconnect()
{
  if ( __s ) {
    __s->close();
    delete __s;
  }
  printf("Trying to connect to refbox at %s:%u\n", __refbox_host, __refbox_port);
  do {
    try {
      __s = new StreamSocket();
      __s->connect(__refbox_host, __refbox_port);

      char welcomebuf[strlen(REFBOX_WELCOME) + 1];
      welcomebuf[strlen(REFBOX_WELCOME)] = 0;
      __s->read(welcomebuf, strlen(REFBOX_WELCOME));
      printf("Connected, received welcome string: %s\n", welcomebuf);
    } catch (Exception &e) {
      delete __s;
      __s = NULL;
      printf(".");
      fflush(stdout);
      usleep(500000);
    }
  } while ( ! __s );
}


/** Process received string. */
void
MidsizeRefBoxRepeater::process_string(char *buf, size_t len)
{
  for (size_t b = 0; b < len; ++b) {
    switch (buf[b]) {
    case REFBOX_TEST:
      // immediately reply
      printf("Received connection test, replying\n");
      __s->write("*", 1);
      break;

    case REFBOX_CANCEL:
      printf("RefBox cancelled last command\n");
      break;

    case REFBOX_START:
      __rss.set_gamestate(GS_PLAY, TEAM_BOTH);
      break;

    case REFBOX_HALT:
    case REFBOX_STOP:
      __rss.set_gamestate(GS_FROZEN, TEAM_BOTH);
      break;

    case REFBOX_DROPPED_BALL:
      __rss.set_gamestate(GS_DROP_BALL, TEAM_BOTH);
      break;

    case REFBOX_GOAL_CYAN:
      __rss.set_score(++__score_cyan, __score_magenta);
      __rss.set_gamestate(GS_FROZEN, TEAM_BOTH);
      break;

    case REFBOX_GOAL_MAGENTA:
      __rss.set_score(__score_cyan, ++__score_magenta);
      __rss.set_gamestate(GS_FROZEN, TEAM_BOTH);
      break;

    case REFBOX_KICK_OFF_CYAN:
      __rss.set_gamestate(GS_KICK_OFF, TEAM_CYAN);
      break;

    case REFBOX_KICK_OFF_MAGENTA:
      __rss.set_gamestate(GS_KICK_OFF, TEAM_MAGENTA);
      break;

    case REFBOX_PENALTY_CYAN:
      __rss.set_gamestate(GS_PENALTY, TEAM_CYAN);
      break;

    case REFBOX_PENALTY_MAGENTA:
      __rss.set_gamestate(GS_PENALTY, TEAM_MAGENTA);
      break;
      
    case REFBOX_CORNER_KICK_CYAN:
      __rss.set_gamestate(GS_CORNER_KICK, TEAM_CYAN);
      break;

    case REFBOX_CORNER_KICK_MAGENTA:
      __rss.set_gamestate(GS_CORNER_KICK, TEAM_MAGENTA);
      break;

    case REFBOX_THROW_IN_CYAN:
      __rss.set_gamestate(GS_THROW_IN, TEAM_CYAN);
      break;

    case REFBOX_THROW_IN_MAGENTA:
      __rss.set_gamestate(GS_THROW_IN, TEAM_MAGENTA);
      break;

    case REFBOX_FREE_KICK_CYAN:
       __rss.set_gamestate(GS_FREE_KICK, TEAM_CYAN);
      break;

    case REFBOX_FREE_KICK_MAGENTA:
      __rss.set_gamestate(GS_FREE_KICK, TEAM_MAGENTA);
      break;

    case REFBOX_GOAL_KICK_CYAN:
       __rss.set_gamestate(GS_GOAL_KICK, TEAM_CYAN);
      break;

    case REFBOX_GOAL_KICK_MAGENTA:
      __rss.set_gamestate(GS_GOAL_KICK, TEAM_MAGENTA);
      break;

    case REFBOX_FIRST_HALF:
      __rss.set_half(HALF_FIRST);
      break;

    case REFBOX_SECOND_HALF:
      __rss.set_half(HALF_SECOND);
      break;

    case REFBOX_HALF_TIME:
      __rss.set_gamestate(GS_HALF_TIME, TEAM_BOTH);
      break;

    default:
      printf("Received unknown command: '%c'\n", buf[b]);
      break;
    }    
  }

  __rss.send();
}


/** Run.
 * Reads messages from the network, processes them and calls the refbox state sender.
 */
void
MidsizeRefBoxRepeater::run()
{
  char tmpbuf[100];
  while ( ! __quit ) {
    size_t bytes_read = __s->read(tmpbuf, sizeof(tmpbuf), /* read all */ false);
    if ( bytes_read == 0 ) {
      // seems that the remote has died, reconnect
      printf("Connection died, reconnecting\n");
      reconnect();
    } else {
      process_string(tmpbuf, bytes_read);
    }
  }
}
