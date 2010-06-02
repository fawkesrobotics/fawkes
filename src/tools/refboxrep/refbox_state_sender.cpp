
/***************************************************************************
 *  refbox_state_sender.cpp - Fawkes RefBox state sender
 *
 *  Created: Wed Apr 09 10:19:27 2008
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

#include "refbox_state_sender.h"

#include <netcomm/worldinfo/transceiver.h>
#include <core/macros.h>

#include <cstdio>

using namespace fawkes;

/** @class RefBoxStateSender "refbox_state_sender.h"
 * RefBox repeater state sender.
 * Adapter to the WorldInfoTransceiver, provides easy optional debugging output
 * to stdout.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param addr multicast address to send information to and receive from
 * @param port UDP port to send information to and receive from
 * @param key encryption key
 * @param iv encryption initialisation vector
 * @param debug true to enable debug output
 */
RefBoxStateSender::RefBoxStateSender(const char *addr, unsigned short port,
				     const char *key, const char *iv,
				     bool debug)
{
  __debug = debug;

  __transceiver = new WorldInfoTransceiver(WorldInfoTransceiver::MULTICAST, addr, port, key, iv);
  __transceiver->set_loop( true );

  __game_state = GS_FROZEN;
  __state_team = TEAM_BOTH;
  __score_cyan = 0;
  __score_magenta = 0;
  __our_team = TEAM_CYAN;
  __our_goal_color = GOAL_BLUE;
  __half = HALF_FIRST;
  __timeout_thread = NULL;
}

/** Constructor.
 * Only to be used by derivatives. These must implement the send() method!
 */
RefBoxStateSender::RefBoxStateSender()
{
  __debug = false;
  __transceiver = NULL;
  __game_state = GS_FROZEN;
  __state_team = TEAM_BOTH;
  __score_cyan = 0;
  __score_magenta = 0;
  __our_team = TEAM_CYAN;
  __our_goal_color = GOAL_BLUE;
  __half = HALF_FIRST;
  __timeout_thread = NULL;
}


/** Destructor. */
RefBoxStateSender::~RefBoxStateSender()
{
  if ( __timeout_thread ) {
    __timeout_thread->cancel();
    __timeout_thread->join();
    delete __timeout_thread;
  }
  delete __transceiver;
}


/** Set current game state.
 * @param game_state current game state
 * @param state_team team referenced by the game state
 */
void
RefBoxStateSender::set_gamestate(int game_state,
				 worldinfo_gamestate_team_t state_team)
{
  if ( __debug ) {
    printf("Setting gamestate to '%d' for team '%s'\n",
	   game_state, worldinfo_gamestate_team_tostring(state_team));
  }

  __game_state = game_state;
  __state_team = state_team;
}


/** Set score.
 * @param score_cyan current score of team cyan
 * @param score_magenta current score of team magenta
 */
void
RefBoxStateSender::set_score(unsigned int score_cyan, unsigned int score_magenta)
{
  if ( __debug ) {
    printf("Setting score to %u:%u (cyan:magenta)\n", score_cyan, score_magenta);
  }
  __score_cyan = score_cyan;
  __score_magenta = score_magenta;
}


/** Set team and goal info.
 * @param our_team our team color
 * @param goal_color our goal color
 */
void
RefBoxStateSender::set_team_goal(worldinfo_gamestate_team_t our_team,
				 worldinfo_gamestate_goalcolor_t goal_color)
{
  if ( __debug ) {
    printf("Setting team color to '%s' and goal color to '%s'\n",
	   worldinfo_gamestate_team_tostring(our_team),
	   worldinfo_gamestate_goalcolor_tostring(goal_color));
  }
  __our_team = our_team;
  __our_goal_color = goal_color;
}


/** Set current half of the game time.
 * @param half current half
 */
void
RefBoxStateSender::set_half(worldinfo_gamestate_half_t half)
{
  if ( __debug ) {
    printf("Setting half to '%s'\n",
	   worldinfo_gamestate_half_tostring(half));
  }
  __half = half;
}


/** Add penalty.
 * @param player number of the player to add the penalty for
 * @param penalty penalty code
 * @param seconds_remaining estimated time when the penalty will be lifted
 */
void
RefBoxStateSender::add_penalty(unsigned int player, unsigned int penalty,
			       unsigned int seconds_remaining)
{
  rss_penalty_t p;
  p.player            = player;
  p.penalty           = penalty;
  p.seconds_remaining = seconds_remaining;
}


/** Send worldinfo. */
void
RefBoxStateSender::send()
{
  if ( __debug ) {
    printf("Sending worldinfo\n");
  }

  if ( __timeout_thread ) {
    __timeout_thread->cancel();
    __timeout_thread->join();
    delete __timeout_thread;
  }
  __timeout_thread = new RefBoxStateSender::TimeoutThread(this);
  __timeout_thread->start();
}


/** Execute send operation.
 * Called by internal timeout thread.
 */
void
RefBoxStateSender::execute_send()
{
  if (unlikely(! __transceiver)) {
    return;
  } else {
    __transceiver->set_gamestate(__game_state, __state_team);
    __transceiver->set_score(__score_cyan, __score_magenta);
    __transceiver->set_team_goal(__our_team, __our_goal_color);
    __transceiver->set_half(__half);
    for (__pit = __penalties.begin(); __pit != __penalties.end(); ++__pit) {
      __transceiver->add_penalty(__pit->second.player, __pit->second.penalty,
				 __pit->second.seconds_remaining);
    }
    __penalties.clear();
    __transceiver->send();
  }
}

/** @class RefBoxStateSender::TimeoutThread <tools/refboxrep/refbox_state_sender.h>
 * Timeout thread.
 * This thread sends out a burst of world info packages if new information has been set
 * for sending and will then slow down until only one packet per second is sent.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param rss parent refbox state sender
 */
RefBoxStateSender::TimeoutThread::TimeoutThread(RefBoxStateSender *rss)
  : Thread("RefBoxStateSender::TimeoutThread", Thread::OPMODE_CONTINUOUS)
{
  __timeout_usec = 0;
  __rss = rss;
}


/** Destructor. */
RefBoxStateSender::TimeoutThread::~TimeoutThread()
{
}


void
RefBoxStateSender::TimeoutThread::loop()
{
  __rss->execute_send();

  switch (__timeout_usec) {
  case      0: __timeout_usec =       1; break;
  case      1: __timeout_usec =       2; break;
  case      2: __timeout_usec =   50000; break;
  //case  50000: __timeout_usec =  250000; break;
  //case 250000: __timeout_usec =  500000; break;
  //case 500000: __timeout_usec = 1000000; break;
  }

  usleep(__timeout_usec);
}
