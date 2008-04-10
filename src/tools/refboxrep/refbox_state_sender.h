
/***************************************************************************
 *  refbox_state_sender.h - Fawkes RefBox state sender
 *
 *  Created: Wed Apr 09 09:56:57 2008
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

#ifndef __TOOLS_REFBOXREP_REFBOX_STATE_SENDER_H_
#define __TOOLS_REFBOXREP_REFBOX_STATE_SENDER_H_

#include <core/threading/thread.h>
#include <netcomm/worldinfo/enums.h>

class WorldInfoTransceiver;

class RefBoxStateSender
{
 public:
  RefBoxStateSender(const char *addr, unsigned short port,
		    const char *key, const char *iv,
		    bool debug = false);
  ~RefBoxStateSender();

  void send();
  void execute_send();
  void set_gamestate(worldinfo_gamestate_t game_state, worldinfo_gamestate_team_t state_team);
  void set_score(unsigned int score_cyan, unsigned int score_magenta);
  void set_team_goal(worldinfo_gamestate_team_t our_team,
		     worldinfo_gamestate_goalcolor_t goal_color);
  void set_half(worldinfo_gamestate_half_t half);

  class TimeoutThread : public Thread
  {
   public:
    TimeoutThread(RefBoxStateSender *rss);
    virtual ~TimeoutThread();
    virtual void loop();
   private:
    unsigned int __timeout_usec;
    RefBoxStateSender *__rss;
  };

 private:
  bool                    __debug;
  WorldInfoTransceiver   *__transceiver;
  TimeoutThread          *__timeout_thread;

  worldinfo_gamestate_t           __game_state;
  worldinfo_gamestate_team_t      __state_team;
  unsigned int                    __score_cyan;
  unsigned int                    __score_magenta;
  worldinfo_gamestate_team_t      __our_team;
  worldinfo_gamestate_goalcolor_t __our_goal_color;
  worldinfo_gamestate_half_t      __half;
};

#endif
