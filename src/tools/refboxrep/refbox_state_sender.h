
/***************************************************************************
 *  refbox_state_sender.h - Fawkes RefBox state sender
 *
 *  Created: Wed Apr 09 09:56:57 2008
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

#ifndef __TOOLS_REFBOXREP_REFBOX_STATE_SENDER_H_
#define __TOOLS_REFBOXREP_REFBOX_STATE_SENDER_H_

#include <core/threading/thread.h>
#include <netcomm/worldinfo/enums.h>
#include <map>

namespace fawkes {
  class WorldInfoTransceiver;
}

class RefBoxStateSender
{
 public:
  RefBoxStateSender(const char *addr, unsigned short port,
		    const char *key, const char *iv,
		    bool debug = false);
  virtual ~RefBoxStateSender();

  virtual void send();
  virtual void set_gamestate(int game_state,
			     fawkes::worldinfo_gamestate_team_t state_team);
  virtual void set_score(unsigned int score_cyan, unsigned int score_magenta);
  virtual void set_team_goal(fawkes::worldinfo_gamestate_team_t our_team,
			     fawkes::worldinfo_gamestate_goalcolor_t goal_color);
  virtual void set_half(fawkes::worldinfo_gamestate_half_t half);
  virtual void add_penalty(unsigned int player, unsigned int penalty,
			   unsigned int seconds_remaining);

  class TimeoutThread : public fawkes::Thread
  {
   public:
    TimeoutThread(RefBoxStateSender *rss);
    virtual ~TimeoutThread();
    virtual void loop();
   private:
    unsigned int __timeout_usec;
    RefBoxStateSender *__rss;
  };

 protected:
  RefBoxStateSender();

 private:
  void execute_send();

 private:
  bool                                    __debug;
  fawkes::WorldInfoTransceiver           *__transceiver;
  TimeoutThread                          *__timeout_thread;

  int                                     __game_state;
  fawkes::worldinfo_gamestate_team_t      __state_team;
  unsigned int                            __score_cyan;
  unsigned int                            __score_magenta;
  fawkes::worldinfo_gamestate_team_t      __our_team;
  fawkes::worldinfo_gamestate_goalcolor_t __our_goal_color;
  fawkes::worldinfo_gamestate_half_t      __half;

  /// @cond INTERNALS
  typedef struct {
    unsigned int player;
    unsigned int penalty;
    unsigned int seconds_remaining;
  } rss_penalty_t;
  /// @endcond
  std::map<unsigned int, rss_penalty_t> __penalties;
  std::map<unsigned int, rss_penalty_t>::iterator __pit;

};

#endif
