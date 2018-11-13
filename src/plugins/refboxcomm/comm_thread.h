
/***************************************************************************
 *  comm_thread.h - Fawkes RefBox Communication Thread
 *
 *  Created: Sun Apr 19 13:10:29 2009
 *  Copyright  2009  Tim Niemueller [www.niemueller.de]
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

#ifndef _PLUGINS_REFBOXCOMM_COMM_THREAD_H_
#define _PLUGINS_REFBOXCOMM_COMM_THREAD_H_

#include "processor/state_handler.h"

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>

namespace fawkes {
  class GameStateInterface;
  class SwitchInterface;
#ifdef HAVE_SPL
  class SoccerPenaltyInterface;
#endif
}

class RefBoxProcessor;

class RefBoxCommThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public RefBoxStateHandler
{
 public:
  RefBoxCommThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  // RefBoxStateHandler
  virtual void set_gamestate(int game_state,
			     fawkes::worldinfo_gamestate_team_t state_team);
  virtual void set_score(unsigned int score_cyan, unsigned int score_magenta);
  virtual void set_team_goal(fawkes::worldinfo_gamestate_team_t our_team,
			     fawkes::worldinfo_gamestate_goalcolor_t goal_color);
  virtual void set_half(fawkes::worldinfo_gamestate_half_t half, bool kickoff);
  virtual void add_penalty(unsigned int penalty,
                           unsigned int seconds_remaining);

  virtual void handle_refbox_state();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private: /* methods */

 private: /* members */
  fawkes::GameStateInterface   *gamestate_if_;
#ifdef HAVE_SPL
  fawkes::SoccerPenaltyInterface  *penalty_if_;
#endif
  RefBoxProcessor              *refboxproc_;

  bool         gamestate_modified_;
  int          last_gamestate_;
  fawkes::worldinfo_gamestate_half_t  last_half_;
  bool kickoff_;
  unsigned int last_score_cyan_;
  unsigned int last_score_magenta_;
  fawkes::worldinfo_gamestate_team_t our_team_;
  fawkes::worldinfo_gamestate_goalcolor_t our_goal_color_;
  unsigned int team_number_;
  unsigned int player_number_;

  bool cfg_beep_on_change_;
  float cfg_beep_frequency_;
  float cfg_beep_duration_;
  fawkes::SwitchInterface *beep_if_;
};


#endif
