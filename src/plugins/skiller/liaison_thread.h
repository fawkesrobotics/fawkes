
/***************************************************************************
 *  liaison_thread.h - Fawkes Skiller: Liaison Thread
 *
 *  Created: Mon Feb 18 10:23:05 2008
 *  Copyright  2006-2008  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_SKILLER_LIAISON_THREAD_H_
#define __PLUGINS_SKILLER_LIAISON_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/clock.h>

#include <core/utils/lock_list.h>
#include <blackboard/interface_observer.h>
#include <blackboard/interface_listener.h>

namespace fawkes {
  class Barrier;
  class ObjectPositionInterface;
  class NavigatorInterface;
  class SkillerInterface;
  class GameStateInterface;
  class HumanoidMotionInterface;
  class SwitchInterface;
  class SpeechSynthInterface;
}
class SkillerExecutionThread;

class SkillerLiaisonThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardInterfaceObserver,
  public fawkes::BlackBoardInterfaceListener
{
 friend class SkillerExecutionThread;
 public:
  SkillerLiaisonThread(fawkes::Barrier *liaison_exec_barrier);
  virtual ~SkillerLiaisonThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  /* BlackBoardInterfaceObserver */
  virtual void bb_interface_created(const char *type, const char *id) throw();

  /* BlackBoardInterfaceListener */
  void bb_interface_reader_removed(fawkes::Interface *interface, unsigned int instance_serial) throw();

  void set_execthread(SkillerExecutionThread *set);

 private:
  void init_failure_cleanup();

 private:
  fawkes::Barrier *__liaison_exec_barrier;
  SkillerExecutionThread *__exec_thread;

  fawkes::SkillerInterface *skiller;
  //fawkes::ObjectPositionInterface *wm_ball_w;
  fawkes::ObjectPositionInterface *wm_ball;
  fawkes::ObjectPositionInterface *wm_pose;
  //fawkes::ObjectPositionInterface *wm_pose_w;
  fawkes::ObjectPositionInterface *nao_ball;
  fawkes::NavigatorInterface      *navigator;
  fawkes::NavigatorInterface      *nao_navigator;
  fawkes::GameStateInterface      *gamestate;
  fawkes::HumanoidMotionInterface *hummot;
  fawkes::SwitchInterface         *chbut;
  fawkes::SwitchInterface         *tballrec;
  fawkes::SpeechSynthInterface    *speechsynth;

  fawkes::LockList<fawkes::ObjectPositionInterface *>  wm_obstacles;
  fawkes::LockList<fawkes::ObjectPositionInterface *>::iterator  wm_obs_it;
};


#endif
