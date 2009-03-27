
/***************************************************************************
 *  wm_thread.h - Fawkes WorldModel Plugin Thread
 *
 *  Created: Fri Jun 29 11:54:58 2007 (on flight to RoboCup 2007, Atlanta)
 *  Copyright  2006-2007  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_WORLDMODEL_WM_THREAD_H_
#define __PLUGINS_WORLDMODEL_WM_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lock_list.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/network.h>
#include <aspect/clock.h>

namespace fawkes {
  class WorldInfoTransceiver;
  class ObjectPositionInterface;
}

class WorldModelNetworkThread;
class WorldModelFuser;

class WorldModelThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ClockAspect,
  public fawkes::NetworkAspect
{
 public:
  WorldModelThread(WorldModelNetworkThread *net_thread);
  virtual ~WorldModelThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

  void init_failure_cleanup();

 private:
  std::string   __cfg_confspace;

  WorldModelNetworkThread *__net_thread;

  std::list<WorldModelFuser *>           __fusers;
  std::list<WorldModelFuser *>::iterator __fit;

  bool __wi_send_enabled;
  fawkes::ObjectPositionInterface* __wi_send_ball;
  fawkes::ObjectPositionInterface* __wi_send_pose;

};


#endif
