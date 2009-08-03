
/***************************************************************************
 *  thread.h - Fawkes ball position logger thread - for demonstration
 *
 *  Created: Thu Jan 24 17:01:54 2008
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

#ifndef __PLUGINS_BALLPOSLOG_THREAD_H_
#define __PLUGINS_BALLPOSLOG_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>

namespace fawkes {
  class ObjectPositionInterface;
}

class BallPosLogThread
  : public fawkes::Thread,
    public fawkes::BlockedTimingAspect,
    public fawkes::LoggingAspect,
    public fawkes::ConfigurableAspect,
    public fawkes::BlackBoardAspect
{
 public:
  BallPosLogThread();
  virtual ~BallPosLogThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  fawkes::ObjectPositionInterface *wm_ball_interface;
  fawkes::Logger::LogLevel         log_level;
};

#endif
