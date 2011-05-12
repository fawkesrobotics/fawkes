
/***************************************************************************
 *  message_handler_thread.h - OpenRAVE interface messages handler thread
 *
 *  Created: Fri Feb 25 15:08:00 2011
 *  Copyright  2011  Bahram Maleki-Fard
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

#ifndef __PLUGINS_OPENRAVE_MESSAGE_HANDLER_THREAD_H_
#define __PLUGINS_OPENRAVE_MESSAGE_HANDLER_THREAD_H_

#include "openrave_thread.h"

#include <core/threading/thread.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blocked_timing.h>
#include <aspect/blackboard.h>

namespace fawkes {
  class OpenRaveInterface;
}

class OpenRaveMessageHandlerThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::BlackBoardAspect
{
 public:
  OpenRaveMessageHandlerThread(OpenRaveThread* or_thread);
  virtual ~OpenRaveMessageHandlerThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  OpenRaveThread*		__or_thread;
  fawkes::OpenRaveInterface*	__if_openrave;
};

#endif
