
/***************************************************************************
 *  act_thread.h - Robotis dynamixel servo actuator integration
 *
 *  Created: Mon Mar 23 20:23:33 2015 (based on pantilt plugin)
 *  Copyright  2006-2015  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_DYNAMIXEL_ACT_THREAD_H_
#define __PLUGINS_DYNAMIXEL_ACT_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>

class DynamixelDriverThread;

class DynamixelActThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::SyncPointAspect
{
 public:
  DynamixelActThread();
  virtual void loop();

  void add_driver_thread(DynamixelDriverThread *drv_thread);

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  std::list<DynamixelDriverThread *> driver_threads_;

};


#endif
