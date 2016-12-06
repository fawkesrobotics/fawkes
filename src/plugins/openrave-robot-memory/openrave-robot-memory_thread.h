
/***************************************************************************
 *  openrave-robot-memory_thread.h - openrave-robot-memory
 *
 *  Created: Thu Nov 24 13:14:33 2016
 *  Copyright  2016  Frederik Zwilling
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

#ifndef __PLUGINS_OPENRAVE_ROBOT_MEMORY_THREAD_H_
#define __PLUGINS_OPENRAVE_ROBOT_MEMORY_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/configurable.h>
#include <interfaces/OpenRaveInterface.h>
#include <interfaces/OpenraveRobotMemoryInterface.h>
#include <plugins/robot-memory/aspect/robot_memory_aspect.h>
#include <list>
#include <algorithm>

namespace fawkes {
  // add forward declarations here, e.g., interfaces
}

class OpenraveRobotMemoryThread 
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::RobotMemoryAspect,
  public fawkes::BlackBoardAspect
{

 public:
  OpenraveRobotMemoryThread();

  virtual void init();
  virtual void finalize();
  virtual void loop();

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() { Thread::run(); }

 private:
  fawkes::OpenRaveInterface* openrave_if_;
  fawkes::OpenraveRobotMemoryInterface* or_rm_if_;
  std::list<std::string> added_objects_;
  std::list<std::string> added_object_types_;

  void construct_scene();
};


#endif
