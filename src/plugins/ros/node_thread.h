
/***************************************************************************
 *  node_thread.h - ROS node handle providing thread
 *
 *  Created: Thu May 05 18:35:12 2011
 *  Copyright  2006-2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_ROS_NODE_THREAD_H_
#define __PLUGINS_ROS_NODE_THREAD_H_

#include <core/threading/thread.h>
#include <core/utils/lockptr.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/clock.h>
#include <aspect/blocked_timing.h>
#include <aspect/aspect_provider.h>
#include <plugins/ros/aspect/ros_inifin.h>
#include <utils/time/time.h>

#include <sys/types.h>

namespace ros {
  class NodeHandle;
  class AsyncSpinner;
}

class ROSNodeThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ClockAspect,
  public fawkes::AspectProviderAspect
{
 public:
  ROSNodeThread();
  virtual ~ROSNodeThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
 protected: virtual void run() { Thread::run(); }

 private:
  bool cfg_async_spinning_;
  unsigned int cfg_async_num_threads_;

  fawkes::LockPtr<ros::NodeHandle>  rosnode_;
  fawkes::ROSAspectIniFin           ros_aspect_inifin_;

  ros::AsyncSpinner                *async_spinner_;
};

#endif
